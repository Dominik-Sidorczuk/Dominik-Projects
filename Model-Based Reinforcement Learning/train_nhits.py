# -*- coding: utf-8 -*-
"""
Zrefaktoryzowany i zoptymalizowany potok treningowy dla modelu NHITS.
Wersja 33: Usunięto logikę wizualizacji, aby oddzielić proces treningu
od analizy wyników.
"""
import pandas as pd
import numpy as np
import torch
import torch.multiprocessing as mp
from torch.utils.checkpoint import checkpoint
from pyarrow.parquet import ParquetFile
import pyarrow as pa
import torch.profiler
import optuna
import os
import joblib
import logging
import argparse
import json
from dataclasses import dataclass, asdict
from types import SimpleNamespace
from typing import List, Optional, Dict, Any, Tuple, Generator
import gc
import polars as pl
import pytorch_lightning as L
import sys

# --- Biblioteki ML/DL ---
from sklearn.metrics import mean_absolute_error
from sklearn.preprocessing import StandardScaler
from neuralforecast import NeuralForecast
from neuralforecast.models import NHITS
from neuralforecast.losses.pytorch import MQLoss
from pytorch_lightning.profilers import PyTorchProfiler
from neuralforecast.common._base_model import BaseModel
from neuralforecast.models.nhits import NHITSBlock, _IdentityBasis

# --- Konfiguracja logowania ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s',
    force=True
)

# --- Zaawansowane zarządzanie VRAM ---
if torch.cuda.is_available():
    alloc_conf_var = 'PYTORCH_CUDA_ALLOC_CONF'
    if torch.version.hip:
        alloc_conf_var = 'PYTORCH_HIP_ALLOC_CONF'
        logging.info("Wykryto środowisko ROCm/HIP dla GPU AMD.")
    os.environ[alloc_conf_var] = 'expandable_segments:True'
    logging.info(f"Ustawiono {alloc_conf_var} w celu potencjalnej redukcji fragmentacji VRAM.")
    torch.set_float32_matmul_precision('high')
    logging.info(f"CUDA/HIP dostępne. Używane urządzenie: {torch.cuda.get_device_name(0)}")
else:
    logging.warning("CUDA/HIP niedostępne. Trening zostanie przeprowadzony na CPU.")

# --- Struktury danych ---
@dataclass
class PreparedData:
    initial_train_path: str
    fold_paths: List[str]
    test_path: str
    hpo_sample_df: pl.DataFrame
    used_covariates: List[str]
    covariate_scaler: Optional[StandardScaler] = None

@dataclass
class TrainingArtifacts:
    model_wrapper: NeuralForecast
    best_params: Dict[str, Any]
    final_mae: Optional[float]
    fold_maes: Dict[int, float]

# --- Poprawiony, wydajny pamięciowo Data Handler ---
class StreamingDataHandler:
    def __init__(self, config: SimpleNamespace):
        self.config = config
        self.log = logging.getLogger(self.__class__.__name__)
        self.save_dir = getattr(config, 'NHITS_SAVE_DIR', './nhits_artifacts')
        os.makedirs(self.save_dir, exist_ok=True)

    def _prepare_and_save_files(self, df_long: pd.DataFrame, used_covariates: List[str]):
        df_long = df_long.sort_values('ds').reset_index(drop=True)
        
        n = len(df_long['ds'].unique())
        unique_dates = sorted(df_long['ds'].unique())
        
        initial_train_end_idx = int(n * self.config.INITIAL_TRAIN_RATIO)
        test_start_idx = int(n * (1.0 - self.config.TEST_SET_RATIO))

        initial_train_end_date = unique_dates[initial_train_end_idx]
        test_start_date = unique_dates[test_start_idx]

        cv_dates = [d for d in unique_dates if initial_train_end_date < d < test_start_date]
        fold_dates = np.array_split(cv_dates, self.config.CV_FOLDS)

        self.log.info(f"Całkowita liczba unikalnych kroków czasowych: {n}.")
        self.log.info(f"Podział na {self.config.CV_FOLDS} okien walidacyjnych.")

        initial_train_df = df_long[df_long['ds'] <= initial_train_end_date].copy()
        scaler = None
        if used_covariates:
            self.log.info(f"Skalowanie kowariantów na podstawie początkowego zbioru treningowego: {used_covariates}")
            scaler = StandardScaler()
            initial_train_df.loc[:, used_covariates] = scaler.fit_transform(initial_train_df[used_covariates])
        
        initial_train_path = os.path.join(self.save_dir, "train_initial.parquet")
        pl.from_pandas(initial_train_df).write_parquet(initial_train_path, compression='zstd', use_pyarrow=True)
        self.log.info(f"Zapisano początkowy zbiór treningowy ({len(initial_train_df)} wierszy) w: {initial_train_path}")
        
        fold_paths = []
        for i, dates in enumerate(fold_dates):
            if len(dates) == 0: continue
            fold_df = df_long[(df_long['ds'] > dates[0]) & (df_long['ds'] <= dates[-1])].copy()
            if used_covariates and scaler:
                fold_df.loc[:, used_covariates] = scaler.transform(fold_df[used_covariates])
            
            fold_path = os.path.join(self.save_dir, f"fold_{i+1}.parquet")
            pl.from_pandas(fold_df).write_parquet(fold_path, compression='zstd', use_pyarrow=True)
            fold_paths.append(fold_path)
            self.log.info(f"Zapisano okno walidacyjne #{i+1} ({len(fold_df)} wierszy) w: {fold_path}")

        test_df = df_long[df_long['ds'] >= test_start_date].copy()
        if used_covariates and scaler:
            test_df.loc[:, used_covariates] = scaler.transform(test_df[used_covariates])
        test_path = os.path.join(self.save_dir, "test_data.parquet")
        pl.from_pandas(test_df).write_parquet(test_path, compression='zstd', use_pyarrow=True)
        self.log.info(f"Zapisano zbiór testowy ({len(test_df)} wierszy) w: {test_path}")

        hpo_sample_df = pl.from_pandas(initial_train_df).sample(fraction=self.config.HPO_SAMPLE_RATIO, shuffle=True, seed=42)
        self.log.info(f"Utworzono próbkę HPO z {len(hpo_sample_df)} wierszy.")

        return PreparedData(
            initial_train_path=initial_train_path,
            fold_paths=fold_paths,
            test_path=test_path,
            hpo_sample_df=hpo_sample_df,
            used_covariates=used_covariates,
            covariate_scaler=scaler
        )

    def prepare_data_for_pipeline(self) -> PreparedData:
        self.log.info("--- Rozpoczęcie przygotowania danych do strumieniowania ---")
        df_raw = pd.read_parquet(self.config.DATA_PATH)
        for col in df_raw.select_dtypes(include=['float64']).columns:
            df_raw[col] = df_raw[col].astype('float32')
        if not pd.api.types.is_datetime64_any_dtype(df_raw.index):
            df_raw.index = pd.to_datetime(df_raw.index)
        if df_raw.index.tz is not None:
            df_raw.index = df_raw.index.tz_localize(None)
        df_raw.index.name = "ds"
        df_base = df_raw.resample(self.config.FREQ).mean().sort_index()
        for col, prefix in zip(self.config.NHITS_TARGET_COLUMNS, self.config.NHITS_TARGET_PREFIXES):
            df_base[f"{prefix}_delta_target"] = df_base[col].shift(-self.config.PREDICTION_HORIZON) - df_base[col]
        target_delta_cols = [f"{p}_delta_target" for p in self.config.NHITS_TARGET_PREFIXES]
        df_base.dropna(subset=target_delta_cols, inplace=True)
        used_covariates = [col for col in self.config.COVARIATE_COLUMNS_TO_USE if col in df_base.columns]
        
        df_long = pd.melt(
            df_base.reset_index(), 
            id_vars=["ds"] + used_covariates + self.config.NHITS_TARGET_COLUMNS,
            value_vars=target_delta_cols, 
            var_name="unique_id", 
            value_name="y"
        )
        self.log.info(f"Dane przekonwertowane do formatu 'long'. Liczba wierszy: {len(df_long)}")
        return self._prepare_and_save_files(df_long, used_covariates)

# --- Zmodyfikowane klasy modelu i potoku ---
class CheckpointingNHITS(NHITS):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def forward(self, windows_batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        insample_y = windows_batch["insample_y"].squeeze(-1).contiguous()
        insample_mask = windows_batch["insample_mask"].squeeze(-1).contiguous()
        futr_exog, hist_exog, stat_exog = windows_batch.get("futr_exog"), windows_batch.get("hist_exog"), windows_batch.get("stat_exog")
        residuals, insample_mask = insample_y.flip(dims=(-1,)), insample_mask.flip(dims=(-1,))
        forecast = insample_y[:, -1:, None]
        for block in self.blocks:
            backcast, block_forecast = checkpoint(block, insample_y=residuals, futr_exog=futr_exog, hist_exog=hist_exog, stat_exog=stat_exog, use_reentrant=False)
            residuals = (residuals - backcast) * insample_mask
            forecast = forecast + block_forecast
        return forecast

class HyperparameterOptimizer:
    def __init__(self, config: SimpleNamespace, freq: str):
        self.config, self.freq = config, freq
        self.input_size = self.config.PREDICTION_HORIZON * self.config.INPUT_SIZE_MULTIPLIER
        self.log = logging.getLogger(self.__class__.__name__)

    def _build_model_params(self, params: Dict[str, Any]) -> Dict[str, Any]:
        num_stacks = params.get("num_stacks", 2)
        n_blocks = params.get("n_blocks", 1)
        layer_width = params.get("layer_width", 256)
        
        mlp_config_for_stack = [layer_width, layer_width]
        
        return {
            "n_blocks": [n_blocks] * num_stacks, 
            "mlp_units": [mlp_config_for_stack] * num_stacks, 
            "stack_types": ['identity'] * num_stacks, 
            "n_pool_kernel_size": [1] * num_stacks, 
            "n_freq_downsample": [1] * num_stacks, 
            "learning_rate": params["learning_rate"], 
            "batch_size": params["batch_size"], 
            "accumulate_grad_batches": params["accumulate_grad_batches"]
        }

    def _suggest_hpo_params(self, trial: optuna.Trial) -> Dict[str, Any]:
        return {
            "num_stacks": trial.suggest_int("num_stacks", 2, 3), 
            "n_blocks": trial.suggest_int("n_blocks", 1, 2), 
            "layer_width": trial.suggest_categorical("layer_width", [256, 512]), 
            "batch_size": trial.suggest_categorical("batch_size", [128, 256]), 
            "learning_rate": trial.suggest_float("learning_rate", 1e-4, 1e-3, log=True), 
            "accumulate_grad_batches": trial.suggest_categorical("accumulate_grad_batches", [1, 2, 4])
        }

    def _objective(self, trial: optuna.Trial, hpo_train_df: pl.DataFrame, used_covariates: List[str]) -> float:
        torch.cuda.empty_cache(); gc.collect()
        if hpo_train_df.is_empty(): return float('inf')
        try:
            raw_params = self._suggest_hpo_params(trial)
            model_params = self._build_model_params(raw_params)
            windows_batch_size = getattr(self.config, 'WINDOWS_BATCH_SIZE', 1024)
            model = CheckpointingNHITS(h=self.config.PREDICTION_HORIZON, input_size=self.input_size, loss=MQLoss(self.config.NHITS_QUANTILES), max_steps=self.config.NHITS_MAX_STEPS_HPO, hist_exog_list=used_covariates, scaler_type="robust", windows_batch_size=windows_batch_size, **model_params)
            nf = NeuralForecast(models=[model], freq=self.freq)
            cv_df = nf.cross_validation(df=hpo_train_df.to_pandas(), n_windows=self.config.OPTUNA_CV_N_WINDOWS, step_size=self.config.PREDICTION_HORIZON)
            median_col = next((c for c in cv_df.columns if 'q-50' in c or 'median' in c or 'NHITS' in c), None)
            if 'y' not in cv_df.columns or not median_col: return float('inf')

            total_error = 0
            for prefix in self.config.NHITS_TARGET_PREFIXES:
                target_id = f"{prefix}_delta_target"
                series_df = cv_df[cv_df['unique_id'] == target_id]
                if not series_df.empty:
                    error = mean_absolute_error(series_df['y'], series_df[median_col])
                    self.log.info(f"  -> Próba {trial.number}, MAE dla '{prefix}': {error:.4f}")
                    total_error += error
                else:
                    self.log.warning(f"Brak danych dla serii '{target_id}' w wynikach walidacji krzyżowej HPO.")

            if np.isnan(total_error) or np.isinf(total_error) or total_error == 0:
                return float('inf')

            trial.report(total_error, step=0)
            if trial.should_prune(): raise optuna.exceptions.TrialPruned()
            self.log.info(f"Próba {trial.number} zakończona z sumarycznym MAE: {total_error:.4f}")
            return total_error
        except Exception as e:
            self.log.warning(f"Próba HPO {trial.number} nie powiodła się: {type(e).__name__} - {e}", exc_info=True)
            return float('inf')

    def find_best_params(self, data: PreparedData) -> Dict[str, Any]:
        self.log.info("--- Etap: Optymalizacja Hiperparametrów ---")
        if data.hpo_sample_df.is_empty(): return self.get_default_params()
        db_path = f"{self.config.NHITS_STUDY_NAME}.db"
        if os.path.exists(db_path): os.remove(db_path)
        study = optuna.create_study(direction='minimize', study_name=self.config.NHITS_STUDY_NAME, storage=f"sqlite:///{db_path}", pruner=optuna.pruners.MedianPruner())
        study.optimize(lambda trial: self._objective(trial, data.hpo_sample_df, data.used_covariates), n_trials=self.config.OPTUNA_TRIALS, show_progress_bar=True)
        if not study.best_trial or study.best_trial.value is None or np.isinf(study.best_trial.value):
            self.log.error("HPO nie znalazło prawidłowych prób. Używanie domyślnych parametrów.")
            return self.get_default_params()
        self.log.info(f"Najlepsze parametry znalezione przez HPO: {study.best_params}")
        return study.best_params

    def get_default_params(self) -> Dict[str, Any]:
        self.log.warning("Używanie domyślnych, bezpiecznych parametrów.")
        return {"num_stacks": 2, "n_blocks": 1, "layer_width": 256, "learning_rate": 0.001, "batch_size": 512, "accumulate_grad_batches": 4}

class ModelTrainer:
    @staticmethod
    def read_parquet_chunks(path: str, chunk_size: int = 10) -> Generator[pl.DataFrame, None, None]:
        pq_file = ParquetFile(path)
        for i in range(0, pq_file.num_row_groups, chunk_size):
            row_groups_to_read = list(range(i, min(i + chunk_size, pq_file.num_row_groups)))
            if not row_groups_to_read: break
            tables = [pq_file.read_row_group(rg) for rg in row_groups_to_read]
            combined_table = pa.concat_tables(tables)
            yield pl.from_arrow(combined_table)
            del tables, combined_table
            gc.collect()

    def train_on_files(self, model: BaseModel, file_paths: List[str], freq: str):
        self.log = logging.getLogger(self.__class__.__name__)
        nf_trainer = NeuralForecast(models=[model], freq=freq)
        for file_path in file_paths:
            self.log.info(f"Trening na pliku: {file_path}")
            chunk_iterator = self.read_parquet_chunks(file_path, chunk_size=20)
            for i, chunk_df in enumerate(chunk_iterator):
                if chunk_df.is_empty(): continue
                self.log.info(f"  -> Trening na porcji #{i+1} z pliku...")
                nf_trainer.fit(df=chunk_df.to_pandas())
        return nf_trainer

class ModelEvaluator:
    def __init__(self, config: SimpleNamespace):
        self.config = config
        self.log = logging.getLogger(self.__class__.__name__)

    def evaluate(self, model_wrapper: NeuralForecast, data_path: str, used_covariates: List[str]) -> Optional[Tuple[float, pl.DataFrame]]:
        self.log.info(f"--- Ewaluacja na pliku: {data_path} ---")
        try:
            eval_df = pl.read_parquet(data_path)
            if eval_df.is_empty():
                self.log.warning(f"Plik ewaluacyjny {data_path} jest pusty. Pomijanie.")
                return None, None

            if not all(col in eval_df.columns for col in self.config.NHITS_TARGET_COLUMNS):
                 self.log.error(f"Brak wymaganych kolumn bazowych {self.config.NHITS_TARGET_COLUMNS} w pliku {data_path}")
                 return None, None

            futr_df_pd = eval_df.select(['unique_id', 'ds'] + used_covariates).to_pandas()
            futr_df_pd['ds'] = pd.to_datetime(futr_df_pd['ds'])

            predictions_df_pd = model_wrapper.predict(futr_df=futr_df_pd)
            if predictions_df_pd is None or predictions_df_pd.empty:
                self.log.warning("Predykcja zwróciła pustą ramkę danych.")
                return None, None
            
            predictions_df = pl.from_pandas(predictions_df_pd)
            eval_with_preds_df = eval_df.join(predictions_df, on=['unique_id', 'ds'], how='left')

            median_col = next((c for c in eval_with_preds_df.columns if 'q-50' in c or 'NHITS' in c), None)
            if not median_col:
                self.log.error("Nie znaleziono kolumny z medianą predykcji (q-50).")
                return None, None
            
            valid_preds_df = eval_with_preds_df.drop_nulls(subset=[median_col])
            if valid_preds_df.is_empty():
                self.log.warning("Brak poprawnych predykcji do obliczenia MAE.")
                mae = float('nan')
            else:
                mae = mean_absolute_error(valid_preds_df['y'], valid_preds_df[median_col])
            
            self.log.info(f"Ostateczne MAE (na wartościach delta) na zbiorze ewaluacyjnym: {mae:.4f}")
            
            return mae, eval_with_preds_df
        except Exception as e:
            self.log.error(f"Wystąpił błąd podczas ewaluacji na pliku {data_path}: {e}", exc_info=True)
            return None, None

class TrainingOrchestrator:
    def __init__(self, config: SimpleNamespace):
        self.config = config
        self.log = logging.getLogger(self.__class__.__name__)
        self.data_handler = StreamingDataHandler(config)
        FREQ_MAP = {'H': '1h', 'D': '1d', 'W': '1w', 'M': '1mo', 'Q': '1q', 'Y': '1y', 'T': '1min', 'min': '1min', 'S': '1s'}
        self.freq = FREQ_MAP.get(config.FREQ.rstrip('S'), config.FREQ)
        self.optimizer = HyperparameterOptimizer(config, self.freq)
        self.trainer = ModelTrainer()
        self.evaluator = ModelEvaluator(config)
        self.artifact_manager = ArtifactManager(config)

    def run(self):
        self.log.info(">>> START: Potok Treningowy NHITS z Walidacją Kroczącą <<<")
        try:
            prepared_data = self.data_handler.prepare_data_for_pipeline()
            best_raw_params = self.optimizer.find_best_params(prepared_data) if self.config.NHITS_HPO_ENABLED else self.optimizer.get_default_params()
            final_model_params = self.optimizer._build_model_params(best_raw_params)
            self.log.info(f"Używane pełne parametry modelu: {final_model_params}")

            windows_batch_size = getattr(self.config, 'WINDOWS_BATCH_SIZE', 1024)
            model = CheckpointingNHITS(h=self.config.PREDICTION_HORIZON, input_size=self.optimizer.input_size, loss=MQLoss(self.config.NHITS_QUANTILES), max_steps=self.config.NHITS_MAX_STEPS_FINAL, hist_exog_list=prepared_data.used_covariates, scaler_type="robust", windows_batch_size=windows_batch_size, **final_model_params)
            
            self.log.info("--- Etap: Początkowy Trening ---")
            model_wrapper = self.trainer.train_on_files(model, [prepared_data.initial_train_path], self.freq)
            
            fold_maes = {}
            for i, fold_path in enumerate(prepared_data.fold_paths):
                self.log.info(f"--- Walidacja Krocząca: Fold #{i+1} ---")
                mae, _ = self.evaluator.evaluate(model_wrapper, fold_path, prepared_data.used_covariates)
                if mae is not None: 
                    fold_maes[i+1] = mae
                
                self.log.info(f"--- Refit: Dotrenowywanie na danych z Fold #{i+1} ---")
                model_wrapper = self.trainer.train_on_files(model, [fold_path], self.freq)
            
            self.log.info("--- Etap: Ostateczna Ewaluacja na Zbiorze Testowym ---")
            final_mae, _ = self.evaluator.evaluate(model_wrapper, prepared_data.test_path, prepared_data.used_covariates)
            
            artifacts = TrainingArtifacts(model_wrapper=model_wrapper, best_params=final_model_params, final_mae=final_mae, fold_maes=fold_maes)
            self.artifact_manager.save(artifacts, prepared_data)
            self.log.info(">>> SUKCES: Potok treningowy zakończony pomyślnie. <<<")
            self.log.info(f"Aby zwizualizować wyniki, uruchom skrypt 'visualize_results.py' wskazując na katalog: {self.config.NHITS_SAVE_DIR}")

        except Exception as e:
            self.log.error(f"!!! KRYTYCZNY BŁĄD: Potok nie powiódł się: {e}", exc_info=True)
            raise

class ArtifactManager:
    def __init__(self, config: SimpleNamespace):
        self.config = config
        self.log = logging.getLogger(self.__class__.__name__)

    def save(self, artifacts: TrainingArtifacts, data: PreparedData):
        save_dir = self.config.NHITS_SAVE_DIR
        self.log.info(f"--- Etap: Zapisywanie Artefaktów w '{save_dir}' ---")
        os.makedirs(save_dir, exist_ok=True)
        
        # Zapis konfiguracji użytej do treningu
        with open(os.path.join(save_dir, "training_run_config.json"), 'w') as f:
            json.dump(vars(self.config), f, indent=4)

        torch.save(artifacts.model_wrapper.models[0].state_dict(), os.path.join(save_dir, "nhits_model_state.pth"))
        
        safe_params = {k: (v.item() if isinstance(v, np.generic) else v) for k, v in artifacts.best_params.items()}
        
        metadata = {
            "hyperparameters": safe_params, 
            "final_mae": artifacts.final_mae,
            "fold_maes": artifacts.fold_maes,
            "training_timestamp": pd.Timestamp.now().isoformat(),
            "prediction_horizon": self.config.PREDICTION_HORIZON,
            "input_size": self.config.PREDICTION_HORIZON * self.config.INPUT_SIZE_MULTIPLIER,
            "used_covariates": data.used_covariates
        }
        with open(os.path.join(save_dir, "model_metadata.json"), 'w') as f:
            json.dump(metadata, f, indent=4)
        if data.used_covariates and data.covariate_scaler:
            joblib.dump(data.covariate_scaler, os.path.join(save_dir, "nhits_scaler.joblib"))
        joblib.dump(data.used_covariates, os.path.join(save_dir, "nhits_covariate_columns.joblib"))
        self.log.info("Artefakty zapisane pomyślnie.")

def main():
    parser = argparse.ArgumentParser(description="Uruchom potok treningowy NHITS.")
    parser.add_argument('--data-path', type=str, required=True, help='Ścieżka do wejściowego pliku danych.parquet.')
    parser.add_argument('--config-path', type=str, required=True, help='Ścieżka do pliku konfiguracyjnego.json.')
    args = parser.parse_args()
    logging.info(f"Wczytywanie konfiguracji z: {args.config_path}")
    with open(args.config_path, 'r') as f:
        config_dict = json.load(f)
    config_dict['DATA_PATH'], config_dict['CONFIG_PATH'] = args.data_path, args.config_path
    config = SimpleNamespace(**config_dict)
    if len(config.NHITS_TARGET_COLUMNS) != len(config.NHITS_TARGET_PREFIXES): raise ValueError("Liczba kolumn docelowych musi być równa liczbie prefiksów.")
    if not config.NHITS_TARGET_COLUMNS: raise ValueError("Lista `NHITS_TARGET_COLUMNS` nie może być pusta.")
    orchestrator = TrainingOrchestrator(config)
    orchestrator.run()

if __name__ == '__main__':
    try:
        mp.set_start_method('spawn', force=True)
        logging.info("Ustawiono metodę startową multiprocessing na 'spawn'.")
    except RuntimeError as e:
        logging.warning(f"Nie można ustawić metody startowej 'spawn' (prawdopodobnie już ustawiona): {e}.")
    main()
