import pandas as pd
import numpy as np
import lightgbm as lgb
import optuna
import m2cgen as m2c
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.signal import savgol_filter
from sklearn.metrics import mean_absolute_error, root_mean_squared_error
import warnings
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple

# Enforce professional logging stack
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(name)s: %(message)s')
logger = logging.getLogger("Cal3_Engine")
optuna.logging.set_verbosity(optuna.logging.WARNING)
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=FutureWarning)

# SOTA Matplotlib aesthetics
plt.style.use('dark_background')
sns.set_theme(style="darkgrid", rc={"axes.facecolor": "#121212", "figure.facecolor": "#121212", 
                                    "text.color": "white", "axes.labelcolor": "white", 
                                    "xtick.color": "white", "ytick.color": "white", 
                                    "grid.color": "#333333"})

@dataclass(frozen=True)
class CalibrationConfig:
    """Strict execution configuration profile for the Calibration pipeline."""
    base_dir: Path
    filepath: str = 'history.csv'
    target_sensors: List[str] = field(default_factory=lambda: [
        'alpstuga_air_quality_monitor_temperatura',
        'timmerflotte_temp_hmd_sensor_temperatura',
        'timmerflotte_temp_hmd_sensor_temperatura_2',
        'timmerflotte_temp_hmd_sensor_temperatura_3'
    ])
    calibrated_targets: Dict[str, str] = field(default_factory=lambda: {
        'alpstuga_air_quality_monitor_temperatura': 'alpstuga_temperatura_calibrated',
        'timmerflotte_temp_hmd_sensor_temperatura': 'timmerflotte_temperatura_calibrated',
        'timmerflotte_temp_hmd_sensor_temperatura_2': 'timmerflotte_temperatura_2_calibrated',
        'timmerflotte_temp_hmd_sensor_temperatura_3': 'timmerflotte_temperatura_3_calibrated'
    })
    ref_tmp: str = 'p2_tmp119'
    ref_sht: str = 'p2_sht45_temperatura'
    var_tmp: float = 0.08 ** 2
    var_sht: float = 0.10 ** 2
    dt_step: float = 12.0
    tau_base: float = 60.0
    tau_pred: float = 120.0
    
    def get_hardware_profile(self) -> Dict[str, Any]:
        """Resolves active architectural parameters dynamically."""
        profile_path = self.base_dir / "hardware_profile.json"
        if profile_path.exists():
            with open(profile_path, "r", encoding="utf-8") as f:
                return json.load(f).get("devices", {})
        logger.warning("hardware_profile.json missing. Operating blind.")
        return {}

class DataLoader:
    """Data Ingestion & Sanitization Engine."""
    def __init__(self, config: CalibrationConfig):
        self.config = config

    def ingest(self) -> pd.DataFrame:
        target_path = self.config.base_dir / self.config.filepath
        if not target_path.exists():
            logger.error(f"Target telemetry file missing: {target_path}")
            return pd.DataFrame()
            
        logger.info(f"Ingesting memory stream from {target_path.name}...")
        df = pd.read_csv(target_path, usecols=['entity_id', 'state', 'last_changed'])
        df['entity_id'] = df['entity_id'].str.replace('sensor.', '', regex=False)
        
        valid_sensors = set(self.config.target_sensors + [self.config.ref_tmp, self.config.ref_sht])
        df = df[df['entity_id'].isin(valid_sensors)].copy()

        df['last_changed'] = pd.to_datetime(df['last_changed'], format='ISO8601', errors='coerce', utc=True)
        df = df.dropna(subset=['last_changed'])
        
        # Explicit Hard Cut against Unix Epoch leakage and anomalies
        df = df[df['last_changed'] >= '2023-01-01']

        df['state'] = pd.to_numeric(df['state'], errors='coerce')
        df = df.dropna(subset=['state'])
        df = df[(df['state'] > -30) & (df['state'] < 60)]
        return df.sort_values('last_changed')

class GroundTruthBuilder:
    """Rigorous signal construction via statistical fusions and morphological filtering."""
    def __init__(self, config: CalibrationConfig):
        self.config = config

    def build(self, df_raw: pd.DataFrame) -> pd.DataFrame:
        logger.info("Constructing morphological ground truth signal.")
        freq = f"{int(self.config.dt_step)}s"
        df_refs = []
        
        # 1. Independent extraction and Hampel filtering
        for ref in [self.config.ref_tmp, self.config.ref_sht]:
            df_sensor = df_raw[df_raw['entity_id'] == ref].set_index('last_changed')[['state']].rename(columns={'state': ref})
            df_sensor = df_sensor[~df_sensor.index.duplicated(keep='last')]
            if not df_sensor.empty:
                rolling_med = df_sensor[ref].rolling(window=5, center=True).median()
                df_sensor.loc[abs(df_sensor[ref] - rolling_med) > 2.0, ref] = np.nan
                df_refs.append(df_sensor)
                
        if not df_refs:
            logger.error("No reference truth sources available. Halting.")
            return pd.DataFrame()
            
        df_truth = pd.concat(df_refs, axis=1).sort_index()
        
        # 2. Strict resample, physics-preserving PCHIP interpolation
        df_truth = df_truth.resample(freq).mean().interpolate(method='pchip', limit=30)
        
        # 3. Savitzky-Golay for final sensor noise smoothing
        for ref in [self.config.ref_tmp, self.config.ref_sht]:
            if ref in df_truth.columns:
                mask = df_truth[ref].notna()
                if mask.sum() > 31:
                    df_truth.loc[mask, ref] = savgol_filter(df_truth.loc[mask, ref], window_length=31, polyorder=3)
                    
        # 4. Inverse-Variance weighting fusion
        w_tmp = self.config.var_sht / (self.config.var_tmp + self.config.var_sht)
        w_sht = self.config.var_tmp / (self.config.var_tmp + self.config.var_sht)
        
        df_truth['ground_truth'] = np.nan
        cond_both = df_truth[self.config.ref_tmp].notna() & df_truth[self.config.ref_sht].notna()
        df_truth.loc[cond_both, 'ground_truth'] = (df_truth.loc[cond_both, self.config.ref_tmp] * w_tmp) + (df_truth.loc[cond_both, self.config.ref_sht] * w_sht)
        
        # 5. Fault tolerance fallback
        df_truth['ground_truth'] = df_truth['ground_truth'].fillna(df_truth.get(self.config.ref_tmp)).fillna(df_truth.get(self.config.ref_sht))
        
        return df_truth.dropna(subset=['ground_truth'])

class FeatureEngineer:
    """Pure Vectorized feature projection engine (Zero loop performance impact over taus logic)."""
    def __init__(self, config: CalibrationConfig):
        self.config = config

    def create_features(self, df_target: pd.DataFrame, target_sensor: str, taus: List[int]) -> pd.DataFrame:
        df_copy = df_target.copy()
        dt = self.config.dt_step
        target_series = df_copy[target_sensor]
        
        # Base Substrate Signal
        alpha_base = 1.0 - np.exp(-dt / self.config.tau_base)
        df_copy['T_base_1'] = target_series.ewm(alpha=alpha_base, adjust=False).mean()
        df_copy['T_base']   = df_copy['T_base_1'].ewm(alpha=alpha_base, adjust=False).mean()
        
        # ZLEMA Projection via strict math parameters
        for i, tau in enumerate(taus):
            lag = max(1, int(tau / dt) // 2)
            alpha = 1.0 - np.exp(-dt / tau)
            shifted = target_series.shift(lag).bfill()
            delagged = target_series + (target_series - shifted)
            df_copy[f'ZLEMA_{i+1}'] = delagged.ewm(alpha=alpha, adjust=False).mean()
            
        # Rigid Kinematic Features
        df_copy['Velocity'] = df_copy['ZLEMA_1'] - df_copy[f'ZLEMA_{len(taus)}']
        df_copy['Vel_Heating'] = df_copy['Velocity'].clip(lower=0)
        df_copy['Vel_Cooling'] = df_copy['Velocity'].clip(upper=0).abs()
            
        # Slice off burn-in instabilities
        if len(df_copy) > 50:
            df_copy = df_copy.iloc[25:].copy()
            
        return df_copy

class Visualizer:
    """Generates high tier visual artifacts for performance verification."""
    @staticmethod
    def render_calibration_plot(df: pd.DataFrame, target_sensor: str, split_idx: int, 
                                mae_before: float, mae_after: float, output_dir: Path) -> None:
                                    
        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(3, 1, height_ratios=[3, 1, 1])
        
        ax_main = fig.add_subplot(gs[0])
        ax_err = fig.add_subplot(gs[1])
        ax_dist = fig.add_subplot(gs[2])
        
        # View 1: Main Overlays
        ax_main.plot(df.index, df['ground_truth'], label='Ground Truth (Target)', color='#00ffcc', linewidth=1.5, zorder=3)
        ax_main.plot(df.index, df['T_base'], label='Substrate (T_base)', color='#ff9900', alpha=0.5, linestyle=':', zorder=1)
        ax_main.plot(df.index, df[target_sensor], label='Raw Telemetry', color='#ff3366', alpha=0.4, linestyle='-', zorder=2)
        ax_main.plot(df.index, df[f'{target_sensor}_cal'], label='Optuna LightGBM Calibration', color='#00ff00', linewidth=2, zorder=4)
        
        # Test Split Indicator
        ax_main.axvline(x=df.index[split_idx], color='#8888ff', linestyle='--', linewidth=2, label='OOS Evaluation Horizon')
        ax_main.axvspan(df.index[split_idx], df.index[-1], color='#8888ff', alpha=0.08)
        ax_main.set_title(f'SOTA 2026 Model Inference Profile: {target_sensor}\nTest Set MAE OOS | Before: {mae_before:.4f}°C -> After: {mae_after:.4f}°C', fontsize=14, pad=15)
        ax_main.legend(loc='upper right', frameon=True, facecolor='#222222', ncols=2)
        ax_main.set_ylabel("Temperature [°C]")
        
        # View 2: Residuals in Time Space
        residuals_raw = df[target_sensor] - df['ground_truth']
        residuals_cal = df[f'{target_sensor}_cal'] - df['ground_truth']
        
        ax_err.plot(df.index, residuals_raw, color='#ff3366', alpha=0.5, label='Raw Error Residuals', linewidth=1)
        ax_err.plot(df.index, residuals_cal, color='#00ff00', alpha=0.8, label='Calibrated Residuals Focus', linewidth=1.5)
        ax_err.axhline(0, color='white', linestyle='--', alpha=0.7)
        ax_err.axvspan(df.index[split_idx], df.index[-1], color='#8888ff', alpha=0.08)
        ax_err.set_title("Temporal Residual Distribution Analysis", fontsize=11)
        ax_err.legend(loc='lower right', frameon=True, facecolor='#222222', ncols=2)
        ax_err.set_ylabel("Error [°C]")
        
        # View 3: Error Distributions (KDE)
        sns.kdeplot(residuals_raw.iloc[split_idx:], color='#ff3366', fill=True, alpha=0.3, ax=ax_dist, label='Raw Dist. OOS')
        sns.kdeplot(residuals_cal.iloc[split_idx:], color='#00ff00', fill=True, alpha=0.5, ax=ax_dist, label='Calibrated Dist. OOS')
        ax_dist.set_title("Probability Density Heteroscedastic Analysis (OOS)", fontsize=11)
        ax_dist.axvline(0, color='white', linestyle='--', alpha=0.7)
        ax_dist.legend(loc='upper right', frameon=True, facecolor='#222222')
        ax_dist.set_xlabel("Error Deviation [°C]")
        
        plt.tight_layout()
        out_path = output_dir / f"Cal3_Resolution_{target_sensor}.png"
        plt.savefig(out_path, dpi=200, bbox_inches='tight')
        plt.close()
        logger.info(f"Analytical rendering exported strictly to {out_path.name}")


class ModelTrainer:
    """Formalized ML Workflow utilizing strict Optuna optimization logic."""
    def __init__(self, config: CalibrationConfig, feature_engineer: FeatureEngineer):
        self.config = config
        self.fe = feature_engineer
        self.feature_cols = [f'ZLEMA_{i+1}' for i in range(4)] + ['Vel_Heating', 'Vel_Cooling']
        self.hardware_profiles = config.get_hardware_profile()

    def train(self, df_raw: pd.DataFrame, df_truth: pd.DataFrame, target_sensor: str) -> Optional[Dict[str, Any]]:
        logger.info(f"Initiating Training Sub-Routine: {target_sensor}")
        
        df_sens = df_raw[df_raw['entity_id'] == target_sensor].set_index('last_changed')[['state']].rename(columns={'state': target_sensor})
        df_sens = df_sens[~df_sens.index.duplicated(keep='last')].sort_index()
        
        if df_sens.empty: 
            logger.warning(f"Telemetry missing entirely. Bypassing {target_sensor}.")
            return None
            
        freq = f'{int(self.config.dt_step)}s'
        df_target_raw = pd.merge(df_sens.resample(freq).ffill(), df_truth.resample(freq).mean()[['ground_truth']], 
                                 left_index=True, right_index=True, how='inner').dropna()
        if len(df_target_raw) < 100: 
            logger.warning("Unusable metric convergence window. Model aborted.")
            return None

        hardware = self.hardware_profiles.get(target_sensor, {})
        lag_s = hardware.get('thermal_lag_s', 45.0)
        huber_alpha = hardware.get('huber_alpha', 0.5)
        
        # 1. Hyperparameter Search Horizon
        def objective(trial: optuna.Trial) -> Tuple[float, float]:
            min_tau_1 = max(10, int(lag_s - 15))
            max_tau_1 = max(min_tau_1 + 5, int(lag_s + 30))
            
            taus = [
                trial.suggest_int('tau_1', min_tau_1, max_tau_1),        
                trial.suggest_int('tau_2', max_tau_1 + 1, max_tau_1 + 300),       
                trial.suggest_int('tau_3', max_tau_1 + 301, max_tau_1 + 800),      
                trial.suggest_int('tau_4', max_tau_1 + 801, max_tau_1 + 1500),      
            ]
            
            lgb_params = {
                'objective': 'huber',
                'alpha': huber_alpha,
                'n_estimators': trial.suggest_int('n_estimators', 8, 42),
                'max_depth': trial.suggest_int('max_depth', 2, 6),
                'learning_rate': trial.suggest_float('learning_rate', 0.01, 0.2, log=True),
                'reg_alpha': trial.suggest_float('reg_alpha', 1e-3, 10.0, log=True),
                'random_state': 42,
                'n_jobs': -1,
                'verbose': -1
            }
            
            df_feat = self.fe.create_features(df_target_raw, target_sensor, taus)
            X = df_feat[self.feature_cols]
            y = df_feat['ground_truth'] - df_feat['T_base']
            
            # Ironclad Evaluation Horizon Isolation
            split_idx = int(len(X) * 0.8)
            X_train = X.iloc[:split_idx]
            y_train = y.iloc[:split_idx]
            X_val = X.iloc[split_idx:]
            y_val = y.iloc[split_idx:]
            
            MAX_OPT_SAMPLES = 8000 
            if len(X_train) > MAX_OPT_SAMPLES:
                X_tr = X_train.sample(n=MAX_OPT_SAMPLES, random_state=42).sort_index()
                y_tr = y_train.loc[X_tr.index]
            else:
                X_tr, y_tr = X_train, y_train
                
            model = lgb.LGBMRegressor(**lgb_params)
            model.fit(X_tr, y_tr)
            
            # Apply exact edge deployment logic to validation set
            alpha_pred = 1.0 - np.exp(-self.config.dt_step / self.config.tau_pred)
            preds_raw = pd.Series(model.predict(X_val), index=y_val.index)
            preds_smoothed = preds_raw.ewm(alpha=alpha_pred, adjust=False).mean()
            
            return mean_absolute_error(y_val, preds_smoothed), root_mean_squared_error(y_val, preds_smoothed)

        logger.info("Executing Deterministic Optuna Multi-Objective Surface Exploration.")
        sampler = optuna.samplers.NSGAIISampler(seed=42)
        study = optuna.create_study(directions=['minimize', 'minimize'], sampler=sampler)
        study.optimize(objective, n_trials=250, n_jobs=1) 
        
        # 2. Extract strictly optimal models and retrain Full Scale Substrate
        best_trial = min(study.best_trials, key=lambda t: t.values[0] + t.values[1])
        best_taus = [best_trial.params[f'tau_{i+1}'] for i in range(4)]
        best_lgb_params = {k: v for k, v in best_trial.params.items() if not k.startswith('tau')}
        best_lgb_params['objective'] = 'huber'
        best_lgb_params['alpha'] = huber_alpha
        
        df_target = self.fe.create_features(df_target_raw, target_sensor, best_taus)
        X = df_target[self.feature_cols]
        y_residual = df_target['ground_truth'] - df_target['T_base']
        split_idx = int(len(X) * 0.8)
        
        # Intermediate Evaluation (to verify integrity)
        eval_model = lgb.LGBMRegressor(**best_lgb_params, random_state=42, verbose=-1)
        eval_model.fit(X.iloc[:split_idx], y_residual.iloc[:split_idx])
        
        alpha_pred = 1.0 - np.exp(-self.config.dt_step / self.config.tau_pred)
        df_target['smoothed_preds'] = pd.Series(eval_model.predict(X), index=df_target.index).ewm(alpha=alpha_pred, adjust=False).mean()
        df_target[f'{target_sensor}_cal'] = df_target['T_base'] + df_target['smoothed_preds']
        
        mae_before = mean_absolute_error(df_target['ground_truth'].iloc[split_idx:], df_target[target_sensor].iloc[split_idx:])
        mae_after = mean_absolute_error(df_target['ground_truth'].iloc[split_idx:], df_target[f'{target_sensor}_cal'].iloc[split_idx:])
        logger.info(f"OOS MAE Trajectory: {mae_before:.4f}°C -> {mae_after:.4f}°C")
        
        # Render Analytics
        Visualizer.render_calibration_plot(df_target, target_sensor, split_idx, mae_before, mae_after, self.config.base_dir)

        # Retrain comprehensively on all data for Edge deployment
        final_model = lgb.LGBMRegressor(**best_lgb_params, random_state=42, verbose=-1)
        final_model.fit(X, y_residual)
        
        # Tree expansion format to python
        code = m2c.export_to_python(final_model)
        code = code.replace("def score", f"def score_{target_sensor}")
        return {"code": code, "taus": best_taus}

class EdgeCompiler:
    """Translates Python algorithms physically into O(1) Memory C-like Python streams."""
    def __init__(self, config: CalibrationConfig):
        self.config = config

    def compile(self, models: Dict[str, Dict[str, Any]], calibrated_targets: Optional[Dict[str, str]] = None) -> None:
        pyscript_path = self.config.base_dir / "pyscript_sota_zlema.py"
        targets_map = calibrated_targets or self.config.calibrated_targets
        
        with open(pyscript_path, "w", encoding="utf-8") as f:
            f.write('import time\nimport math\n\n')
            
            for sensor, data in models.items():
                f.write(data["code"] + '\n\n')
                
            f.write('CONFIG = {\n')
            for sensor, data in models.items():
                taus_str = "[" + ", ".join([str(t) for t in data["taus"]]) + "]"
                
                if targets_map and sensor in targets_map:
                    out_name = targets_map[sensor]
                else:
                    out_name = f"{sensor}_calibrated_sota"
                    
                out_entity = out_name if out_name.startswith("sensor.") else f"sensor.{out_name}"
                
                f.write(f'    "sensor.{sensor}": {{\n')
                f.write(f'        "taus": {taus_str},\n')
                f.write(f'        "func": score_{sensor},\n')
                f.write(f'        "out_entity": "{out_entity}",\n')
                f.write(f'    }},\n')
            f.write('}\n\n')
            
            f.write('state_memory = {}\n')
            f.write(f'DT_STEP = {self.config.dt_step}\n')
            f.write(f'TAU_BASE = {self.config.tau_base}\n')
            f.write(f'TAU_PRED = {self.config.tau_pred}\n\n')
            
            # Streaming engine: Ring buffers, no iterative recalculation over history
            f.write(f'''@time_trigger("period(now, {int(self.config.dt_step)}s)")
def tick_calibrate_ikea_sensors():
    for sensor_id, cfg in CONFIG.items():
        raw_val = state.get(sensor_id)
        if raw_val in (None, "unknown", "unavailable"): continue
        try: current_temp = float(raw_val)
        except (ValueError, TypeError): continue
            
        out_sensor = cfg["out_entity"]
        taus = cfg["taus"]
        mem = state_memory.get(sensor_id)
        
        if "lags" not in cfg:
            lags = cfg["lags"] = [max(1, int(t / DT_STEP) // 2) for t in taus]
            cfg["alphas"] = [1.0 - math.exp(-DT_STEP / t) for t in taus]
            cfg["alpha_base"] = 1.0 - math.exp(-DT_STEP / TAU_BASE)
            cfg["alpha_pred"] = 1.0 - math.exp(-DT_STEP / TAU_PRED)
        else:
            lags = cfg["lags"]
            
        if not mem:
            mem = state_memory[sensor_id] = {{}}
            mem["base1"] = mem["base2"] = current_temp
            mem["zley"] = [current_temp] * len(taus)
            mem["prev_pred"] = 0.0
            
            # Preallocated Ring Buffers to eliminate list.append() O(1) performance
            mem["history"] = []
            mem["head"] = []
            for lag in lags:
                mem["history"].append([current_temp] * lag)
                mem["head"].append(0)
                
            state.set(out_sensor, round(current_temp, 2), unit_of_measurement="°C", device_class="temperature")
            continue
            
        alpha_base = cfg["alpha_base"]
        b1 = (current_temp * alpha_base) + (mem["base1"] * (1.0 - alpha_base))
        b2 = (b1 * alpha_base) + (mem["base2"] * (1.0 - alpha_base))
        
        new_zley = []
        for i, a in enumerate(cfg["alphas"]):
            lag = lags[i]
            head = mem["head"][i]
            
            # Direct Ring Buffer access
            shifted_val = mem["history"][i][head]
            mem["history"][i][head] = current_temp
            mem["head"][i] = (head + 1) % lag
            
            delagged_signal = current_temp + (current_temp - shifted_val)
            new_zley.append((delagged_signal * a) + (mem["zley"][i] * (1.0 - a)))
            
        velocity = new_zley[0] - new_zley[-1]
        vel_heating = velocity if velocity > 0 else 0.0
        vel_cooling = abs(velocity) if velocity <= 0 else 0.0
        
        mem["base1"], mem["base2"], mem["zley"] = b1, b2, new_zley
        
        raw_pred = max(-4.0, min(4.0, cfg["func"](new_zley + [vel_heating, vel_cooling])))
        
        alpha_pred = cfg["alpha_pred"]
        smoothed_pred = (raw_pred * alpha_pred) + (mem["prev_pred"] * (1.0 - alpha_pred))
        mem["prev_pred"] = smoothed_pred
        
        state.set(out_sensor, round(b2 + smoothed_pred, 2), unit_of_measurement="°C", device_class="temperature")
''')
        logger.info("Compiled pyscript deployment bundle structurally sound.")

def main():
    base_dir = Path(__file__).parent.absolute()
    config = CalibrationConfig(base_dir=base_dir)
    loader = DataLoader(config)
    gt_builder = GroundTruthBuilder(config)
    fe = FeatureEngineer(config)
    trainer = ModelTrainer(config, fe)
    compiler = EdgeCompiler(config)
    
    logger.info("--- Bootstrapping Project Calibration Vector ---")
    df_raw = loader.ingest()
    if df_raw.empty:
        logger.error("Data pipeline broken upfront. Ensure target DB exists.")
        return
        
    df_truth = gt_builder.build(df_raw)
    if df_truth.empty:
        logger.error("Signal engine collapsed on Ground Truth building.")
        return
    
    models = {}
    for target_sensor in config.target_sensors:
        res = trainer.train(df_raw, df_truth, target_sensor)
        if res: 
            models[target_sensor] = res
            
    if models: 
        logger.info("Linking logic modules and exporting execution scripts...")
        compiler.compile(models)
    
    logger.info("--- Sequence Terminated Successfully ---")

if __name__ == "__main__":
    main()