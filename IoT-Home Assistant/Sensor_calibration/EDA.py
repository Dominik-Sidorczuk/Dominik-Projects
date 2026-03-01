import pandas as pd
import numpy as np
import scipy.signal
import scipy.stats
import json
import logging
import warnings
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import List, Dict, Optional, Any
from statsmodels.graphics.tsaplots import plot_acf, plot_pacf

# Configure structured logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("HardwareProfiler")
warnings.filterwarnings("ignore", category=RuntimeWarning)

# High-fidelity plotting configuration
plt.style.use('dark_background')
sns.set_theme(
    style="darkgrid",
    rc={
        "axes.facecolor": "#121212",
        "figure.facecolor": "#121212",
        "text.color": "white",
        "axes.labelcolor": "white",
        "xtick.color": "white",
        "ytick.color": "white",
        "grid.color": "#333333"
    }
)

class HardwareProfiler:
    """
    Thermal Profiler for Sensor Diagnostics.
    Calculates physical phase-shifts (thermal lag) via cross-correlation,
    analyzes temporal dependencies via PACF, and isolates noise floors utilizing Huber bounds.
    """
    
    def __init__(self, data_path: Path, target_sensors: List[str], ref_tmp: str, ref_sht: str, freq_s: int = 12):
        self.data_path = data_path
        self.target_sensors = target_sensors
        self.freq_s = freq_s
        self.ref_tmp = ref_tmp
        self.ref_sht = ref_sht
        self.hardware_profile: Dict[str, Any] = {"devices": {}}
        self.output_dir = data_path.parent
        
    def execute(self) -> None:
        """Main execution pipeline."""
        if not self.data_path.exists():
            logger.error(f"Dataset not found at: {self.data_path}")
            return
            
        logger.info(f"Ingesting raw telemetry from {self.data_path.name}")
        df_grid = self._ingest_and_pivot()
        if df_grid is None or df_grid.empty:
            logger.error("Data pipeline collapsed. Insufficient valid data grids.")
            return
            
        logger.info(f"Synthesizing Ground Truth Reference...")
        df_grid = self._synthesize_truth(df_grid)
        if 'truth' not in df_grid.columns:
            logger.error("Failed to extract Ground Truth from references.")
            return

        for sensor in self.target_sensors:
            if sensor not in df_grid.columns:
                logger.warning(f"Sensor {sensor} completely missing from grid. Skipping.")
                continue
                
            self._profile_sensor(df_grid, sensor)
            
        self._export_artifacts()
        
    def _ingest_and_pivot(self) -> Optional[pd.DataFrame]:
        """Loads and aligns raw states to a synchronous physical grid."""
        try:
            df = pd.read_csv(self.data_path, usecols=['entity_id', 'state', 'last_changed'])
            df['entity_id'] = df['entity_id'].str.replace('sensor.', '', regex=False)
            
            # If the user only has one reference sensor, ref_tmp and ref_sht can be identically named in external config
            valid_sensors = set(self.target_sensors + [self.ref_tmp, self.ref_sht])
            df = df[df['entity_id'].isin(valid_sensors)].copy()
            
            df['last_changed'] = pd.to_datetime(df['last_changed'], format='ISO8601', errors='coerce', utc=True)
            df = df.dropna(subset=['last_changed'])
            # Exclude Unix Epoch and early-stage anomalies
            df = df[df['last_changed'] >= '2023-01-01']
            
            df['state'] = pd.to_numeric(df['state'], errors='coerce')
            df = df.dropna(subset=['state'])
            
            freq_str = f'{self.freq_s}s'
            df_pivot = df.pivot_table(index='last_changed', columns='entity_id', values='state')
            return df_pivot.resample(freq_str).mean().interpolate(method='linear')
        except Exception as e:
            logger.exception(f"Error during data ingestion: {str(e)}")
            return None
            
    def _synthesize_truth(self, df_grid: pd.DataFrame) -> pd.DataFrame:
        """Fuses multiple reference sensors into a single physical truth."""
        if self.ref_tmp in df_grid.columns and self.ref_sht in df_grid.columns:
            df_grid['truth'] = df_grid[[self.ref_tmp, self.ref_sht]].mean(axis=1)
        elif self.ref_tmp in df_grid.columns:
            df_grid['truth'] = df_grid[self.ref_tmp]
        elif self.ref_sht in df_grid.columns:
            df_grid['truth'] = df_grid[self.ref_sht]
        return df_grid

    def _profile_sensor(self, df_grid: pd.DataFrame, sensor: str) -> None:
        """Executes full mathematical profiling and visual mapping for a specific sensor."""
        df_sensor = df_grid.dropna(subset=['truth', sensor]).copy()
        if df_sensor.empty:
            logger.warning(f"Insufficient temporal overlap between truth and {sensor}.")
            return
            
        logger.info(f"Extracting physical limits for: {sensor}")
        
        # 1. Cross-Correlation for Thermal Phase-Shift (Lag)
        t_norm = df_sensor['truth'] - df_sensor['truth'].mean()
        s_norm = df_sensor[sensor] - df_sensor[sensor].mean()
        
        corr = scipy.signal.correlate(t_norm, s_norm, mode='full')
        lags = scipy.signal.correlation_lags(len(t_norm), len(s_norm))
        
        max_idx = int(np.argmax(corr))
        optimal_lag_samples = lags[max_idx]
        
        thermal_lag_s = float(abs(optimal_lag_samples * self.freq_s))
        
        # 2. Robust Statistical Floor (Huber Alpha via Median Absolute Deviation)
        err = (df_sensor['truth'] - df_sensor[sensor]).dropna()
        abs_err = err.abs()
        mad = scipy.stats.median_abs_deviation(abs_err)
        median_err = float(np.median(abs_err))
        huber_alpha = float(median_err + 3 * mad)
        
        self.hardware_profile["devices"][sensor] = {
            "thermal_lag_s": round(thermal_lag_s, 2),
            "huber_alpha": round(huber_alpha, 3)
        }
        
        # Dashboard visualizations
        self._generate_dashboard(sensor, lags, corr, max_idx, thermal_lag_s, err, huber_alpha)
        self._generate_hysteresis_plot(df_sensor, sensor, err)
        self._generate_acf_pacf_plot(df_sensor, sensor)

    def _generate_dashboard(self, sensor: str, lags: np.ndarray, corr: np.ndarray, 
                            max_idx: int, thermal_lag_s: float, 
                            err: pd.Series, huber_alpha: float) -> None:
        """Renders analytical plots for phase shift and loss boundaries."""
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        
        # Plot A: Cross-Correlation
        time_lags = lags * self.freq_s
        axes[0].plot(time_lags, corr, color='#00ffcc', alpha=0.8, linewidth=2)
        axes[0].axvline(x=time_lags[max_idx], color='#ff0055', linestyle='--', linewidth=2, 
                        label=f'Extracted Phase-Shift: {thermal_lag_s}s')
        axes[0].set_title(f'Thermal Phase-Shift (Cross-Correlation)\n{sensor}', fontsize=12, pad=15)
        axes[0].set_xlabel('Time Lag (seconds)', fontsize=10)
        axes[0].set_ylabel('Signal Correlation Amplitude', fontsize=10)
        axes[0].set_xlim(-2000, 2000)
        axes[0].legend(loc='upper right', frameon=True, facecolor='#222222')
        
        # Plot B: Error Histograms bounds
        sns.histplot(err, bins=120, kde=True, ax=axes[1], color='#a020f0', edgecolor='none', alpha=0.5)
        axes[1].axvline(x=0, color='white', linestyle='-', linewidth=1, alpha=0.5)
        axes[1].axvline(x=huber_alpha, color='#ffcc00', linestyle='--', linewidth=2, 
                        label=f'+ Huber Boundary ({huber_alpha:.3f}°C)')
        axes[1].axvline(x=-huber_alpha, color='#ffcc00', linestyle='--', linewidth=2, 
                        label=f'- Huber Boundary')
        
        axes[1].set_title('Loss Topography: Huber Isolation Bounds', fontsize=12, pad=15)
        axes[1].set_xlabel('∆ Temperature (Reference - Sensor) [°C]', fontsize=10)
        axes[1].set_ylabel('Kernel Density', fontsize=10)
        axes[1].legend(loc='upper right', frameon=True, facecolor='#222222')
        
        plt.tight_layout()
        plot_path = self.output_dir / f"EDA_HardwareProfile_{sensor}.png"
        plt.savefig(plot_path, dpi=200, bbox_inches='tight')
        plt.close()

    def _generate_hysteresis_plot(self, df_sensor: pd.DataFrame, sensor: str, err: pd.Series) -> None:
        """Renders Phase-Space Hysteresis to show thermal asymmetric properties."""
        df_plot = df_sensor.copy()
        
        # Digital derivative to isolate heating/cooling phases
        smooth_truth = df_plot['truth'].rolling(10, center=True).mean()
        df_plot['Truth_Velocity'] = smooth_truth.diff()
        
        df_plot['Phase'] = 'Stable'
        df_plot.loc[df_plot['Truth_Velocity'] > 0.01, 'Phase'] = 'Heating'
        df_plot.loc[df_plot['Truth_Velocity'] < -0.01, 'Phase'] = 'Cooling'
        df_plot['Error_Raw'] = err

        fig, ax = plt.subplots(figsize=(10, 8))
        
        sns.scatterplot(
            data=df_plot[df_plot['Phase'] != 'Stable'], 
            x='truth', 
            y='Error_Raw', 
            hue='Phase', 
            palette={'Heating': '#ff3366', 'Cooling': '#00ffcc'},
            alpha=0.6,
            s=20,
            edgecolor='none',
            ax=ax
        )
        
        ax.axhline(0, color='white', linestyle='--', alpha=0.5)
        ax.set_title(f'Phase-Space Hysteresis Evidence\n{sensor}', fontsize=14, pad=15)
        ax.set_xlabel('Ground Truth Temperature [°C]', fontsize=12)
        ax.set_ylabel('Sensor Error (Truth - Sensor) [°C]', fontsize=12)
        ax.legend(title="Thermal Regime", loc='upper right', frameon=True, facecolor='#222222')
        
        plt.tight_layout()
        plot_path = self.output_dir / f"EDA_Hysteresis_{sensor}.png"
        plt.savefig(plot_path, dpi=200, bbox_inches='tight')
        plt.close()

    def _generate_acf_pacf_plot(self, df_sensor: pd.DataFrame, sensor: str) -> None:
        """Autocorrelation and Partial Autocorrelation Analysis for feature design."""
        fig, axes = plt.subplots(2, 1, figsize=(14, 10))
        
        # Use first 5000 valid samples downsampled to 1 minute increments to easily visualize long-term lags
        # downsample specifically for ACF plotting
        sig = df_sensor[sensor].resample('1min').mean().interpolate(method='linear').dropna()
        if len(sig) > 2000:
            sig = sig.iloc[:2000]

        # ACF
        plot_acf(sig, ax=axes[0], lags=60, color='#00ffcc', vlines_kwargs={"colors": '#00ffcc'})
        axes[0].set_title(f'Autocorrelation Function (ACF)\n{sensor}', fontsize=12)
        axes[0].set_xlabel('Lag (Minutes)')
        axes[0].set_ylabel('Correlation')

        # PACF
        plot_pacf(sig, ax=axes[1], lags=60, color='#ff3366', vlines_kwargs={"colors": '#ff3366'})
        axes[1].set_title(f'Partial Autocorrelation Function (PACF)\n{sensor}', fontsize=12)
        axes[1].set_xlabel('Lag (Minutes)')
        axes[1].set_ylabel('Correlation')
        
        plt.tight_layout()
        plot_path = self.output_dir / f"EDA_ACF_PACF_{sensor}.png"
        plt.savefig(plot_path, dpi=200, bbox_inches='tight')
        plt.close()

    def _export_artifacts(self) -> None:
        """Writes the computed parameters out to JSON."""
        out_path = self.output_dir / "hardware_profile.json"
        try:
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(self.hardware_profile, f, indent=4)
            logger.info(f"Hardware profile exported to: {out_path.name}")
        except IOError as e:
            logger.error(f"Failed to export matrix: {e}")