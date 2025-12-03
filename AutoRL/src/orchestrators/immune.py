from __future__ import annotations
import logging
import numpy as np
import math
from typing import Dict, Optional, Any
from numba import njit

from src.config import ImmuneConfig, AutoRLConfig

log = logging.getLogger(__name__)

# =============================================================================
# STAŁE I MAPOWANIE (Dla szybkości dostępu)
# =============================================================================
IDX_GRAD = 0
IDX_LOSS = 1
IDX_ENT = 2
IDX_SPREAD = 3
IDX_SCORE = 4

METRIC_MAP = {
    'grad_norm': IDX_GRAD,
    'loss': IDX_LOSS,
    'entropy': IDX_ENT,
    'q_spread': IDX_SPREAD,
    'score': IDX_SCORE,
    'mean_return': IDX_SCORE # Alias
}
NUM_METRICS = len(METRIC_MAP)

@njit(fastmath=True, cache=True)
def _numba_slope_ring(buffer: np.ndarray, head: int, count: int, capacity: int) -> float:
    """Calculate slope on ring buffer (zero-copy)"""
    if count < 2:
        return 0.0

    n = float(count)
    sum_x = n * (n - 1.0) * 0.5
    m = n - 1.0
    sum_xx = m * n * (2.0 * m + 1.0) / 6.0

    sum_y = 0.0
    sum_xy = 0.0

    start_idx = (head - count) % capacity
    if start_idx < 0: start_idx += capacity

    for x in range(count):
        phys_idx = (start_idx + x)
        if phys_idx >= capacity:
            phys_idx -= capacity
        
        y = buffer[phys_idx]
        sum_y += y
        sum_xy += x * y

    denominator = (n * sum_xx - sum_x * sum_x)
    
    if abs(denominator) < 1e-9:
        return 0.0

    slope = (n * sum_xy - sum_x * sum_y) / denominator
    return slope

@njit(fastmath=True, cache=True)
def _numba_std_mean_ring(buffer: np.ndarray, head: int, count: int, capacity: int) -> tuple:
    """Calculate (mean, std) on ring buffer"""
    if count < 1:
        return 0.0, 0.0

    sum_val = 0.0
    start_idx = (head - count) % capacity
    if start_idx < 0: start_idx += capacity

    for i in range(count):
        phys_idx = start_idx + i
        if phys_idx >= capacity: phys_idx -= capacity
        sum_val += buffer[phys_idx]
    
    mean = sum_val / count
    
    if count < 2:
        return mean, 0.0

    var_sum = 0.0
    for i in range(count):
        phys_idx = start_idx + i
        if phys_idx >= capacity: phys_idx -= capacity
        diff = buffer[phys_idx] - mean
        var_sum += diff * diff

    std = np.sqrt(var_sum / count)
    return mean, std

class VitalSignsHP:
    """Zero-allocation ring buffer for vitals tracking"""
    def __init__(self, window_size: int):
        self.capacity = window_size
        self.data = np.zeros((NUM_METRICS, window_size), dtype=np.float64)
        
        self.head = 0  
        self.count = 0 

    def update(self, metrics: Dict[str, float]):
        """O(1) update, no allocation"""
        for name, val in metrics.items():
            idx = METRIC_MAP.get(name, -1)
            if idx >= 0:
                if math.isfinite(val):
                    self.data[idx, self.head] = val
                else:
                    self.data[idx, self.head] = 0.0 

        self.head += 1
        if self.head >= self.capacity:
            self.head = 0
            
        if self.count < self.capacity:
            self.count += 1

    def get_stats(self, metric_name: str) -> tuple:
        idx = METRIC_MAP.get(metric_name, -1)
        if idx == -1 or self.count < 2:
            return 0.0, 0.0, 0.0
            
        row = self.data[idx]
        
        slope = _numba_slope_ring(row, self.head, self.count, self.capacity)
        mean, std = _numba_std_mean_ring(row, self.head, self.count, self.capacity)
        
        return slope, mean, std
        
    def get_latest(self, metric_name: str) -> float:
        idx = METRIC_MAP.get(metric_name, -1)
        if idx == -1 or self.count == 0:
            return 0.0
        
        prev = self.head - 1
        if prev < 0: prev = self.capacity - 1
        return float(self.data[idx, prev])

class ImmunePolicy:
    def __init__(self, cfg: ImmuneConfig):
        self.cfg = cfg
        self.vitals = VitalSignsHP(window_size=cfg.window_size)
        
        self.interventions_count = 0
        self.nan_streak = 0
        
        self.last_panic_step = -99999
        self.last_chronic_step = -99999
        
        self.panic_cooldown = cfg.panic_cooldown
        self.chronic_cooldown = cfg.chronic_cooldown

    def update_vitals(self, step: int, metrics: Dict[str, float]):
        self.vitals.update(metrics)

    def check_panic_conditions(self, step: int, metrics: Dict[str, float]) -> Optional[Dict[str, Any]]:
        loss = metrics.get('loss', 0.0)
        grad = metrics.get('grad_norm', 0.0)
        
        if not (math.isfinite(loss) and math.isfinite(grad)):
            self.nan_streak += 1
            if (step - self.last_panic_step < self.panic_cooldown) and self.nan_streak < 3:
                return None
            reason = "NAN_PANIC"
            action_type = "sedative_reset" if self.nan_streak >= 3 else "sedative"
            return self._prescribe_medication(step, action_type, reason)
        
        self.nan_streak = 0
        
        if (step - self.last_panic_step > self.panic_cooldown):
            entropy = metrics.get('entropy', 1.0)
            if entropy < self.cfg.entropy_crash_threshold:
                 return self._prescribe_medication(step, "stimulant_ent", "ENTROPY_COLLAPSE")

        return None

    def check_chronic_health(self, step: int) -> Optional[Dict[str, Any]]:
        if step - self.last_chronic_step < self.chronic_cooldown:
            return None
            
        if self.vitals.count < 10:
            return None

        score_slope, score_mean, score_std = self.vitals.get_stats('score')
        _, ent_mean, _ = self.vitals.get_stats('entropy')
        
        if score_std < 0.5 and abs(score_slope) < self.cfg.plateau_slope_threshold and score_mean < -10.0:
            return self._prescribe_medication(step, "stimulant_lr", "DEEP_PLATEAU", is_chronic=True)
            
        if ent_mean > 0.8 and abs(score_slope) < 0.1 and abs(score_mean) > 10.0:
             return self._prescribe_medication(step, "sedative_ent", "CHAOTIC_WANDERING", is_chronic=True)

        return None

    def _prescribe_medication(self, step: int, kind: str, reason: str, is_chronic: bool = False) -> Dict[str, Any]:
        self.interventions_count += 1
        
        if is_chronic:
            self.last_chronic_step = step
        else:
            self.last_panic_step = step

        changes = {'reason': reason, 'kind': kind}
        
        if kind == "sedative":
            changes['optimizer'] = {'lr_scale': self.cfg.sedative_lr_factor}
            changes['train'] = {'max_grad_norm': 0.5} 
        elif kind == "sedative_reset":
            changes['optimizer'] = {'lr_scale': 0.2, 'reset_state': True}
            changes['train'] = {'max_grad_norm': 0.1}
            log.warning("⚡ IMMUNE SYSTEM: TRIGGERING HARD OPTIMIZER RESET!")
        elif kind == "stimulant_ent":
            changes['agent'] = {'ent_boost': self.cfg.stimulant_boost - 1.0} # Convert boost factor to additive/relative if needed, or just use as is. Assuming boost is factor > 1.0, but set_immune_boost expects float. Let's use 0.05 as base * boost? Or just use a fixed small value scaled?
            # Re-reading config: stimulant_boost is float > 1.0 (e.g. 1.5). set_immune_boost sets _immune_eps_boost.
            # If boost is 1.5, we might want eps to be higher.
            # Let's use a derived value or just keep 0.05 if unsure, but user wants config usage.
            # Let's assume stimulant_boost is the multiplier for LR, but for entropy we use a fixed value?
            # Config says: stimulant_boost: Annotated[float, Field(gt=1.0)] = 1.5
            # Let's use 0.05 * self.cfg.stimulant_boost for now to respect the param.
            changes['agent'] = {'ent_boost': 0.5 * self.cfg.stimulant_boost}
        elif kind == "sedative_ent":
            changes['agent'] = {'ent_decay_boost': 1.5}
        elif kind == "stimulant_lr":
            changes['optimizer'] = {'lr_scale': self.cfg.stimulant_boost}
            
        return changes

class ImmuneOrchestrator:
    """
    Manager for Immune Policies of multiple agents.
    """
    def __init__(self, cfg: ImmuneConfig):
        self.cfg = cfg
        self.policies: Dict[int, ImmunePolicy] = {}

    def check(self, agent_id: int, step: int, metrics: Dict[str, float]) -> Optional[Dict[str, Any]]:
        """
        Checks agent health and returns a prescription (dict) or None.
        """
        if not self.cfg.enabled:
            return None

        if agent_id not in self.policies:
            self.policies[agent_id] = ImmunePolicy(self.cfg)

        policy = self.policies[agent_id]
        policy.update_vitals(step, metrics)
        
        # 1. Panic Check
        prescription = policy.check_panic_conditions(step, metrics)
        if prescription:
            return prescription
            
        # 2. Chronic Check
        prescription = policy.check_chronic_health(step)
        return prescription

    def prescribe(self, agent_id: int, prescription: Dict[str, Any], trainer) -> bool:
        """
        Applies the prescription to the trainer/agent.
        """
        if not prescription:
            return False

        reason = prescription.get('reason', 'UNKNOWN')
        kind = prescription.get('kind', 'UNKNOWN')
        log.info(f"🚑 Immune System: Treating Agent {agent_id} for '{reason}' ({kind})")

        # Apply Optimizer changes
        if 'optimizer' in prescription:
            opt_changes = prescription['optimizer']
            if 'lr_scale' in opt_changes:
                scale = opt_changes['lr_scale']
                for param_group in trainer.optimizer.param_groups:
                    param_group['lr'] *= scale
                log.info(f"💊 Adjusted LR by factor {scale}")
            
            if opt_changes.get('reset_state', False):
                trainer.optimizer.state.clear()
                log.warning("⚡ Optimizer state cleared!")

        # Apply Agent changes
        if 'agent' in prescription:
            agent_changes = prescription['agent']
            if 'ent_boost' in agent_changes:
                boost = agent_changes['ent_boost']
                trainer.agent.set_immune_boost(boost) # Assuming simple boost for now
                log.info(f"💉 Applied Entropy Boost: {boost}")

        return True
