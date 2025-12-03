from __future__ import annotations
import numpy as np
from numba import njit

# --- NUMBA KERNELS ---

@njit(fastmath=True, cache=True)
def _numba_process_actions(
    actions_raw: np.ndarray, 
    action_space_n: int
) -> np.ndarray:
    """Process actions: NaN check, cast to int, clip, return"""
    n = actions_raw.size
    out = np.empty(n, dtype=np.int32)
    max_idx = action_space_n - 1
    
    # Ensure raw actions are treated as floats for NaN checks
    raw_flat = actions_raw.astype(np.float64)
    
    for i in range(n):
        val = raw_flat[i]
        
        if np.isnan(val) or val < 0.0: 
             val = 0.0
        
        ival = np.int32(val)
        if ival < 0: ival = 0
        elif ival > max_idx: ival = max_idx
        
        out[i] = ival
    return out

@njit(fastmath=True, cache=True)
def _numba_store_step(
    # Buffers
    b_obs: np.ndarray,
    b_act: np.ndarray,
    b_rew: np.ndarray,
    b_term: np.ndarray,
    b_trunc: np.ndarray,
    # Data
    step_idx: int,
    obs: np.ndarray,
    act: np.ndarray,
    rew: np.ndarray,
    term: np.ndarray,
    trunc: np.ndarray
) -> np.ndarray:
    """Szybki zapis kroku do bufora NumPy i obliczenie maski 'done'."""
    # Direct copy (no allocation)
    b_obs[step_idx] = obs
    b_act[step_idx] = act
    b_rew[step_idx] = rew
    
    # Fast bool->float cast and store
    for i in range(len(term)):
        t_val = 1.0 if term[i] else 0.0
        tr_val = 1.0 if trunc[i] else 0.0
        b_term[step_idx, i] = t_val
        b_trunc[step_idx, i] = tr_val
        
    # Return dones mask for RNN reset (term | trunc)
    dones = np.logical_or(term, trunc)
    return dones