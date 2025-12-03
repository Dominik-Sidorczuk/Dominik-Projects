import numpy as np
import torch
import torch.nn.functional as F
from numba import njit, prange
from typing import Tuple, Optional

@njit(fastmath=True, cache=True)
def _numba_calculate_evi(
    history_arr: np.ndarray,
    score_mean: float,
    score_std: float,
    score_slope: float,
    curr_score: float,
    curr_entropy: float
) -> float:
    """Compiled EVI metric calculation"""
    if len(history_arr) < 3:
        return 0.0

    raw_z = (curr_score - score_mean) / (score_std + 1e-5)
    if raw_z > 3.0:
        z_score = 3.0
    elif raw_z < -3.0:
        z_score = -3.0
    else:
        z_score = raw_z

    momentum_bonus = np.tanh(score_slope * 0.1) * 3.0
    entropy_penalty = 0.0
    if abs(score_mean) > 5.0:
        if curr_entropy > 0.6 and score_slope <= 0.0:
            entropy_penalty = curr_entropy * 2.0
    plateau_penalty = 0.0
    if score_std < 0.5 and abs(score_slope) < 0.5 and curr_score < -10.0:
        plateau_penalty = 2.0

    return z_score + momentum_bonus - entropy_penalty - plateau_penalty


@njit(fastmath=True, parallel=True, cache=True)
def _numba_nstep_backup(
    rew: np.ndarray,
    term: np.ndarray,
    q_next_values: np.ndarray,
    gamma: float
) -> np.ndarray:
    """Parallel n-step backup (Bellman)"""
    B, T = rew.shape
    targets = np.empty((B, T), dtype=np.float32)

    for b in prange(B):
        next_val = q_next_values[b, -1]

        for t in range(T - 1, -1, -1):
            r_t = rew[b, t]
            
            if term[b, t] > 0.5:
                target_val = r_t
                next_val = 0.0
            else:
                target_val = r_t + gamma * next_val
                next_val = target_val
            
            targets[b, t] = target_val

    return targets


@njit(fastmath=True, parallel=True, cache=True)
def _numba_reduce_distribution(q_dist: np.ndarray) -> np.ndarray:
    """Reduce C51 distribution: [B,T,A,Atoms] -> [B,T]"""
    B, T, A, Atoms = q_dist.shape
    q_reduced = np.empty((B, T), dtype=np.float32)

    for b in prange(B):
        for t in range(T):
            best_action_val = -1e10
            
            for a in range(A):
                atom_sum = 0.0
                for k in range(Atoms):
                    atom_sum += q_dist[b, t, a, k]
                
                action_mean = atom_sum / Atoms
                
                if action_mean > best_action_val:
                    best_action_val = action_mean
            
            q_reduced[b, t] = best_action_val
            
    return q_reduced

def calculate_evi(
    score_history: list,
    score_mean: float,
    score_std: float,
    score_slope: float,
    curr_score: float,
    curr_entropy: float
) -> float:
    if isinstance(score_history, list):
        h_arr = np.array(score_history, dtype=np.float32)
    else:
        h_arr = score_history
        
    return _numba_calculate_evi(h_arr, score_mean, score_std, score_slope, curr_score, curr_entropy)


def nstep_targets(
    rew: torch.Tensor,
    term: torch.Tensor,
    trunc: torch.Tensor,
    q_next: torch.Tensor,
    gamma: float,
    n: int
) -> torch.Tensor:
    """Optimized nstep targets (handles C51 and Q-Learning)"""
    device = rew.device
    
    rew_np = rew.detach().cpu().numpy().astype(np.float32)
    term_np = term.detach().cpu().numpy().astype(np.float32)
    q_next_np = q_next.detach().cpu().numpy().astype(np.float32)
    
    if q_next_np.ndim == 4:
        q_values_np = _numba_reduce_distribution(q_next_np)
    elif q_next_np.ndim == 3:
        q_values_np = np.max(q_next_np, axis=2)
    else:
        q_values_np = q_next_np

    targets_np = _numba_nstep_backup(rew_np, term_np, q_values_np, float(gamma))
    return torch.from_numpy(targets_np).to(device)


def r2d2_loss(
    q_seq: torch.Tensor,
    a_seq: torch.Tensor,
    target_seq: torch.Tensor,
    weights: torch.Tensor,
    burn_in: int = 0
) -> Tuple[torch.Tensor, torch.Tensor]:
    """Calculate R2D2 loss (with autograd support)"""
    if q_seq.ndim == 4: 
        q_seq = q_seq.mean(dim=-1)

    q_chosen = q_seq.gather(-1, a_seq.unsqueeze(-1)).squeeze(-1)
    
    B, T = q_chosen.shape
    
    if burn_in > 0 and burn_in < T:
        q_slice = q_chosen[:, burn_in:]
        t_slice = target_seq[:, burn_in:]
        w_slice = weights.unsqueeze(-1).expand(-1, T - burn_in)
    else:
        q_slice = q_chosen
        t_slice = target_seq
        w_slice = weights.unsqueeze(-1).expand(-1, T)
        
    td_error = t_slice - q_slice
    loss_elementwise = F.smooth_l1_loss(q_slice, t_slice, reduction='none')
    
    loss_weighted = (loss_elementwise * w_slice).mean()
    
    with torch.no_grad():
        new_priorities = td_error.abs().mean(dim=1)
        new_priorities = new_priorities + 1e-5

    return loss_weighted, new_priorities