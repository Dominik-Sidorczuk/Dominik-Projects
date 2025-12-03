from __future__ import annotations

import torch
import numpy as np
from typing import Dict, Tuple
from numba import njit, prange
from src.config import PERConfig

@njit(fastmath=True, cache=True)
def _numba_update_tree(tree: np.ndarray, indices: np.ndarray, priorities: np.ndarray):
    """Fast sum-tree update in Numba"""
    for i in range(len(indices)):
        idx = indices[i]
        prio = priorities[i]
        
        diff = prio - tree[idx]
        tree[idx] = prio
        
        parent = (idx - 1) // 2
        while parent >= 0:
            tree[parent] += diff
            if parent == 0:
                break
            parent = (parent - 1) // 2

@njit(fastmath=True, cache=True)
def _numba_get_indices(tree: np.ndarray, values: np.ndarray, tree_capacity: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fast search in sum-tree (O(log N) per value)"""
    n = len(values)
    tree_indices = np.zeros(n, dtype=np.int64)
    priorities = np.zeros(n, dtype=np.float64)
    
    for i in range(n):
        val = values[i]
        idx = 0
        
        while True:
            left = 2 * idx + 1
            right = left + 1
            
            if left >= len(tree):
                break
                
            left_val = tree[left]
            
            if val <= left_val:
                idx = left
            else:
                val -= left_val
                idx = right
                
        tree_indices[i] = idx
        priorities[i] = tree[idx]
        
    data_indices = tree_indices - tree_capacity + 1
    return tree_indices, priorities, data_indices

class SumTreeHP:
    """NumPy array wrapper with Numba kernel support"""
    def __init__(self, size: int):
        self.size = size
        
        self.tree_capacity = 1
        while self.tree_capacity < size:
            self.tree_capacity *= 2
            
        self.tree = np.zeros(2 * self.tree_capacity - 1, dtype=np.float64)

    def batch_update(self, tree_indices: np.ndarray, priorities: np.ndarray):
        priorities = np.nan_to_num(priorities, nan=1e-5, posinf=1.0, neginf=1e-5)
        _numba_update_tree(self.tree, tree_indices, priorities)

    def batch_get(self, values: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return _numba_get_indices(self.tree, values, self.tree_capacity)

    @property
    def total_priority(self):
        return max(float(self.tree[0]), 1e-5)


class PrioritizedSequenceBufferHP:
    """High-performance sequence buffer with Numba acceleration"""
    def __init__(self, cfg: PERConfig, capacity: int, seq_len: int, obs_dim: int, num_envs: int, device: str = 'cpu'):
        self.cfg = cfg
        self.capacity = int(capacity)
        self.seq_len = int(seq_len)
        self.obs_dim = int(obs_dim)
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        
        self.total_slots = self.capacity * self.num_envs
        
        self.obs = torch.zeros((self.capacity, self.num_envs, self.obs_dim), dtype=torch.float32)
        self.actions = torch.zeros((self.capacity, self.num_envs), dtype=torch.long)
        self.rewards = torch.zeros((self.capacity, self.num_envs), dtype=torch.float32)
        self.terms = torch.zeros((self.capacity, self.num_envs), dtype=torch.float32)
        self.truncs = torch.zeros((self.capacity, self.num_envs), dtype=torch.float32)
        
        self.valid_starts = np.zeros(self.total_slots, dtype=bool)
        
        self.tree = SumTreeHP(self.total_slots)
        
        self.pos = 0
        self.is_full = False
        self.max_priority = 1.0 
        self.alpha = cfg.alpha
        self.beta = cfg.beta_start 
        
        self.seq_offsets = torch.arange(self.seq_len, dtype=torch.long)
        
        # DEBUG LOGGING (Temporary)
        import logging
        _log = logging.getLogger(__name__)
        _log.debug(f"BufferHP initialized. Capacity: {self.capacity}, Envs: {self.num_envs}, Device: {self.device}")
        _log.debug(f"SumTree size: {self.total_slots}")

    def add_batch(self, batch: Dict[str, torch.Tensor]):
        b_obs = batch['obs'].squeeze(0).cpu()
        b_act = batch['act'].squeeze(0).cpu()
        b_rew = batch['rew'].squeeze(0).cpu()
        b_term = batch['term'].squeeze(0).cpu()
        b_trunc = batch['trunc'].squeeze(0).cpu()
        
        self.obs[self.pos] = b_obs
        self.actions[self.pos] = b_act
        self.rewards[self.pos] = b_rew
        self.terms[self.pos] = b_term
        self.truncs[self.pos] = b_trunc
        
        start_flat_idx = self.pos * self.num_envs
        end_flat_idx = start_flat_idx + self.num_envs
        flat_indices = np.arange(start_flat_idx, end_flat_idx)
        
        self.valid_starts[flat_indices] = True
        
        back_times = (np.arange(self.pos - self.seq_len + 1, self.pos) % self.capacity)
        for bt in back_times:
             self.valid_starts[bt * self.num_envs : (bt + 1) * self.num_envs] = False
        
        tree_indices = flat_indices + self.tree.tree_capacity - 1
        new_priorities = np.full(self.num_envs, self.max_priority, dtype=np.float64)
        self.tree.batch_update(tree_indices, new_priorities)

        self.pos += 1
        if self.pos >= self.capacity:
            self.pos = 0
            self.is_full = True

    def sample(self, batch_size: int) -> Dict[str, torch.Tensor]:
        batch_size = int(batch_size)
        total_p = self.tree.total_priority
        segment = total_p / batch_size
        
        rand_vals = np.random.uniform(0, segment, batch_size)
        targets = rand_vals + np.arange(batch_size) * segment
        
        targets = np.clip(targets, 0.0, total_p - 1e-6)
        
        _, priorities, flat_indices = self.tree.batch_get(targets)
        
        is_valid = self.valid_starts[flat_indices]
        if not np.all(is_valid):
            valid_candidates = np.where(self.valid_starts)[0]
            if len(valid_candidates) > 0:
                replace_count = np.sum(~is_valid)
                replacements = np.random.choice(valid_candidates, size=replace_count)
                flat_indices[~is_valid] = replacements
                priorities[~is_valid] = self.tree.tree[replacements + self.tree.tree_capacity - 1]

        time_indices = flat_indices // self.num_envs
        env_indices = flat_indices % self.num_envs
        
        t_start = torch.from_numpy(time_indices).unsqueeze(1)
        t_seq = t_start + self.seq_offsets.unsqueeze(0)
        t_seq = t_seq % self.capacity
        
        e_seq = torch.from_numpy(env_indices).unsqueeze(1).expand(-1, self.seq_len)
        
        batch = {
            "obs": self.obs[t_seq, e_seq].to(self.device, non_blocking=False),
            "act": self.actions[t_seq, e_seq].to(self.device, non_blocking=False),
            "rew": self.rewards[t_seq, e_seq].to(self.device, non_blocking=False),
            "term": self.terms[t_seq, e_seq].to(self.device, non_blocking=False),
            "trunc": self.truncs[t_seq, e_seq].to(self.device, non_blocking=False),
            "indices": flat_indices
        }
        
        weights = (self.total_slots * priorities) ** (-self.beta)
        weights = weights / (weights.max() + 1e-5)
        batch["weights"] = torch.tensor(weights, device=self.device, dtype=torch.float32)
        
        return batch

    def update_priorities(self, indices: np.ndarray, priorities: np.ndarray):
        if isinstance(priorities, torch.Tensor):
            priorities = priorities.detach().cpu().numpy()
        if isinstance(indices, torch.Tensor):
            indices = indices.cpu().numpy()
            
        priorities = np.abs(priorities) + 1e-6
        self.max_priority = max(self.max_priority, np.max(priorities))
        
        tree_indices = indices + self.tree.tree_capacity - 1
        self.tree.batch_update(tree_indices, priorities ** self.alpha)

    def update_beta(self, current_step: int):
        fraction = min(float(current_step) / self.cfg.beta_steps, 1.0)
        self.beta = self.cfg.beta_start + fraction * (1.0 - self.cfg.beta_start)

    def __len__(self):
        return self.capacity * self.num_envs if self.is_full else self.pos * self.num_envs
    
    def clear(self):
        self.pos = 0
        self.is_full = False
        self.valid_starts.fill(False)
        self.tree = SumTreeHP(self.total_slots)
