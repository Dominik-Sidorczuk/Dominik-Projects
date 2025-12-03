from __future__ import annotations

import logging
import numpy as np
import torch
from typing import TYPE_CHECKING

from src.config import AutoRLConfig
from src.core.kernels import _numba_process_actions, _numba_store_step

if TYPE_CHECKING:
    from runners import Trainer

log = logging.getLogger(__name__)

class VectorRunner:
    """
    High-Performance Runner with Numba Acceleration.
    Uses pre-allocated NumPy buffers to avoid Python GC overhead in hot loops.
    Moved to separate file for better modularity.
    """
    
    def __init__(self, envs, trainer: 'Trainer', cfg: AutoRLConfig):
        self.envs = envs
        self.trainer = trainer
        self.num_envs = cfg.train.num_envs
        self.rollout_steps = cfg.train.rollout_steps
        self.action_space_n = self.envs.single_action_space.n
        
        # Initial observation
        raw_obs = self.envs.reset()
        if isinstance(raw_obs, tuple):
            self.obs = raw_obs[0]
        else:
            self.obs = raw_obs
            
        # RNN State
        log.debug("VectorRunner: init_hidden starting...")
        self.hidden = self.trainer.model.init_hidden(self.num_envs)
        
        # --- Pre-allocate Buffers (CPU RAM) ---
        obs_shape = self.envs.single_observation_space.shape
        self.buf_obs = np.zeros((self.rollout_steps, self.num_envs) + obs_shape, dtype=np.float32)
        self.buf_act = np.zeros((self.rollout_steps, self.num_envs), dtype=np.int64)
        self.buf_rew = np.zeros((self.rollout_steps, self.num_envs), dtype=np.float32)
        self.buf_term = np.zeros((self.rollout_steps, self.num_envs), dtype=np.float32)
        self.buf_trunc = np.zeros((self.rollout_steps, self.num_envs), dtype=np.float32)
        
        log.debug("VectorRunner: Buffers pre-allocated.")
        
    def run(self) -> int:
        """Gorąca pętla zoptymalizowana Numbą"""
        total_steps = 0
        agent = self.trainer.agent
        memory = self.trainer.memory
        stats = self.trainer.stats
        debug_mode = self.trainer._first_run
        
        if debug_mode:
            log.info(f"⚡ VectorRunner: Starting loop. Steps: {self.rollout_steps}")

        for t in range(self.rollout_steps):
            if debug_mode and t % 50 == 0:
                log.info(f"   -> Step {t}/{self.rollout_steps} [Obs processing...]")

            agent.update_step(self.trainer.global_step + total_steps)
            
            # --- Inference (GPU) ---
            with torch.no_grad():
                actions_t, new_hidden = agent.act(self.obs, self.hidden)
            
            self.hidden = new_hidden
            
            # Post-process actions
            if actions_t.ndim > 2:
                actions_t = actions_t.squeeze(1)
            if actions_t.ndim > 1 and actions_t.shape[-1] == self.action_space_n:
                actions_t = torch.argmax(actions_t, dim=-1)
            
            # Transfer to CPU for Env
            actions_raw = actions_t.detach().cpu().numpy().ravel()
            actions = _numba_process_actions(actions_raw, self.action_space_n)
            
            # --- Environment Step (CPU) ---
            step_result = self.envs.step(actions)
            
            if len(step_result) == 5:
                next_obs, rewards, terminated, truncated, infos = step_result
            else:
                next_obs, rewards, dones_raw, infos = step_result
                terminated = dones_raw
                truncated = np.zeros_like(dones_raw, dtype=bool)

            # --- NUMBA STORE (Fastest Part) ---
            dones = _numba_store_step(
                self.buf_obs, self.buf_act, self.buf_rew, self.buf_term, self.buf_trunc,
                t, self.obs, actions, rewards, terminated, truncated
            )
            
            # --- RNN Reset & Stats ---
            if np.any(dones):
                self._reset_hidden(dones)
                self._process_infos(infos, dones, stats)
            
            self.obs = next_obs
            total_steps += self.num_envs
            
        # 2. FAZA TRANSFERU (Bulk Transfer)
        if debug_mode:
             log.info("⚡ VectorRunner: Loop done. Validating data...")

        # SANITY CHECK
        if np.isnan(self.buf_obs).any() or np.isnan(self.buf_rew).any():
            log.error("❌ CRITICAL: NaNs detected in collected buffers! Fixing with nan_to_num...")
            self.buf_obs = np.nan_to_num(self.buf_obs)
            self.buf_rew = np.nan_to_num(self.buf_rew)

        if debug_mode:
             log.info("⚡ VectorRunner: Transferring to GPU/Memory...")

        # Konwersja całego bloku na Tensory
        t_obs = torch.from_numpy(self.buf_obs)
        t_act = torch.from_numpy(self.buf_act)
        t_rew = torch.from_numpy(self.buf_rew)
        t_term = torch.from_numpy(self.buf_term)
        t_trunc = torch.from_numpy(self.buf_trunc)
        
        # Pętla dodawania do bufora
        for t in range(self.rollout_steps):
            memory.add_batch({
                'obs': t_obs[t].view(1, self.num_envs, -1),
                'act': t_act[t].view(1, self.num_envs),
                'rew': t_rew[t].view(1, self.num_envs),
                'term': t_term[t].view(1, self.num_envs),
                'trunc': t_trunc[t].view(1, self.num_envs)
            })
        
        #log.info(f"💾 VectorRunner: Added {self.rollout_steps} batches. Memory len: {len(memory)}")
            
        return total_steps

    def _reset_hidden(self, mask: np.ndarray):
        """Standard PyTorch reset - fastest on GPU"""
        if isinstance(self.hidden, tuple):
            h, c = self.hidden
            h = h.clone()
            c = c.clone()
            h[:, mask, :] = 0
            c[:, mask, :] = 0
            self.hidden = (h, c)
        else:
            h = self.hidden.clone()
            h[:, mask, :] = 0
            self.hidden = h
            
    def _process_infos(self, infos, dones, stats):
        """Helper to process episode stats"""
        if isinstance(infos, dict):
            if "episode" in infos:
                mask = infos.get("_episode", dones)
                valid_idx = np.where(mask & dones)[0]
                
                if len(valid_idx) > 0:
                    ep_data = infos["episode"]
                    if isinstance(ep_data, dict) and "r" in ep_data:
                        stats['return_history'].extend(ep_data["r"][valid_idx])
                    elif isinstance(ep_data, (list, np.ndarray)):
                        for idx in valid_idx:
                            if idx < len(ep_data) and isinstance(ep_data[idx], dict):
                                    stats['return_history'].append(ep_data[idx].get('r', 0.0))