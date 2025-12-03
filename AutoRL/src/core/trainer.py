from __future__ import annotations

import logging
import os
import time
import sys
import numpy as np
import torch
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F
from torch.amp import GradScaler, autocast
from typing import Dict, Any, Optional, Union, Tuple
from numba import njit, prange

from src.config import AutoRLConfig
from src.core.models.drqn import DRQN
from src.core.models.drqn_c51 import DRQN_C51 
from src.core.agent import Agent 
from src.core.metrics import nstep_targets, r2d2_loss
from src.envs.make_env import make_vector_env
from src.eval.evaluate import evaluate_loop 

try:
    from src.core.replay.per_sequence_HP import PrioritizedSequenceBufferHP
except ImportError:
    PrioritizedSequenceBufferHP = None

try:
    from src.core.replay.per_sequence import PrioritizedSequenceBuffer
except ImportError:
    PrioritizedSequenceBuffer = None

if PrioritizedSequenceBufferHP is None and PrioritizedSequenceBuffer is None:
    raise ImportError("CRITICAL: Brak plików bufora pamięci (per_sequence.py lub per_sequence_HP.py)!")

log = logging.getLogger(__name__)

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
    
    for i in range(n):
        val = actions_raw[i]
        
        if not (val >= 0.0): # Catch NaN
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

# --- CLASSES ---

class VectorRunner:
    """
    High-Performance Runner with Numba Acceleration.
    Uses pre-allocated NumPy buffers to avoid Python GC overhead in hot loops.
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

        # SANITY CHECK (Tylko logujemy błąd, nie wywalamy programu printem)
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

class Trainer:
    def __init__(self, cfg: AutoRLConfig, device: str):
        self.raw_cfg = cfg
        self.cfg = cfg.train
        self.DEVICE = torch.device(device)
        self.global_step = 0
        
        log.info(f"🚀 Trainer Init: {self.DEVICE} | AMP: {self.DEVICE.type == 'cuda'}")
        
        self.env = make_vector_env(
            env_id=cfg.env.id,
            num_envs=self.cfg.num_envs,
            seed=cfg.seed,
            action_repeat=cfg.env.action_repeat,
            frame_stack=cfg.env.frame_stack,
            vector_mode=cfg.env.vector_mode,
            clip_reward=cfg.env.clip_reward
        )
        
        obs_dim = self.env.single_observation_space.shape[0]
        action_dim = self.env.single_action_space.n
        
        ModelClass = DRQN_C51 if (hasattr(cfg.model, 'atoms') and cfg.model.atoms > 1) else DRQN
        self.model = ModelClass(obs_dim, action_dim, cfg.model).to(self.DEVICE)
        self.target_model = ModelClass(obs_dim, action_dim, cfg.model).to(self.DEVICE)
        self.target_model.load_state_dict(self.model.state_dict())
        self.target_model.load_state_dict(self.model.state_dict())
        self.target_model.eval()
        
        if self.DEVICE.type == 'cuda':
            torch.cuda.synchronize()
        
        self.agent = Agent(self.model, action_dim, cfg.agent, str(self.DEVICE))
        
        self.optimizer = optim.AdamW(
            self.model.parameters(), 
            lr=float(cfg.optimizer.lr), 
            weight_decay=float(cfg.optimizer.weight_decay),
            eps=cfg.optimizer.eps
        )
        self.scaler = GradScaler('cuda', enabled=(self.cfg.amp and self.DEVICE.type == 'cuda'))
        
        capacity_per_env = max(self.cfg.seq_len, self.cfg.buffer_size // self.cfg.num_envs)
        
        if self.raw_cfg.per.use_hp and PrioritizedSequenceBufferHP is not None:
            BufferClass = PrioritizedSequenceBufferHP
            b_type = "HP (Numba)"
        elif PrioritizedSequenceBuffer is not None:
            BufferClass = PrioritizedSequenceBuffer
            b_type = "Std (NumPy)"
        else:
            BufferClass = PrioritizedSequenceBufferHP
            b_type = "HP (Fallback)"

        log.info(f"💾 Buffer: {capacity_per_env} steps/env | Type: {b_type}")

        self.memory = BufferClass(
            cfg=self.raw_cfg.per,
            capacity=capacity_per_env,
            seq_len=self.cfg.seq_len,
            obs_dim=obs_dim,
            num_envs=self.cfg.num_envs,
            device=str(self.DEVICE)
        )
        
        log.debug("Initializing VectorRunner...")
        self.runner = VectorRunner(self.env, self, cfg)
        log.debug("VectorRunner initialized. System READY.")
        
        self.stats = {'loss': [], 'return_history': [], 'q_values': []}
        self.last_metrics = {}
        self._first_run = True

    def step(self) -> Dict[str, float]:
        # --- DIAGNOSTYKA STARTU ---
        if self._first_run:
            log.info("📢 Starting FIRST rollout (compiling Numba, warming up Env)...")
            start_time = time.time()
            
        collected = self.runner.run()
        
        if self._first_run:
            duration = time.time() - start_time
            log.info(f"✅ First rollout complete. Duration: {duration:.2f}s. Collected: {collected} steps.")
            self._first_run = False
        # --------------------------

        self.global_step += collected
        
        info = {}
        
        # DEBUG: Check condition values
        log.info(f"🔍 Check: Step={self.global_step}, Start={self.cfg.learning_starts}, Mem={len(self.memory)}, Batch={self.cfg.batch_size}")
        
        if len(self.memory) > self.cfg.batch_size and self.global_step > self.cfg.learning_starts:
            # DEBUG: Log condition met
            log.info(f"✅ Update Triggered: Step {self.global_step} > {self.cfg.learning_starts}, Mem {len(self.memory)} > {self.cfg.batch_size}")
            for i in range(self.cfg.gradient_steps):
                upd_info = self.update()
                if upd_info.get('loss', 0.0) == 0.0 and upd_info.get('skipped', 0.0) == 0.0:
                     log.debug(f"⚠️ Zero loss detected without skip flag. Info: {upd_info}")
                info.update(upd_info)
                self._update_target_network()
        elif len(self.memory) > self.cfg.batch_size:
             log.debug(f"⚠️ Skipping update: Global Step {self.global_step} <= Learning Starts {self.cfg.learning_starts}")
                
        hist = self.stats['return_history']
        if len(hist) > 200:
            self.stats['return_history'] = hist[-100:]
            
        curr_ret = np.mean(self.stats['return_history']) if self.stats['return_history'] else 0.0
        
        info['mean_return'] = curr_ret
        info['evi'] = curr_ret 
        return info

    def update(self) -> Dict[str, float]:
        self.model.train() # Ensure training mode (might be changed by eval)
        batch = self.memory.sample(self.cfg.batch_size)
        
        obs_curr = batch['obs'][:, :-1]
        next_obs = batch['obs'][:, 1:]
        actions  = batch['act'][:, :-1]
        rewards  = batch['rew'][:, :-1]
        dones    = torch.max(batch['term'], batch['trunc'])[:, :-1]
        weights  = batch['weights']

        B = obs_curr.size(0)
        hidden = self.model.init_hidden(B)
        target_hidden = self.target_model.init_hidden(B)
        
        # [LIBERATION]: Jawna obsługa błędów numerycznych w pętli uczenia
        try:
            with autocast(self.DEVICE.type, enabled=(self.cfg.amp and self.DEVICE.type == 'cuda')):
                if hasattr(self.model, 'get_q_values'):
                    q_vals, _ = self.model.get_q_values(obs_curr, hidden)
                else:
                    q_vals, _ = self.model(obs_curr, hidden)
                
                with torch.no_grad():
                    if hasattr(self.target_model, 'get_q_values'):
                        target_q, _ = self.target_model.get_q_values(next_obs, target_hidden)
                    else:
                        target_q, _ = self.target_model(next_obs, target_hidden)
                    
                    targets = nstep_targets(
                        rew=rewards,
                        term=dones,
                        trunc=torch.zeros_like(dones), 
                        q_next=target_q,
                        gamma=self.cfg.gamma,
                        n=self.cfg.n_step
                    )

                loss, new_prio = r2d2_loss(
                    q_seq=q_vals,
                    a_seq=actions,
                    target_seq=targets,
                    weights=weights,
                    burn_in=self.cfg.rnn_burn_in
                )
                
                weighted_loss = (loss * weights.view(-1, 1)).mean()

                # --- NAN GUARD START ---
                if self.raw_cfg.stability.nan_guard and (torch.isnan(weighted_loss) or torch.isinf(weighted_loss)):
                    log.warning(f"⚠️ NAN/INF detected in loss! Value: {weighted_loss.item()}. Skipping step.")
                    self.scaler.update() # Aktualizujemy skaler, by pominąć ten krok bezpiecznie
                    return {'loss': 0.0, 'grad_norm': 0.0, 'skipped': 1.0}
                # --- NAN GUARD END ---
                
                with torch.no_grad():
                    q_spread = q_vals.std().mean().item()
                    probs = F.softmax(q_vals, dim=-1)
                    log_probs = torch.log(probs + 1e-6)
                    entropy = -(probs * log_probs).sum(dim=-1).mean().item()

            self.optimizer.zero_grad(set_to_none=True)
            self.scaler.scale(weighted_loss).backward()
            self.scaler.unscale_(self.optimizer)
            
            # Clip grad norm jest kluczowy dla LSTM
            norm = torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.cfg.max_grad_norm)
            
            # --- GRADIENT GUARD (FIXED) ---
            # [FIX]: Nie blokujemy manualnie kroku scaler.step().
            # Jeśli norm == NaN, scaler.step() sam wykryje problem, pominie aktualizację wag
            # i ZMNIEJSZY skalę (scaler.update()), co jest kluczowe dla stabilności.
            if self.raw_cfg.stability.gradient_guard and (torch.isnan(norm) or torch.isinf(norm)):
                log.warning(f"⚠️ NAN/INF in gradients (norm: {norm.item()}). Letting scaler handle skip & scale reduction.")
            
            self.scaler.step(self.optimizer)
            self.scaler.update()
            
            prio_np = new_prio.detach().cpu().numpy() if isinstance(new_prio, torch.Tensor) else new_prio
            self.memory.update_priorities(batch['indices'], prio_np)
            
            return {
                'loss': weighted_loss.item(),
                'grad_norm': norm.item(),
                'q_spread': q_spread,
                'entropy': entropy
            }

        except RuntimeError as e:
            if self.raw_cfg.stability.oom_guard and "out of memory" in str(e):
                log.error("💥 OOM in update step! Clearing cache.")
                torch.cuda.empty_cache()
                return {'loss': 0.0}
            raise e

    def _update_target_network(self):
        with torch.no_grad():
            for p, tp in zip(self.model.parameters(), self.target_model.parameters()):
                tp.data.mul_(1.0 - self.cfg.tau).add_(p.data * self.cfg.tau)

    def eval_and_log(self, episodes: int = 5) -> Dict[str, float]:
        metrics = evaluate_loop(
            self.raw_cfg.env.id, self.online_net, self.raw_cfg.eval, 
            self.raw_cfg.seed, self.raw_cfg.env.action_repeat, str(self.DEVICE)
        )
        self.last_metrics.update(metrics)
        return metrics

    def reload_config(self, new_cfg: AutoRLConfig): 
        self.raw_cfg = new_cfg
        self.cfg = new_cfg.train
        
        for g in self.optimizer.param_groups: 
            g['lr'] = float(new_cfg.optimizer.lr)
            if 'weight_decay' in new_cfg.optimizer:
                 g['weight_decay'] = float(new_cfg.optimizer.weight_decay)

        if hasattr(self.agent, 'cfg'):
            self.agent.cfg = new_cfg.agent

    def save(self, path: str):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save({
            'model': self.model.state_dict(),
            'optim': self.optimizer.state_dict(),
            'step': self.global_step,
            'cfg': self.raw_cfg.model_dump()
        }, path)

    def load(self, path: str):
        if not os.path.exists(path): return
        try:
            ckpt = torch.load(path, map_location=self.DEVICE)
            sd = ckpt['model'] if isinstance(ckpt, dict) and 'model' in ckpt else ckpt
            if 'model_state_dict' in ckpt: sd = ckpt['model_state_dict']
            sd = {k.replace("_orig_mod.", ""): v for k, v in sd.items()}
            
            self.model.load_state_dict(sd, strict=False)
            self.target_model.load_state_dict(self.model.state_dict())
            
            if isinstance(ckpt, dict):
                if 'optim' in ckpt: self.optimizer.load_state_dict(ckpt['optim'])
                if 'step' in ckpt: self.global_step = ckpt['step']
                
            log.info("✅ Checkpoint loaded.")
        except Exception as e:
            log.error(f"❌ Load failed: {e}")
            raise e