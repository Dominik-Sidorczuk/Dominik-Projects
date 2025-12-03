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

from src.config import AutoRLConfig
from src.core.models.drqn import DRQN
from src.core.models.drqn_c51 import DRQN_C51 
from src.core.agent import Agent 
from src.core.metrics import nstep_targets, r2d2_loss
from src.envs.make_env import make_vector_env
from src.eval.evaluate import evaluate_loop 
# [REFAC] Importujemy wydzielony runner
from src.core.vector_runner import VectorRunner
import copy
from src.core.vector_runner import VectorRunner
import copy
from concurrent.futures import ThreadPoolExecutor, as_completed
from concurrent.futures import ThreadPoolExecutor, as_completed
from src.orchestrators.pbt import PBTOrchestrator
from src.orchestrators.immune import ImmuneOrchestrator
from src.orchestrators.knowledge_transfer import NeuralSurgeon

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

# --- CLASSES ---

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
        
        # 1. Check if we meet the requirements
        if len(self.memory) > self.cfg.batch_size and self.global_step > self.cfg.learning_starts:
            for i in range(self.cfg.gradient_steps):
                info.update(self.update())
            
            self._update_target_network()
                
        hist = self.stats['return_history']
        if len(hist) > 200:
            self.stats['return_history'] = hist[-100:]
            
        curr_ret = np.mean(self.stats['return_history']) if self.stats['return_history'] else 0.0
        
        info['mean_return'] = curr_ret
        info['evi'] = curr_ret 
        return info

    def update(self) -> Dict[str, float]:
        self.model.train() # [AMD FIX] Force train mode for backward pass
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
                    # Handle empty tensor case (safe stat calculation)
                    if q_vals.numel() > 0:
                        q_spread = q_vals.std().mean().item()
                        probs = F.softmax(q_vals, dim=-1)
                        log_probs = torch.log(probs + 1e-6)
                        entropy = -(probs * log_probs).sum(dim=-1).mean().item()
                    else:
                        q_spread = 0.0
                        entropy = 0.0

            self.optimizer.zero_grad(set_to_none=True)
            self.scaler.scale(weighted_loss).backward()
            self.scaler.unscale_(self.optimizer)
            
            # Clip grad norm jest kluczowy dla LSTM
            norm = torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.cfg.max_grad_norm)
            
            # --- GRADIENT GUARD (FIXED) ---
            if self.raw_cfg.stability.gradient_guard and (torch.isnan(norm) or torch.isinf(norm)):
                log.warning(f"⚠️ NAN/INF in gradients (norm: {norm.item()}). letting scaler handle it.")
            
            # Skaler automatycznie pominie krok, jeśli wykryje Inf/NaN po unscale
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
        # [FIX]: Use self.model instead of self.online_net
        metrics = evaluate_loop(
            self.raw_cfg.env.id, self.model, self.raw_cfg.eval, 
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

class PopulationManager:
    def __init__(self, cfg: AutoRLConfig):
        self.cfg = cfg
        self.pbt = PBTOrchestrator(cfg.pbt)
        self.population = []
        
        log.info(f"👥 Initializing Population of {cfg.pbt.population_size} agents...")
        
        for i in range(cfg.pbt.population_size):
            agent_cfg = cfg.model_copy(deep=True)
            agent_cfg.seed = cfg.seed + i
            # Create unique output dir for each agent
            agent_cfg.out_dir = os.path.join(cfg.out_dir, f"agent_{i}")
            os.makedirs(agent_cfg.out_dir, exist_ok=True)
            
            trainer = Trainer(agent_cfg, cfg.device)
            self.population.append({
                'id': i,
                'trainer': trainer,
                'cfg': agent_cfg,
                'score': -float('inf'),
                'score': -float('inf'),
                'last_log_step': 0,
                'last_pbt_step': 0
            })
            
        self.immune = ImmuneOrchestrator(cfg.immune)
        self.surgeon = NeuralSurgeon(cfg.orchestrators['knowledge_transfer'])
            
    def run(self):
        log.info("🚀 Starting PBT Loop...")
        total_steps = self.cfg.train.total_steps
        pbt_interval = self.cfg.pbt.interval
        log_freq = self.cfg.train.log_freq
        
        active = True
        with ThreadPoolExecutor(max_workers=self.cfg.pbt.population_size) as executor:
            # 2. Continuous Loop
            from concurrent.futures import wait, FIRST_COMPLETED
            
            # Helper to distinguish task types
            # futures[future] = (agent, task_type)
            # task_type: 'train' or 'pbt'
            
            # Re-init futures with type info
            futures = {}
            for agent in self.population:
                futures[executor.submit(agent['trainer'].step)] = (agent, 'train')

            while futures:
                # Wait for at least one future to complete
                done, _ = wait(futures.keys(), return_when=FIRST_COMPLETED)
                
                for future in done:
                    agent, task_type = futures.pop(future)
                    trainer = agent['trainer']
                    
                    try:
                        if task_type == 'train':
                            info = future.result()
                            
                            # Logging
                            if trainer.global_step - agent['last_log_step'] >= log_freq:
                                 log.info(f"Agent {agent['id']} Step {trainer.global_step}: Loss={info.get('loss', 0):.4f} | Return={info.get('mean_return', 0):.2f}")
                                 agent['last_log_step'] = trainer.global_step

                            # Checkpointing
                            if trainer.global_step % self.cfg.train.checkpoint_freq == 0:
                                path = os.path.join(agent['cfg'].out_dir, f"ckpt_{trainer.global_step}.pt")
                                trainer.save(path)

                            # Immune System Check
                            diagnosis = self.immune.check(agent['id'], trainer.global_step, info)
                            if diagnosis:
                                 self.immune.prescribe(agent['id'], diagnosis, trainer)
                            
                            # DECIDE NEXT STEP: PBT or TRAIN
                            if trainer.global_step - agent['last_pbt_step'] >= pbt_interval:
                                # Submit PBT Task
                                futures[executor.submit(self._pbt_task, agent)] = (agent, 'pbt')
                            elif trainer.global_step < total_steps:
                                # Submit Train Task
                                futures[executor.submit(trainer.step)] = (agent, 'train')
                                
                        elif task_type == 'pbt':
                            # PBT finished, just resubmit training
                            future.result() # Check for exceptions
                            
                            # [AMD FIX] Cleanup after PBT/Eval
                            trainer.model.train()
                            trainer.target_model.eval()
                            agent['last_pbt_step'] = trainer.global_step
                            
                            if trainer.global_step < total_steps:
                                futures[executor.submit(trainer.step)] = (agent, 'train')
                            
                    except Exception as e:
                        log.error(f"❌ Agent {agent['id']} failed in {task_type}: {e}")
                        raise e
                    
    def _pbt_task(self, agent):
        """Wrapper for PBT step to run in thread"""
        self._pbt_step(agent)
        return True

    def _pbt_step(self, agent):
        # Evaluate
        trainer = agent['trainer']
        metrics = trainer.eval_and_log()
        score = metrics.get('eval_return', -float('inf'))
        agent['score'] = score
        
        log.info(f"🧬 Agent {agent['id']} PBT Check. Score: {score:.2f}")
        
        # Exploit
        mentor = self.pbt.get_mentor(self.population)
        if mentor['id'] != agent['id'] and mentor['score'] > score:
             # Exploit (Neural Transplant)
             log.info(f"♻️  Agent {agent['id']} exploits Mentor {mentor['id']} (Score: {mentor['score']:.2f} > {score:.2f})")
             
             # Use NeuralSurgeon for safe weight transfer (handles shape mismatches if any)
             self.surgeon.perform_transplant(mentor['trainer'].model.state_dict(), trainer.model)
             self.surgeon.perform_transplant(mentor['trainer'].target_model.state_dict(), trainer.target_model)
             
             trainer.optimizer.state.clear()
             
             # Explore (Perturb config)
             new_cfg = self.pbt.perturb_single_agent(
                 current_cfg=agent['cfg'],
                 current_score=score,
                 agent_id=agent['id']
             )
             
             if new_cfg:
                 agent['cfg'] = new_cfg
                 trainer.reload_config(new_cfg)
                 self.pbt.reset_local_momentum(agent['id'], mentor['score'])
             
    def stop(self):
        log.info("🛑 Stopping Population Manager...")
        for agent in self.population:
             path = os.path.join(agent['cfg'].out_dir, "final_ckpt.pt")
             agent['trainer'].save(path)
             log.info(f"💾 Saved final checkpoint for Agent {agent['id']}")

def run_single(cfg: AutoRLConfig):
    log.info(f"🚀 Starting Single Run: {cfg.env.id} | Seed: {cfg.seed}")
    trainer = Trainer(cfg, cfg.device)
    
    # Load checkpoint if specified
    if cfg.eval.ckpt_path:
        trainer.load(cfg.eval.ckpt_path)
        
    start_time = time.time()
    last_log_step = 0
    
    try:
        while trainer.global_step < cfg.train.total_steps:
            info = trainer.step()
            
            if trainer.global_step - last_log_step >= cfg.train.log_freq:
                elapsed = time.time() - start_time
                sps = trainer.global_step / (elapsed + 1e-6)
                log.info(f"Step {trainer.global_step}: Loss={info.get('loss', 0):.4f} | Return={info.get('mean_return', 0):.2f} | SPS={sps:.1f}")
                last_log_step = trainer.global_step
                
            if trainer.global_step % cfg.train.eval_freq == 0:
                metrics = trainer.eval_and_log()
                log.info(f"🔍 Eval at {trainer.global_step}: {metrics}")
                
            if trainer.global_step % cfg.train.checkpoint_freq == 0:
                path = os.path.join(cfg.out_dir, f"ckpt_{trainer.global_step}.pt")
                trainer.save(path)
                
    except KeyboardInterrupt:
        log.info("🛑 Training interrupted.")
    finally:
        path = os.path.join(cfg.out_dir, "final_ckpt.pt")
        trainer.save(path)
        log.info("✅ Training finished.")