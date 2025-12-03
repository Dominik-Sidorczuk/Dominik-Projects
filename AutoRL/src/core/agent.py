from __future__ import annotations

import torch
import numpy as np
import math
import logging
from typing import Union, Tuple, Optional

from src.config import AgentConfig

log = logging.getLogger(__name__)

class Agent:
    """Hyper-optimized inference agent with cycle and immune support"""
    def __init__(
        self,
        net: torch.nn.Module,
        action_dim: int,
        cfg: AgentConfig,
        device: str = "cuda",
    ):
        self.cfg = cfg
        self.net = net
        self.action_dim = int(action_dim)
        self.device = torch.device(device)
        self.net.to(self.device)
        
        self.step_count = 0
        self._external_step = 0
        self._immune_eps_boost = 0.0
        self._last_epsilon = cfg.eps_start
        
        self._decay_factor = -1.0 / self.cfg.eps_decay
        self._eps_diff = self.cfg.eps_start - self.cfg.eps_min
        
        self.is_noisy = cfg.use_noisy
        
        log.info(f"[AGENT] Init on {device}. Strategy: {'NoisyNet' if self.is_noisy else 'EpsGreedy'}")

    def get_epsilon(self) -> float:
        """Calculate epsilon with cycles, decay and immune boost"""
        if self._immune_eps_boost > 0:
            return self._immune_eps_boost
        
        if self.is_noisy:
            return 0.0

        step = self._external_step if self.cfg.eps_anneal_source == "external" else self.step_count
        
        if self.cfg.use_cycles:
            cycle_pos = (step % self.cfg.cycle_length) / self.cfg.cycle_length
            cos_out = 0.5 * (1 + math.cos(math.pi * cycle_pos)) 
            eps = self.cfg.eps_min + self._eps_diff * cos_out
            
        else:
            decay = math.exp(step * self._decay_factor)
            eps = self.cfg.eps_min + self._eps_diff * decay

        self._last_epsilon = eps
        return max(self.cfg.eps_min, eps)

    def update_step(self, global_step: int):
        """Sync with trainer global step"""
        self._external_step = global_step

    def set_immune_boost(self, val: float):
        """Immune system API for exploration boost"""
        self._immune_eps_boost = val

    @torch.inference_mode()
    def act(self, 
            obs: Union[np.ndarray, torch.Tensor], 
            hidden: Optional[Tuple] = None) -> Tuple[torch.Tensor, Optional[Tuple]]:
        """High-performance inference with minimal CPU<->GPU switching"""
        if isinstance(obs, np.ndarray):
            obs_tensor = torch.as_tensor(obs, device=self.device)
        else:
            obs_tensor = obs.to(self.device)
        
        if obs_tensor.dtype == torch.uint8:
            obs_tensor = obs_tensor.float().div_(255.0)
            
        if obs_tensor.dim() == 2:
            obs_tensor = obs_tensor.unsqueeze(1) 
        elif obs_tensor.dim() == 1:
            obs_tensor = obs_tensor.unsqueeze(0).unsqueeze(0)

        if hidden is not None:
            hidden = tuple(t.to(self.device) for t in hidden)


        if hasattr(self.net, 'get_q_values'):
            q_values, next_hidden = self.net.get_q_values(obs_tensor, hidden)
        else:
            q_values, next_hidden = self.net(obs_tensor, hidden)
        q_t = q_values[:, -1, :]
        greedy_actions = q_t.argmax(dim=-1)


        # [IMMUNE FIX]: Allow immune system to force epsilon noise even if NoisyNets are on
        immune_eps = self._immune_eps_boost if self._immune_eps_boost > 0 else 0.0
        
        if self.is_noisy and immune_eps == 0.0:
            final_actions = greedy_actions
        else:
            eps = self.get_epsilon()
            if eps > 1e-4:
                rand_probs = torch.rand(greedy_actions.shape, device=self.device)
                random_actions = torch.randint(0, self.action_dim, greedy_actions.shape, device=self.device)
                final_actions = torch.where(rand_probs < eps, random_actions, greedy_actions)
            else:
                final_actions = greedy_actions


        if self.cfg.eps_anneal_source == "agent_calls":
            self.step_count += final_actions.shape[0]
            
        return final_actions, next_hidden