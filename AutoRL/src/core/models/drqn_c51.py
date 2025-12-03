from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Union, Tuple
from src.config import DRQNConfig
from .common import init_orthogonal_weights, NoisyLinear

class DRQN_C51(nn.Module):
    """Distributional DRQN (Categorical DQN)"""
    def __init__(
        self, 
        obs_dim: int, 
        action_dim: int, 
        cfg: DRQNConfig, 
        **kwargs
    ):
        super().__init__()
        self.cfg = cfg
        self.hidden_size = cfg.hidden_size
        self.action_dim = action_dim
        self.atoms = int(cfg.atoms)
        self.v_min = float(cfg.v_min)
        self.v_max = float(cfg.v_max)
        self.use_noisy = cfg.use_noisy

        self.register_buffer('supports', torch.linspace(self.v_min, self.v_max, self.atoms))

        self.embed = nn.Sequential(
            nn.LayerNorm(obs_dim),
            nn.Linear(obs_dim, self.hidden_size),
            nn.SiLU()
        )
        
        if getattr(cfg, 'use_lstm', True):
            self.core = nn.LSTM(self.hidden_size, self.hidden_size, batch_first=True)
        else:
            self.core = nn.GRU(self.hidden_size, self.hidden_size, batch_first=True)
        
        if self.use_noisy:
            self.head = NoisyLinear(self.hidden_size, action_dim * self.atoms, cfg.noisy_std_init)
        else:
            self.head = nn.Linear(self.hidden_size, action_dim * self.atoms)

        self.apply(init_orthogonal_weights)

    def reset_noise(self):
        """Reset noise in NoisyLinear layers"""
        if not self.use_noisy: return
        for m in self.modules():
            if m is not self and hasattr(m, 'reset_noise'):
                m.reset_noise()

    def init_hidden(self, batch_size: int) -> Union[torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        """Initialize hidden state based on RNN type"""
        weight = next(self.parameters())
        device, dtype = weight.device, weight.dtype
        
        if isinstance(self.core, nn.LSTM):
            return (
                torch.zeros(1, batch_size, self.hidden_size, device=device, dtype=dtype),
                torch.zeros(1, batch_size, self.hidden_size, device=device, dtype=dtype)
            )
        else:
            return torch.zeros(1, batch_size, self.hidden_size, device=device, dtype=dtype)

    def forward(self, obs: torch.Tensor, hidden: Union[torch.Tensor, Tuple[torch.Tensor, torch.Tensor]] = None):
        if obs.dim() == 2:
            obs = obs.unsqueeze(1)
            
        x = self.embed(obs)
        
        self.core.flatten_parameters()
        x, hidden = self.core(x, hidden)
        logits_flat = self.head(x)
        
        # Reshape: [B, T, Action, Atoms]
        B, T, _ = logits_flat.shape
        logits = logits_flat.view(B, T, self.action_dim, self.atoms)
        
        return logits, hidden

    def get_q_values(self, obs: torch.Tensor, hidden: Union[torch.Tensor, Tuple[torch.Tensor, torch.Tensor]] = None):
        """Get expected Q-values from distribution"""
        logits, hidden = self.forward(obs, hidden)
        probs = F.softmax(logits, dim=-1)
        
        q_values = (probs * self.supports).sum(dim=-1)
        
        return q_values, hidden