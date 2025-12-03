from __future__ import annotations

import torch
import torch.nn as nn
from typing import Tuple, Union
from src.config import DRQNConfig
from .common import DuelingHead, init_orthogonal_weights, ResMLP, ModernSelfAttentionBlock

class R2D2(nn.Module):
    """R2D2 with ResNet + Attention + LSTM/GRU"""
    def __init__(
        self, 
        obs_dim: int, 
        action_dim: int, 
        cfg: DRQNConfig,
        **kwargs
    ):
        super().__init__()
        
        # Obsługa dict-config hacka (dla wstecznej kompatybilności)
        if isinstance(cfg, dict):
            cfg = DRQNConfig(**{k:v for k,v in cfg.items() if k in DRQNConfig.model_fields})
            
        self.cfg = cfg
        self.hidden_size = cfg.hidden_size
        self.use_noisy = cfg.use_noisy

        self.embed = ResMLP(obs_dim, self.hidden_size, depth=cfg.encoder_depth)

        if cfg.use_attention:
            self.attention = ModernSelfAttentionBlock(self.hidden_size, num_heads=cfg.attn_heads)
        else:
            self.attention = nn.Identity()

        if getattr(cfg, 'use_lstm', True):
            self.core = nn.LSTM(self.hidden_size, self.hidden_size, batch_first=True)
        else:
            self.core = nn.GRU(self.hidden_size, self.hidden_size, batch_first=True)
        
        if cfg.dueling:
            self.head = DuelingHead(self.hidden_size, action_dim, self.use_noisy, cfg.noisy_std_init)
        else:
            from .common import NoisyLinear
            layer = NoisyLinear if self.use_noisy else nn.Linear
            self.head = nn.Sequential(
                nn.LayerNorm(self.hidden_size),
                layer(self.hidden_size, self.hidden_size, cfg.noisy_std_init) if self.use_noisy else layer(self.hidden_size, self.hidden_size),
                nn.SiLU(),
                layer(self.hidden_size, action_dim, cfg.noisy_std_init) if self.use_noisy else layer(self.hidden_size, action_dim)
            )

        self.apply(init_orthogonal_weights)

    def init_hidden(self, batch_size: int) -> Union[torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
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
        
        x = self.attention(x)
        
        self.core.flatten_parameters()
        x, hidden = self.core(x, hidden)
        
        q_values = self.head(x)
        
        return q_values, hidden