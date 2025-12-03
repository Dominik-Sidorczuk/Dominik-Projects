from __future__ import annotations

import torch
import torch.nn as nn
from typing import Tuple, Union, Optional

from src.config import DRQNConfig
from .common import (
    DuelingHead, 
    ResMLP, 
    ModernSelfAttentionBlock, 
    SinusoidalPositionalEncoding, 
    init_orthogonal_weights
)

class DRQN(nn.Module):
    """Rainbow-DRQN with transformer hybrid architecture"""
    def __init__(self, obs_dim: int, action_dim: int, cfg: DRQNConfig):
        super().__init__()
        self.cfg = cfg
        self.hidden_size = cfg.hidden_size
        self.use_noisy = cfg.use_noisy

        self.embed = ResMLP(obs_dim, self.hidden_size, depth=getattr(cfg, 'encoder_depth', 1))
        if cfg.use_lstm:
            self.core = nn.LSTM(self.hidden_size, self.hidden_size, batch_first=True)
        else:
            self.core = nn.GRU(self.hidden_size, self.hidden_size, batch_first=True)

        use_attn = getattr(cfg, 'use_attention', True)
        if use_attn:
            self.pos_encoder = SinusoidalPositionalEncoding(self.hidden_size)
            self.attention = ModernSelfAttentionBlock(
                self.hidden_size, 
                num_heads=cfg.attn_heads,
                dropout=0.0
            )
        else:
            self.pos_encoder = nn.Identity()
            self.attention = nn.Identity()

        if cfg.dueling:
            self.head = DuelingHead(self.hidden_size, action_dim, self.use_noisy, cfg.noisy_std_init)
        else:
            from .common import NoisyLinear
            layer = NoisyLinear if self.use_noisy else nn.Linear
            self.head = nn.Sequential(
                nn.LayerNorm(self.hidden_size),
                layer(self.hidden_size, self.hidden_size),
                nn.SiLU(),
                layer(self.hidden_size, action_dim)
            )

        self.apply(init_orthogonal_weights)

    @property
    def core_is_lstm(self) -> bool:
        return isinstance(self.core, nn.LSTM)

    def init_hidden(self, batch: int) -> Union[torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        """Initialize hidden state on model device"""
        param = next(self.parameters())
        device = param.device
        dtype = param.dtype

        if self.core_is_lstm:
            h = torch.zeros(1, batch, self.hidden_size, device=device, dtype=dtype)
            c = torch.zeros(1, batch, self.hidden_size, device=device, dtype=dtype)
            return (h, c)
        else:
            return torch.zeros(1, batch, self.hidden_size, device=device, dtype=dtype)

    def reset_noise(self):
        """Reset noise in NoisyLinear layers"""
        if not self.use_noisy: return
        for m in self.modules():
            if m is not self and hasattr(m, 'reset_noise'):
                m.reset_noise()

    def forward(self, obs_seq: torch.Tensor, hidden=None):
        """Forward pass: [Batch, SeqLen, ObsDim] -> Q-values"""
        if obs_seq.dim() == 2:
            obs_seq = obs_seq.unsqueeze(1)

        x = self.embed(obs_seq)
        self.core.flatten_parameters()
        rnn_out, new_hidden = self.core(x, hidden)
        
        if not isinstance(self.attention, nn.Identity):
            x = self.pos_encoder(rnn_out)
            x = self.attention(x)
        else:
            x = rnn_out

        q_values = self.head(x)
        
        return q_values, new_hidden