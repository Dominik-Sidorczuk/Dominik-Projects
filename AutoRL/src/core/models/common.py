from __future__ import annotations

import math
import logging
import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional, Tuple, Type

log = logging.getLogger(__name__)

class NoisyLinear(nn.Module):
    """Factorized Gaussian noise for exploration"""
    def __init__(self, in_features: int, out_features: int, std_init: float = 0.5):
        super().__init__()
        self.in_features = in_features
        self.out_features = out_features
        self.std_init = std_init

        self.weight_mu = nn.Parameter(torch.empty(out_features, in_features))
        self.bias_mu = nn.Parameter(torch.empty(out_features))
        
        self.weight_sigma = nn.Parameter(torch.empty(out_features, in_features))
        self.bias_sigma = nn.Parameter(torch.empty(out_features))
        
        self.register_buffer('weight_epsilon', torch.empty(out_features, in_features))
        self.register_buffer('bias_epsilon', torch.empty(out_features))
        
        self.reset_parameters()
        self.reset_noise()

    def reset_parameters(self):
        mu_range = 1 / math.sqrt(self.in_features)
        
        self.weight_mu.data.uniform_(-mu_range, mu_range)
        self.bias_mu.data.uniform_(-mu_range, mu_range)
        
        self.weight_sigma.data.fill_(self.std_init / math.sqrt(self.in_features))
        self.bias_sigma.data.fill_(self.std_init / math.sqrt(self.out_features))

    def _scale_noise(self, size: int) -> torch.Tensor:
        x = torch.randn(size, device=self.weight_mu.device)
        return x.sign().mul_(x.abs().sqrt_())

    def reset_noise(self):
        epsilon_in = self._scale_noise(self.in_features)
        epsilon_out = self._scale_noise(self.out_features)
        
        self.weight_epsilon.copy_(epsilon_out.ger(epsilon_in))
        self.bias_epsilon.copy_(epsilon_out)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if self.training:
            return F.linear(
                x,
                self.weight_mu + self.weight_sigma * self.weight_epsilon,
                self.bias_mu + self.bias_sigma * self.bias_epsilon
            )
        else:
            return F.linear(x, self.weight_mu, self.bias_mu)

def init_orthogonal_weights(m: nn.Module):
    """Orthogonal initialization for layers"""
    if isinstance(m, nn.Linear):
        nn.init.orthogonal_(m.weight, gain=math.sqrt(2))
        if m.bias is not None:
            nn.init.zeros_(m.bias)
            
    elif isinstance(m, NoisyLinear):
        nn.init.orthogonal_(m.weight_mu, gain=math.sqrt(2))
        if m.bias_mu is not None:
            nn.init.zeros_(m.bias_mu)
            
    elif isinstance(m, (nn.LSTM, nn.GRU)):
        for name, param in m.named_parameters():
            if 'weight_ih' in name:
                nn.init.orthogonal_(param, gain=1.0)
            elif 'weight_hh' in name:
                nn.init.orthogonal_(param, gain=1.0)
            elif 'bias' in name:
                nn.init.zeros_(param)
                n = param.size(0)
                if isinstance(m, nn.LSTM):
                    start, end = n // 4, n // 2
                    param.data[start:end].fill_(1.0)
                elif isinstance(m, nn.GRU):
                    pass

class SinusoidalPositionalEncoding(nn.Module):
    """Adds temporal information for attention"""
    def __init__(self, d_model: int, max_len: int = 5000):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))
        
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        
        self.register_buffer('pe', pe)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        T = x.size(1)
        return x + self.pe[:T, :].unsqueeze(0)

class ModernSelfAttentionBlock(nn.Module):
    """FlashAttention-ready transformer block"""
    def __init__(self, embed_dim: int, num_heads: int = 4, dropout: float = 0.0):
        super().__init__()
        self.embed_dim = embed_dim
        self.num_heads = num_heads
        self.head_dim = embed_dim // num_heads
        assert self.head_dim * num_heads == self.embed_dim, "Dim must be divisible by heads"

        self.qkv = nn.Linear(embed_dim, 3 * embed_dim)
        self.proj = nn.Linear(embed_dim, embed_dim)
        
        self.norm1 = nn.LayerNorm(embed_dim)
        self.norm2 = nn.LayerNorm(embed_dim)
        
        self.mlp = nn.Sequential(
            nn.Linear(embed_dim, 4 * embed_dim),
            nn.GELU(),
            nn.Linear(4 * embed_dim, embed_dim)
        )
        self.dropout = dropout

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        B, T, C = x.shape
        
        residual = x
        x_norm = self.norm1(x)
        
        qkv = self.qkv(x_norm).view(B, T, 3, self.num_heads, self.head_dim).permute(2, 0, 3, 1, 4)
        q, k, v = qkv[0], qkv[1], qkv[2]
        
        attn_out = F.scaled_dot_product_attention(
            q, k, v,
            dropout_p=self.dropout if self.training else 0.0,
            is_causal=True 
        )
        
        attn_out = attn_out.transpose(1, 2).reshape(B, T, C)
        x = residual + self.proj(attn_out)
        
        residual = x
        x_norm = self.norm2(x)
        x = residual + self.mlp(x_norm)
        
        return x

class ResBlock(nn.Module):
    """Lightweight residual block"""
    def __init__(self, dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.LayerNorm(dim),
            nn.Linear(dim, dim),
            nn.SiLU(),
            nn.Linear(dim, dim)
        )

    def forward(self, x):
        return x + self.net(x)

class ResMLP(nn.Module):
    """Modern encoder with residual connections"""
    def __init__(self, input_dim: int, hidden_dim: int, depth: int = 1):
        super().__init__()
        self.proj = nn.Linear(input_dim, hidden_dim)
        self.blocks = nn.Sequential(*[ResBlock(hidden_dim) for _ in range(depth)])
        self.act = nn.SiLU()

    def forward(self, x):
        x = self.proj(x)
        x = self.blocks(x)
        return self.act(x)

class DuelingHead(nn.Module):
    """Dueling architecture for value and advantage streams"""
    def __init__(self, hidden: int, action_dim: int, use_noisy: bool = False, std_init: float = 0.5):
        super().__init__()
        def layer(i, o):
            return NoisyLinear(i, o, std_init) if use_noisy else nn.Linear(i, o)

        self.adv_head = nn.Sequential(
            nn.LayerNorm(hidden),
            layer(hidden, hidden),
            nn.SiLU(),
            layer(hidden, action_dim)
        )
        
        self.val_head = nn.Sequential(
            nn.LayerNorm(hidden),
            layer(hidden, hidden),
            nn.SiLU(),
            layer(hidden, 1)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        adv = self.adv_head(x)
        val = self.val_head(x)
        return val + adv - adv.mean(dim=-1, keepdim=True)

def instantiate_model(cfg_model, obs_dim, action_dim, device) -> Tuple[nn.Module, Type]:
    """Create model instance (DRQN/R2D2)"""
    from src.core.models.drqn import DRQN
    
    model = DRQN(obs_dim, action_dim, cfg=cfg_model)
    
    return model.to(device), type(model)