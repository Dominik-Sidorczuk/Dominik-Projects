from __future__ import annotations
from typing import List, Optional, Any, Dict, Literal
from typing_extensions import Annotated
from pydantic import BaseModel, Field, ConfigDict, model_validator, field_validator

class MutationRule(BaseModel):
    """Reguła mutacji (PBT)."""
    model_config = ConfigDict(extra='forbid')

    param: str
    min: Optional[float] = None
    max: Optional[float] = None
    type: Literal["float", "int"] = "float"

    @model_validator(mode='after')
    def _check_bounds(self) -> 'MutationRule':
        if self.min is not None and self.max is not None and self.min > self.max:
            raise ValueError(f"MutationRule bounds invalid for '{self.param}': min({self.min}) > max({self.max})")
        return self

class PERConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    enabled: bool = True
    use_hp: bool = True
    alpha: Annotated[float, Field(ge=0.0, le=1.0)] = 0.6
    beta_start: float = 0.4
    beta_steps: Annotated[int, Field(gt=0)] = 250_000

class AgentConfig(BaseModel):
    model_config = ConfigDict(extra='forbid', validate_assignment=True)

    use_noisy: bool = True
    eps_start: Annotated[float, Field(ge=0.0, le=1.0)] = 1.0
    eps_min: Annotated[float, Field(ge=0.0, le=1.0)] = 0.1
    eps_decay: Annotated[int, Field(gt=0)] = 250_000
    eps_anneal_source: Literal["agent_calls", "env_steps", "external"] = "agent_calls"

    use_cycles: bool = True
    cycle_length: Annotated[int, Field(gt=0)] = 250_000 
    cycle_decay: Annotated[float, Field(ge=0.0, le=1.0)] = 0.5

    @model_validator(mode='after')
    def _eps_sanity(self) -> 'AgentConfig':
        if self.use_noisy:
            object.__setattr__(self, 'eps_start', 0.0)
            object.__setattr__(self, 'eps_min', 0.0)
        elif self.eps_min >= self.eps_start:
            raise ValueError(f"AgentConfig: eps_min ({self.eps_min}) musi być < eps_start ({self.eps_start}).")
        return self

    @property
    def eps_range(self) -> float:
        return max(0.0, float(self.eps_start - self.eps_min))

    @property
    def greedy_epsilon(self) -> float:
        return 0.0 if self.use_noisy else float(self.eps_min)

class TrainConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    total_steps: Annotated[int, Field(gt=0)] = 400_000
    num_envs: Annotated[int, Field(gt=0)] = 92 
    buffer_size: Annotated[int, Field(gt=0)] = 150_000
    batch_size: Annotated[int, Field(gt=0)] = 384
    seq_len: Annotated[int, Field(gt=0)] = 48
    
    rollout_steps: Annotated[int, Field(gt=0)] = 100
    gradient_steps: Annotated[int, Field(ge=1)] = 64
    learning_starts: Annotated[int, Field(ge=0)] = 10_000
    rnn_burn_in: Annotated[int, Field(ge=0)] = 12          
    
    gamma: Annotated[float, Field(ge=0.0, le=0.9999)] = 0.994
    n_step: Annotated[int, Field(ge=1, le=10)] = 3
    
    tau: Annotated[float, Field(gt=0.0, le=1.0)] = 0.005
    target_update_freq: Annotated[int, Field(gt=0)] = 5_000
    log_freq: Annotated[int, Field(gt=0)] = 5_000
    checkpoint_freq: Annotated[int, Field(gt=0)] = 15_000
    compile: bool = False
    stride: Optional[int] = 8
    eval_freq: Annotated[int, Field(gt=0)] = 15_000
    max_grad_norm: Annotated[float, Field(gt=0.0)] = 8.0
    amp: bool = False 

    @model_validator(mode='after')
    def _check_train_consistency(self) -> 'TrainConfig':
        min_buffer = self.num_envs * self.seq_len
        if self.buffer_size < min_buffer:
             raise ValueError(f"TrainConfig: buffer_size ({self.buffer_size}) must be >= num_envs * seq_len ({min_buffer}).")
        
        if self.batch_size > self.buffer_size:
            raise ValueError("TrainConfig: batch_size nie może być > buffer_size.")
        if self.seq_len <= self.rnn_burn_in:
            raise ValueError("TrainConfig: seq_len musi być > rnn_burn_in.")
        return self

class EnvConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    id: str = "Acrobot-v1"
    vector_mode: Literal["async", "sync"] = "sync"
    action_repeat: Annotated[int, Field(ge=1)] = 4
    frame_stack: Annotated[int, Field(ge=1)] = 1
    clip_reward: bool = False

    @field_validator('vector_mode', mode='before')
    def _normalize_vector_mode(cls, v: Any) -> str:
        if isinstance(v, str):
            v = v.strip().lower()
            if v in {"asynchronous", "async"}: return "async"
            if v in {"synchronous", "sync"}: return "sync"
        return v

class DRQNConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    hidden_size: Annotated[int, Field(gt=0)] = 256
    use_lstm: bool = True
    use_noisy: bool = True
    encoder_depth: Annotated[int, Field(ge=1, le=8)] = 2
    use_attention: bool = False
    attn_heads: Annotated[int, Field(ge=1, le=16)] = 4
    dueling: bool = True
    noisy_std_init: Annotated[float, Field(gt=0.0)] = 2.0
    
    atoms: int = 1
    v_min: float = -600.0
    v_max: float = 100.0

class OptimizerConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    lr: Annotated[float, Field(gt=0.0)] = 1e-4
    eps: Annotated[float, Field(gt=0.0)] = 2e-6
    weight_decay: Annotated[float, Field(ge=0.0)] = 0.005

class EvalConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    episodes: Annotated[int, Field(ge=1)] = 5
    max_steps: Annotated[int, Field(ge=1)] = 2_000
    epsilon: Annotated[float, Field(ge=0.0, le=1.0)] = 0.001
    greedy: bool = True
    ckpt_path: Optional[str] = None
    seed: Optional[int] = None
    interval: int = 20000

class PBTConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    enabled: bool = True
    population_size: Annotated[int, Field(ge=1)] = 6
    interval: Annotated[int, Field(ge=1)] = 45_000
    mutation_rate: Annotated[float, Field(ge=0.0, le=1.0)] = 0.125
    quantile_fraction: Annotated[float, Field(ge=0.0, le=1.0)] = 0.25
    perturb_factor: Annotated[float, Field(gt=1.0)] = 1.45
    base_perturb_factor: Annotated[float, Field(gt=1.0)] = 1.45
    acceleration: Annotated[float, Field(gt=1.0)] = 1.4
    max_perturb_factor: Annotated[float, Field(gt=1.0)] = 3.0
    dampening: Annotated[float, Field(gt=0.0, lt=1.0)] = 0.8
    mix_ratio: Annotated[float, Field(ge=0.0, le=1.0)] = 0.5
    crossover_enabled: bool = True
    
    ref_enabled: bool = True
    c_stable: float = 0.5
    c_risk: float = 0.5

    mutation_bounds: List[MutationRule] = Field(
        default_factory=lambda: [
            MutationRule(param="per.alpha", min=0.35, max=0.75, type="float"),
            MutationRule(param="per.beta_start", min=0.4, max=1.0, type="float"),
            MutationRule(param="model.noisy_std_init", min=0.15, max=1.65, type="float"),
            MutationRule(param="optimizer.lr", min=3e-5, max=5e-3, type="float"),
        ]
    )

class NASConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    enabled: bool = False
    population_size: Annotated[int, Field(ge=1)] = 6
    generations: Annotated[int, Field(ge=1)] = 20

    panic_threshold: float = -50.0
    stable_threshold: float = 0.0

    risky_mutations: List[Dict[str, Any]] = Field(
        default_factory=lambda: [
            {"param": "model.hidden_size", "kind": "interval", "low": 0.8, "high": 1.25},
            {"param": "agent.use_noisy", "kind": "toggle"},
        ]
    )
    safe_mutations: List[Dict[str, Any]] = Field(
        default_factory=lambda: [
            {"param": "optimizer.lr", "kind": "interval", "low": 0.9, "high": 1.1},
            {"param": "train.n_step", "kind": "choice", "choices": [1, 3, 5]},
        ]
    )

    @model_validator(mode='after')
    def _validate_mutations(self) -> 'NASConfig':
        def _check(rule: Dict[str, Any]) -> None:
            k = rule.get('kind')
            if k == 'interval':
                if 'low' not in rule or 'high' not in rule:
                    raise ValueError("NAS mutation 'interval' wymaga pól 'low' i 'high'.")
                if float(rule['low']) > float(rule['high']):
                    raise ValueError("NAS mutation 'interval': low > high.")
            elif k == 'choice':
                if 'choices' not in rule or not isinstance(rule['choices'], list) or len(rule['choices']) == 0:
                    raise ValueError("NAS mutation 'choice' wymaga niepustej listy 'choices'.")
            elif k == 'toggle':
                pass
            else:
                raise ValueError(f"NAS mutation: nieznany kind='{k}'.")
        for r in self.risky_mutations:
            _check(r)
        for r in self.safe_mutations:
            _check(r)
        return self

class ImmuneConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    enabled: bool = True
    window_size: Annotated[int, Field(ge=10, le=1000)] = 100
    divergence_slope_threshold: float = 0.08
    grad_explosion_threshold: float = 16.0
    q_spread_threshold: float = 16.0
    entropy_crash_threshold: float = 0.125
    plateau_slope_threshold: float = 1e-3
    sedative_lr_factor: Annotated[float, Field(gt=0.0, lt=1.0)] = 0.75
    stimulant_boost: Annotated[float, Field(gt=1.0)] = 1.65
    history_len: Annotated[int, Field(ge=10, le=1000)] = 100
    smoothing: Annotated[float, Field(ge=0.0, le=1.0)] = 0.25
    panic_cooldown: Annotated[int, Field(ge=1)] = 100
    chronic_cooldown: Annotated[int, Field(ge=1)] = 10000

class TransferConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    enabled: bool = False
    source_checkpoint: Optional[str] = None
    layers_to_freeze: List[str] = []
    distillation_steps: Annotated[int, Field(ge=0)] = 50
    distillation_lr: Annotated[float, Field(gt=0.0)] = 1e-3
    distillation_enabled: bool = False
    grafting_noise_scale: Annotated[float, Field(ge=0.0)] = 0.01

class AuditConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')
    
    enabled: bool = True
    log_dir: str = "audit_logs"
    filename: str = "events.jsonl"
    max_queue_size: int = 10000
    flush_interval: float = 5.0
    buffer_size: int = 100

class StabilityConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')
    
    nan_guard: bool = True
    oom_guard: bool = True
    gradient_guard: bool = True

class AutoRLConfig(BaseModel):
    model_config = ConfigDict(extra='forbid', validate_assignment=True)

    mode: Literal["train", "population"] = "population"
    seed: int = 42
    device: Literal["cpu", "cuda"] = "cuda"
    out_dir: str = "outputs"
    
    cuda_deterministic: bool = False
    cuda_benchmark: bool = False

    train: TrainConfig = Field(default_factory=TrainConfig)
    agent: AgentConfig = Field(default_factory=AgentConfig)
    env: EnvConfig = Field(default_factory=EnvConfig)
    model: DRQNConfig = Field(default_factory=DRQNConfig)
    optimizer: OptimizerConfig = Field(default_factory=OptimizerConfig)
    eval: EvalConfig = Field(default_factory=EvalConfig)

    per: PERConfig = Field(default_factory=PERConfig)
    pbt: PBTConfig = Field(default_factory=PBTConfig)
    nas: NASConfig = Field(default_factory=NASConfig)
    immune: ImmuneConfig = Field(default_factory=ImmuneConfig)

    orchestrators: Dict[str, TransferConfig] = Field(default_factory=lambda: {"knowledge_transfer": TransferConfig()})
    telemetry: Dict[str, AuditConfig] = Field(default_factory=lambda: {"audit": AuditConfig()})
    stability: StabilityConfig = Field(default_factory=StabilityConfig)