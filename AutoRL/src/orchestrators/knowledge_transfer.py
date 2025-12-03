import torch
import torch.nn as nn
import logging
from typing import Dict, Any, Optional, Tuple, List

from src.config import AutoRLConfig

log = logging.getLogger(__name__)

class NeuralSurgeon:
    """Neural weight transplantation with knowledge preservation"""
    
    def __init__(self, cfg: AutoRLConfig):
        self.cfg = cfg

    def perform_transplant(self, parent_state: Dict[str, torch.Tensor], child_model: nn.Module) -> None:
        child_state = child_model.state_dict()
        child_core_type = self._detect_core_type(child_model)
        
        transplant_count = 0
        graft_count = 0

        common_keys = set(child_state.keys()) & set(parent_state.keys())

        for name in common_keys:
            child_param = child_model.get_parameter(name) if name in dict(child_model.named_parameters()) else child_state[name]
            parent_param = parent_state[name]
            
            if child_param.dtype != parent_param.dtype:
                parent_param = parent_param.to(child_param.dtype)

            if parent_param.shape == child_param.shape:
                with torch.no_grad():
                    child_param.data.copy_(parent_param.data)
                transplant_count += 1
                
            elif len(parent_param.shape) == len(child_param.shape):
                is_recurrent = "core" in name and child_core_type in ["LSTM", "GRU"]
                
                if is_recurrent:
                    success = self._graft_gated_param(child_param, parent_param, name, child_core_type)
                    if not success:
                        self._graft_simple_param(child_param, parent_param, name)
                else:
                    self._graft_simple_param(child_param, parent_param, name)
                
                graft_count += 1
            
        log.info(f"[SURGEON] Transplantacja zakończona. Direct: {transplant_count}, Grafts: {graft_count}")

    def _detect_core_type(self, model: nn.Module) -> str:
        if hasattr(model, "core"):
            if isinstance(model.core, nn.LSTM): return "LSTM"
            elif isinstance(model.core, nn.GRU): return "GRU"
        return "Linear"

    def _graft_simple_param(self, child_param: nn.Parameter, parent_param: torch.Tensor, layer_name: str):
        """Intelligent copying with noise only for new weights"""
        with torch.no_grad():
            slices = tuple(slice(0, min(ds, ss)) for ds, ss in zip(child_param.shape, parent_param.shape))
            
            child_param.data[slices].copy_(parent_param.data[slices])
            
            if 'weight' in layer_name and self.cfg.grafting_noise_scale > 0:
                noise = torch.randn_like(child_param.data[slices]) * (self.cfg.grafting_noise_scale * 0.1)
                child_param.data[slices].add_(noise)

    def _graft_gated_param(self, child_param: nn.Parameter, parent_param: torch.Tensor, layer_name: str, core_type: str) -> bool:
        """Handle LSTM/GRU with gate-aware grafting"""
        gates_count = 4 if core_type == "LSTM" else 3
        
        if child_param.shape[0] % gates_count != 0 or parent_param.shape[0] % gates_count != 0:
            return False

        with torch.no_grad():
            p_gates = parent_param.chunk(gates_count, dim=0)
            c_gates = child_param.chunk(gates_count, dim=0)
            
            for p_gate, c_gate in zip(p_gates, c_gates):
                slices = tuple(slice(0, min(ds, ss)) for ds, ss in zip(c_gate.shape, p_gate.shape))
                
                c_gate.data[slices].copy_(p_gate.data[slices])
                
                if 'weight' in layer_name and self.cfg.grafting_noise_scale > 0:
                    noise = torch.randn_like(c_gate.data[slices]) * (self.cfg.grafting_noise_scale * 0.05)
                    c_gate.data[slices].add_(noise)
                    
        return True

    def distill_knowledge(self, teacher: nn.Module, student: nn.Module, replay_sample: Dict[str, Any], optimizer: torch.optim.Optimizer):
        """Hot-start behavior cloning on single batch"""
        if not self.cfg.distillation_enabled or replay_sample is None:
            return

        steps = self.cfg.distillation_steps
        if steps <= 0: return

        try:
            obs = replay_sample['obs']
            with torch.no_grad():
                teacher_out, _ = teacher(obs)
            
            loss_fn = nn.MSELoss()
            
            teacher.eval()
            student.train()
            
            loss = None
            for _ in range(steps):
                optimizer.zero_grad()
                student_out, _ = student(obs)
                
                loss = loss_fn(student_out, teacher_out)
                loss.backward()
                optimizer.step()
            
            if loss is not None:
                log.info(f"[SURGEON] Distillation done. Final Loss: {loss.item():.4f}")
                
        except Exception as e:
            log.warning(f"[SURGEON] Distillation failed: {e}")