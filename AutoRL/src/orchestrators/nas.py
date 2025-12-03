import copy
import random
import logging
import operator
from typing import Dict, Any, List, Callable, Tuple
from functools import partial

from src.config import NASConfig, AutoRLConfig

log = logging.getLogger(__name__)

def _mutate_choice(obj: Any, attr: str, options: Tuple[Any, ...], current_val: Any) -> Any:
    """Select from list, rejecting current value"""
    for _ in range(5):
        val = random.choice(options)
        if val != current_val:
            setattr(obj, attr, val)
            return val
    setattr(obj, attr, options[0])
    return options[0]

def _mutate_interval(obj: Any, attr: str, low: float, high: float, min_v: float, max_v: float, current_val: Any) -> Any:
    """Numeric mutation with clipping"""
    factor = random.uniform(low, high)
    new_val = current_val * factor
    
    if new_val < min_v: new_val = min_v
    elif new_val > max_v: new_val = max_v
    
    if isinstance(current_val, int):
        new_val = int(new_val)
        
    setattr(obj, attr, new_val)
    return new_val

def _mutate_toggle(obj: Any, attr: str, _unused1: Any, _unused2: Any, current_val: Any) -> Any:
    """Boolean inversion"""
    new_val = not current_val
    setattr(obj, attr, new_val)
    return new_val

class EvolutionaryArchitect:
    """Zero-overhead NAS with pre-compiled mutations"""
    def __init__(self, cfg: NASConfig):
        self.cfg = cfg
        self._enabled = cfg.enabled
        self._panic_threshold = cfg.panic_threshold
        self._stable_threshold = cfg.stable_threshold
        
        self._log_info = log.isEnabledFor(logging.INFO)

        self._risky_ops = self._compile_mutations(cfg.risky_mutations)
        self._safe_ops = self._compile_mutations(cfg.safe_mutations)

    def _compile_mutations(self, mutation_defs: List[Dict[str, Any]]) -> List[Callable[[Any], None]]:
        """Compile mutation definitions to closures"""
        compiled_ops = []
        
        for m in mutation_defs:
            path = m["param"]
            kind = m["kind"]
            
            if '.' in path:
                parent_path, attr_name = path.rsplit('.', 1)
                parent_getter = operator.attrgetter(parent_path)
            else:
                parent_getter = lambda x: x
                attr_name = path
            handler = None
            args = ()

            if kind == "choice":
                options = tuple(m["choices"])
                if len(options) < 2: continue
                handler = _mutate_choice
                args = (options,)

            elif kind == "interval":
                low = m.get("low", 0.9)
                high = m.get("high", 1.1)
                min_v = m.get("min", -float('inf'))
                max_v = m.get("max", float('inf'))
                handler = _mutate_interval
                args = (low, high, min_v, max_v)

            elif kind == "toggle":
                handler = _mutate_toggle
                args = (None, None)

            if handler:
                op = self._create_op(parent_getter, attr_name, handler, args, path)
                compiled_ops.append(op)
                
        return compiled_ops

    def _create_op(self, parent_getter, attr_name, handler, args, debug_path):
        """Create mutation function closure"""
        log_enabled = self._log_info
        
        def operation(root_config):
            try:
                target = parent_getter(root_config)
                current_val = getattr(target, attr_name)
                new_val = handler(target, attr_name, *args, current_val)
                
                if log_enabled and new_val != current_val:
                    log.info(f"[NAS] Mutated {debug_path}: {current_val} -> {new_val}")
                    
            except Exception:
                pass
                
        return operation

    def mutate(self, parent_config: AutoRLConfig, performance_metric: float) -> AutoRLConfig:
        """High-performance mutation execution"""
        if not self._enabled:
            return parent_config

        if hasattr(parent_config, 'model_copy'):
            child_config = parent_config.model_copy(deep=True)
        else:
            child_config = copy.deepcopy(parent_config)
        
        ops_to_run = None

        if performance_metric < self._panic_threshold:
            if self._log_info:
                log.info(f"[NAS] PANIC: {performance_metric:.2f} < {self._panic_threshold}")
            ops_to_run = self._risky_ops
            
        elif performance_metric < self._stable_threshold:
            if self._log_info:
                log.info(f"[NAS] STABLE: {performance_metric:.2f} < {self._stable_threshold}")
            ops_to_run = self._safe_ops
            
        if ops_to_run:
            op = random.choice(ops_to_run)
            op(child_config)
            
        return child_config