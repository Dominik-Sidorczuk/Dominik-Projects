from __future__ import annotations

import random
import logging
import math
from collections import defaultdict
from typing import Any, Dict, Optional, List, Union

from src.config import AutoRLConfig, PBTConfig

log = logging.getLogger(__name__)

class PBTOrchestrator:
    """PBT with adaptive momentum, immune safety and dynamic perturbation"""

    def __init__(self, cfg: PBTConfig):
        self.cfg = cfg
        self.agent_states = defaultdict(lambda: {
            'params': defaultdict(lambda: {'dir': 0, 'vel': 0.0}),
            'prev_score': -float('inf')
        })

    def reset_local_momentum(self, agent_id: int, mentor_score: float) -> None:
        """Reset momentum after exploit operation"""
        state = self.agent_states[agent_id]
        state['prev_score'] = mentor_score
        state['params'].clear() 
        log.debug(f"[PBT] Agent #{agent_id}: Momentum reset (New Life).")

    def get_mentor(self, population: List[Dict], top_quantile: float = 0.25) -> Dict:
        """Returns random agent from top K%"""
        sorted_pop = sorted(population, key=lambda x: x.get('score', -float('inf')), reverse=True)
        
        cutoff = max(1, int(len(sorted_pop) * top_quantile))
        top_performers = sorted_pop[:cutoff] if sorted_pop else population
        
        return random.choice(top_performers)

    def crossover(self, parent1_cfg: AutoRLConfig, parent2_cfg: AutoRLConfig) -> AutoRLConfig:
        """Genetic crossover: arithmetic for floats, uniform for others"""
        if not self.cfg.crossover_enabled:
            return parent1_cfg.model_copy(deep=True)

        child = parent1_cfg.model_copy(deep=True)
        
        for rule in self.cfg.mutation_bounds:
            path = rule.param
            val1 = self._get_deep(parent1_cfg, path)
            val2 = self._get_deep(parent2_cfg, path)
            
            if val1 is None or val2 is None: continue
            
            if rule.type == "float":
                new_val = (float(val1) + float(val2)) / 2.0
                self._set_deep(child, path, new_val)
            
            elif random.random() < 0.5:
                self._set_deep(child, path, val2)
                
        return child

    def perturb_single_agent(self, 
                             current_cfg: AutoRLConfig, 
                             current_score: float, 
                             agent_id: int, 
                             dynamic_perturb_factor: Optional[float] = None, 
                             is_sick: bool = False) -> Optional[AutoRLConfig]:
        """Evolutionary explore with adaptive momentum and immune safety"""
        if not self.cfg.enabled or not math.isfinite(current_score):
            return None

        new_cfg = current_cfg.model_copy(deep=True)
        state = self.agent_states[agent_id]
        prev_score = state['prev_score']
        
        # Czy nastąpiła poprawa od ostatniej mutacji?
        improved = (current_score > prev_score)
        state['prev_score'] = current_score 
        
        mutated_info = []

        for rule in self.cfg.mutation_bounds:
            path = rule.param
            p_state = state['params'][path]
            
            if p_state['vel'] == 0.0:
                p_state['vel'] = self.cfg.base_perturb_factor
                p_state['dir'] = 1 if random.random() < 0.5 else -1

            if dynamic_perturb_factor is not None:
                p_state['vel'] = dynamic_perturb_factor
                if random.random() < 0.3:
                    p_state['dir'] *= -1
            else:
                if improved:
                    p_state['vel'] = min(p_state['vel'] * self.cfg.acceleration, self.cfg.max_perturb_factor)
                else:
                    p_state['dir'] *= -1
                    p_state['vel'] = max(self.cfg.base_perturb_factor, p_state['vel'] * 0.8)

            if is_sick:
                risky_params = ["lr", "max_grad_norm", "clip_param"]
                if any(risk in path for risk in risky_params):
                    if p_state['dir'] > 0:
                        p_state['dir'] = -1
                        p_state['vel'] = self.cfg.base_perturb_factor
            
            curr_val = self._get_deep(new_cfg, path)
            if curr_val is None: continue
            
            change_factor = 1.0 + (p_state['dir'] * p_state['vel'])
            
            if rule.type == "int":
                delta = int(math.ceil(curr_val * p_state['vel'])) * p_state['dir']
                
                if delta == 0: delta = p_state['dir']
                
                new_val = int(curr_val + delta)
            else:
                new_val = float(curr_val * change_factor)

            if rule.min is not None: new_val = max(rule.min, new_val)
            if rule.max is not None: new_val = min(rule.max, new_val)
            
            if rule.type == "int":
                new_val = int(new_val)

            if new_val != curr_val:
                self._set_deep(new_cfg, path, new_val)
                mutated_info.append(f"{path.split('.')[-1]}: {curr_val:.2e}->{new_val:.2e}")
        
        # [ZMIANA] Wypisanie zebranych informacji o mutacjach
        if mutated_info:
            log.info(f"🧬 [PBT] Mutation #{agent_id}: " + " | ".join(mutated_info))
            
        return new_cfg

    def _get_deep(self, obj, path) -> Any:
        try:
            for part in path.split('.'):
                obj = getattr(obj, part)
            return obj
        except AttributeError:
            return None

    def _set_deep(self, obj, path, value) -> None:
        try:
            parts = path.split('.')
            curr = obj
            for part in parts[:-1]:
                curr = getattr(curr, part)
            setattr(curr, parts[-1], value)
        except AttributeError:
            pass