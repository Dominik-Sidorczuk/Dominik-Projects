#!/usr/bin/env python3
from __future__ import annotations

import sys
import numpy as np
import torch
from pathlib import Path
from typing import Dict, Any, Optional

# --- PATH SETUP ---
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

# Importy z projektu
from src.config import AutoRLConfig, EvalConfig
from src.core.models.drqn import DRQN
from src.envs.make_env import make_env
from src.utils.toolkit import setup_logging, EnhancedJSONEncoder

def is_successful(ret: float, steps: int, term: bool, trunc: bool, info: dict, env_id: str) -> bool:
    """Define success criteria for elementary environments"""
    eid = env_id.lower()
    
    if "acrobot" in eid:
        return term and not trunc

    if "cartpole" in eid:
        return trunc

    if "mountaincar" in eid:
        return term and not trunc

    if "lunarlander" in eid:
        return ret >= 200.0
        
    return ret > 0.0

@torch.inference_mode()
def evaluate_loop(
    env_id: str,
    agent: torch.nn.Module,
    cfg: EvalConfig,
    seed: int,
    action_repeat: int, # Ten argument zostanie zignorowany dla precyzji
    device: str = "cuda"
) -> Dict[str, float]:
    """Evaluation loop with single-step action repeat for precision"""
    import logging
    log = logging.getLogger("EvalLoop")
    
    env = make_env(
        env_id=env_id, 
        seed=seed, 
        idx=0, 
        capture_video=False, 
        run_name="eval", 
        action_repeat=action_repeat, 
        render_mode=None
    )

    agent.eval()
    if hasattr(agent, "reset_noise"):
        agent.reset_noise()

    ep_rewards = []
    ep_lengths = []
    success_count = 0
    
    dev = torch.device(device)

    for i in range(cfg.episodes):
        obs, _ = env.reset(seed=seed + i)
        
        done = False
        total_ret = 0.0
        steps = 0
        
        hidden = None
        
        last_term = False
        last_trunc = False
        last_info = {}

        while not done:
            # Fast Tensor Conversion
            if isinstance(obs, np.ndarray):
                obs_t = torch.as_tensor(obs, device=dev, dtype=torch.float32)
            else:
                obs_t = obs.to(dev, non_blocking=True)

            if obs_t.dtype == torch.uint8:
                obs_t = obs_t.float().div_(255.0)

            if obs_t.dim() == 1: obs_t = obs_t.unsqueeze(0).unsqueeze(0)
            elif obs_t.dim() == 3: obs_t = obs_t.unsqueeze(0).unsqueeze(0)

            # Inference
            if hasattr(agent, 'get_q_values'):
                 q_values, hidden = agent.get_q_values(obs_t, hidden)
            else:
                 q_values, hidden = agent(obs_t, hidden)


            action = q_values.argmax(dim=-1).view(-1).item()

            next_obs, r, term, trunc, info = env.step(action)
            
            total_ret += float(r)
            steps += 1
            obs = next_obs
            
            done = term or trunc
            last_term, last_trunc, last_info = term, trunc, info
            
            if steps >= cfg.max_steps * 4: 
                trunc = True
                done = True
                log.warning(f"Eval break: Steps limit reached ({steps})")
        
        ep_rewards.append(total_ret)
        ep_lengths.append(steps)
        
        if is_successful(total_ret, steps, last_term, last_trunc, last_info, env_id):
            success_count += 1

    env.close()

    n = len(ep_rewards) or 1
    stats = {
        "eval_return": float(np.mean(ep_rewards)) if ep_rewards else 0.0,
        "eval_len": float(np.mean(ep_lengths)) if ep_lengths else 0.0,
        "success_rate": float(success_count / n),
        "min_return": float(np.min(ep_rewards)) if ep_rewards else 0.0,
        "max_return": float(np.max(ep_rewards)) if ep_rewards else 0.0,
    }
    
    log.info(f"📊 Eval Result: R={stats['eval_return']:.1f} | L={stats['eval_len']:.1f} | SR={stats['success_rate']*100:.1f}%")
    return stats

# --- MAIN ENTRY POINT ---
import logging
import argparse

def main():
    parser = argparse.ArgumentParser(description="AutoRL Evaluator")
    parser.add_argument("--ckpt", type=str, required=True, help="Path to checkpoint (.pt)")
    parser.add_argument("--device", type=str, default="cuda", help="Device (cpu/cuda)")
    parser.add_argument("--out", type=str, default="eval_results.json", help="Output JSON path")
    
    args, unknown = parser.parse_known_args()
    
    setup_logging(service_name="Eval")
    log = logging.getLogger("Main")

    if not os.path.isfile(args.ckpt):
        log.error(f"❌ Checkpoint not found: {args.ckpt}")
        sys.exit(1)

    # Load Checkpoint
    log.info(f"📂 Loading: {args.ckpt}")
    try:
        checkpoint = torch.load(args.ckpt, map_location=args.device)
        if 'config' in checkpoint:
            cfg = AutoRLConfig(**checkpoint['config'])
        else:
            log.warning("⚠️ No config in checkpoint! Using default.")
            cfg = AutoRLConfig()
    except Exception as e:
        log.error(f"❌ Failed to load checkpoint: {e}")
        sys.exit(1)

    cfg.device = args.device
    
    # Instantiate Model (dummy env for shapes)
    dummy_env = make_env(cfg.env.id, 0, 0, False, "test", action_repeat=1)
    obs_dim = int(np.prod(dummy_env.observation_space.shape))
    act_dim = int(dummy_env.action_space.n)
    dummy_env.close()

    try:
        model = DRQN(obs_dim, act_dim, cfg.model).to(cfg.device)
        
        state_dict = checkpoint.get('model', checkpoint.get('online', checkpoint))
        clean_state_dict = {k.replace("online_net.", "").replace("module.", ""): v for k, v in state_dict.items()}
        
        model.load_state_dict(clean_state_dict)
        log.info("✅ Model loaded.")
    except Exception as e:
        log.error(f"❌ Model instantiation failed: {e}")
        sys.exit(1)

    # Run Evaluation
    stats = evaluate_loop(
        env_id=cfg.env.id,
        agent=model,
        cfg=cfg.eval,
        seed=cfg.seed,
        action_repeat=cfg.env.action_repeat, # Przekazujemy, ale zostanie zignorowane wewnątrz
        device=cfg.device
    )

    # Save
    try:
        with open(args.out, 'w') as f:
            json.dump(stats, f, indent=4, cls=EnhancedJSONEncoder)
        log.info(f"💾 Saved to: {args.out}")
    except Exception as e:
        log.error(f"❌ Save failed: {e}")

if __name__ == "__main__":
    main()