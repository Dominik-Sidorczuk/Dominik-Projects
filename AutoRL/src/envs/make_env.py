from __future__ import annotations

import logging
import gymnasium as gym
import numpy as np
from typing import Optional, Callable

# Importy do wektoryzacji
from gymnasium.vector import SyncVectorEnv, AsyncVectorEnv

log = logging.getLogger(__name__)

class ActionRepeatWrapper(gym.Wrapper):
    """Repeat action for N frames and return sum of rewards"""
    def __init__(self, env: gym.Env, repeat: int):
        super().__init__(env)
        assert repeat >= 1, "Action repeat musi być >= 1"
        self.repeat = repeat

    def step(self, action):
        total_reward = 0.0
        terminated = False
        truncated = False
        obs = None
        info = {}
        
        for _ in range(self.repeat):
            obs, reward, term, trunc, info = self.env.step(action)
            total_reward += float(reward)
            
            if term or trunc:
                terminated = bool(term)
                truncated = bool(trunc)
                break
        
        return obs, total_reward, terminated, truncated, info

def make_env(
    env_id: str, 
    seed: int, 
    idx: int, 
    capture_video: bool, 
    run_name: str, 
    action_repeat: int = 1, 
    frame_stack: int = 1,
    clip_reward: bool = False,
    render_mode: Optional[str] = None
) -> gym.Env:
    """Create single environment instance"""
    def thunk() -> gym.Env:
        # 1. Inicjalizacja z opcjonalnym nagrywaniem wideo (tylko dla env 0)
        if capture_video and idx == 0:
            env = gym.make(env_id, render_mode="rgb_array")
            env = gym.wrappers.RecordVideo(env, f"videos/{run_name}", disable_logger=True)
        else:
            env = gym.make(env_id, render_mode=render_mode)

        # 2. Podstawowe statystyki (Return, Length, Time) - niezbędne dla logów
        env = gym.wrappers.RecordEpisodeStatistics(env)
        
        # 3. Action Repeat (Frame Skip)
        # Dodajemy tylko jeśli faktycznie jest potrzebny (>1)
        if action_repeat > 1:
            env = ActionRepeatWrapper(env, repeat=action_repeat)

        # 4. Frame Stack
        if frame_stack > 1:
            env = gym.wrappers.FrameStack(env, frame_stack)

        # 5. Clip Reward
        if clip_reward:
            env = gym.wrappers.ClipReward(env, min_reward=-1.0, max_reward=1.0)
        
        env.action_space.seed(seed)
        env.observation_space.seed(seed)
        
        return env

    return thunk()


def make_vector_env(
    env_id: str, 
    num_envs: int, 
    seed: int, 
    action_repeat: int, 
    frame_stack: int = 1,
    clip_reward: bool = False,
    vector_mode: str = "async", 
    capture_video: bool = False, 
    run_name: str = "test"
) -> gym.vector.VectorEnv:
    """Create vectorized environment (sync/async)"""
    envs = [
        lambda i=i: make_env(
            env_id, 
            seed + i, 
            i, 
            capture_video, 
            run_name, 
            action_repeat,
            frame_stack,
            clip_reward
        ) for i in range(num_envs)
    ]

    force_sync = num_envs == 1 or vector_mode.lower() in ["sync", "synchronous"]
    if force_sync:
        log.debug(f"Creating SyncVectorEnv with {num_envs} envs...")
        return SyncVectorEnv(envs)
    else:
        log.debug(f"Creating AsyncVectorEnv with {num_envs} envs...")
        return AsyncVectorEnv(envs)