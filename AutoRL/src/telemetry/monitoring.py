from __future__ import annotations

import os
import sys
import time
import json
import queue
import threading
import numpy as np
from pathlib import Path
from typing import Any, Dict, List, Optional
from dataclasses import dataclass
from collections import deque

from src.config import AuditConfig
import logging

log = logging.getLogger(__name__)

class Audit:
    """Asynchronous audit logger with queue buffering"""

    def __init__(self, cfg: AuditConfig) -> None:
        self.cfg = cfg
        self.path = Path(cfg.log_dir) / cfg.filename
        
        self.queue = queue.Queue(maxsize=cfg.max_queue_size)
        self._stop_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None
        
        if self.cfg.enabled:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                self._start()
            except Exception as e:
                log.warning(f"⚠️ [AUDIT INIT FAIL] Disabling audit: {e}")
                self.cfg.enabled = False

    def _start(self):
        self._worker_thread = threading.Thread(target=self._loop, daemon=True, name="Audit-Worker")
        self._worker_thread.start()

    def log(self, event_type: str, details: Dict[str, Any]) -> None:
        """Add event to queue (non-blocking)"""
        if not self.cfg.enabled: return
        
        event = {
            "ts": time.time(),
            "type": event_type,
            "pid": os.getpid(),
            "data": details
        }
        try:
            self.queue.put_nowait(json.dumps(event))
        except queue.Full:
            log.warning("⚠️ [AUDIT DROPPED] Queue full!")

    def _loop(self):
        """Main write loop thread"""
        buffer = []
        last_flush = time.time()
        
        while not self._stop_event.is_set() or not self.queue.empty():
            try:
                msg = self.queue.get(timeout=1.0)
                buffer.append(msg)
                
                now = time.time()
                
                time_trigger = (now - last_flush) > self.cfg.flush_interval
                size_trigger = len(buffer) >= self.cfg.buffer_size
                
                if (size_trigger or time_trigger) and buffer:
                    self._flush_buffer(buffer)
                    buffer.clear()
                    last_flush = now
                    
            except queue.Empty:
                if buffer and (time.time() - last_flush > self.cfg.flush_interval):
                    self._flush_buffer(buffer)
                    buffer.clear()
                    last_flush = time.time()
                continue
            except Exception as e:
                log.error(f"❌ [AUDIT THREAD ERROR] {e}")

    def _flush_buffer(self, buffer):
        if not buffer: return
        try:
            with self.path.open("a", encoding="utf-8") as f:
                f.write("\n".join(buffer) + "\n")
                f.flush()
                os.fsync(f.fileno())
        except Exception as e:
            log.error(f"❌ [AUDIT DISK ERROR] {e}")

    def close(self):
        self._stop_event.set()
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)

class StatisticalAnalyzer:
    """Online anomaly detection using Z-Score"""
    def __init__(self, window: int = 100):
        self.window = window
        self.history: Dict[str, deque] = {}

    def push(self, key: str, value: float):
        if key not in self.history:
            self.history[key] = deque(maxlen=self.window)
        self.history[key].append(value)

    def z_score(self, key: str, value: float) -> float:
        """Calculate Z-Score for value against history"""
        if key not in self.history or len(self.history[key]) < 10:
            return 0.0
        
        arr = np.array(self.history[key])
        mean = np.mean(arr)
        std = np.std(arr)
        
        if std < 1e-6: return 0.0
        return abs(value - mean) / std

_ANALYZER = StatisticalAnalyzer(window=100)

def check_signals(metrics: Dict[str, Any], grad_clip_ref: float = 1.0) -> List[str]:
    """Analyze metrics for anomalies (signals)"""
    flags = []
    clean_metrics = {}
    nan_found = False
    
    # 1. Pre-processing i NaN check
    for k, v in metrics.items():
        if isinstance(v, (int, float)):
            if not np.isfinite(v):
                nan_found = True
            else:
                clean_metrics[k] = v
                _ANALYZER.push(k, v)

    if metrics.get('nan_detected', 0) or nan_found:
        flags.append('UNSTABLE_NAN')
        return flags 

    curr_grad = clean_metrics.get('grad_norm', 0.0)
    if curr_grad > grad_clip_ref * 2.0:
        grad_z = _ANALYZER.z_score('grad_norm', curr_grad)
        if grad_z > 4.0: # Bardzo rzadkie zdarzenie (Sigma > 4)
            flags.append('EXPLODING_GRAD')
        
    # Martwy Agent (Zero Variance / Collapse)
    td_err = clean_metrics.get('loss', 1.0) # loss to zazwyczaj TD Error
    q_spread = clean_metrics.get('q_spread', 1.0) 
    
    if td_err < 1e-7 and q_spread < 1e-7:
        flags.append('DEAD_AGENT')

    entropy = clean_metrics.get('entropy', 0.5)
    if entropy < 0.01:
        flags.append('ENTROPY_COLLAPSE')
        
    return flags