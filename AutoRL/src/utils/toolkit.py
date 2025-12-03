from __future__ import annotations

import os
import sys
import csv
import time
import json
import random
import platform
import logging
import dataclasses
from datetime import datetime, timezone
from typing import Any, Union, Dict, List, Optional
from dataclasses import dataclass

import numpy as np
import torch
from pydantic import BaseModel
from rich.logging import RichHandler
from rich.console import Console

def set_global_seed(seed_or_config, cuda_deterministic: bool = False, cuda_benchmark: bool = True) -> None:
    """Set random seeds for Python, NumPy and PyTorch"""
    from src.config import AutoRLConfig
    
    if isinstance(seed_or_config, AutoRLConfig):
        seed = int(seed_or_config.seed)
        deterministic = seed_or_config.cuda_deterministic
        benchmark = seed_or_config.cuda_benchmark
    elif isinstance(seed_or_config, int):
        seed = int(seed_or_config)
        deterministic = cuda_deterministic
        benchmark = cuda_benchmark
    else:
        seed = int(seed_or_config)
        deterministic = cuda_deterministic
        benchmark = cuda_benchmark

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
        torch.backends.cudnn.deterministic = deterministic
        torch.backends.cudnn.benchmark = benchmark

def _convert_to_serializable(obj: Any) -> Any:
    """Convert objects to JSON-friendly format"""
    if dataclasses.is_dataclass(obj):
        return dataclasses.asdict(obj)
    if isinstance(obj, BaseModel):
        return obj.model_dump()
    if isinstance(obj, (list, tuple)):
        return [_convert_to_serializable(x) for x in obj]
    if isinstance(obj, set):
        return list(obj)
    if isinstance(obj, torch.device):
        return str(obj)
    if hasattr(obj, 'item'): 
        return obj.item()
    if hasattr(obj, 'dtype'): 
        return obj.tolist()
    return obj

class EnhancedJSONEncoder(json.JSONEncoder):
    """Encoder JSON radzący sobie z nietypowymi obiektami ML."""
    def default(self, o):
        try:
            return _convert_to_serializable(o)
        except:
            return super().default(o)

def save_manifest(path: str, cfg: Union[Dict, Any], extra: Dict | None = None) -> None:
    """
    Zapisuje manifest eksperymentu (metadane + config).
    """
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        serializable_cfg = _convert_to_serializable(cfg)

        manifest = {
            'meta': {
                'timestamp': datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                'pid': os.getpid(),
                'cwd': os.getcwd(),
            },
            'platform': {
                'python': platform.python_version(),
                'system': platform.system(),
                'node': platform.node(),
            },
            'env': {
                'torch': torch.__version__,
                'cuda_available': torch.cuda.is_available(),
                'device': torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"
            },
            'config': serializable_cfg,
        }
        
        if extra:
            manifest.update(extra)
            
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(manifest, f, indent=2, cls=EnhancedJSONEncoder)
            
    except Exception as e:
        sys.stderr.write(f"⚠️ Failed to save manifest at {path}: {e}\n")

# --- [FIX] CUSTOM LOGGING HANDLER ---

class ForceFlushStreamHandler(logging.StreamHandler):
    """
    Handler, który wymusza flush po każdym wpisie.
    Jest to kluczowe w trybie multiprocessing 'spawn', gdzie standardowy stdout
    jest buforowany i logi pojawiają się z ogromnym opóźnieniem lub wcale.
    """
    def emit(self, record):
        try:
            super().emit(record)
            self.flush()
        except Exception:
            self.handleError(record)

# ------------------------------------

_LOGGING_CONFIGURED = False

def setup_logging(service_name: str = "AutoRL", skip_logfire: bool = False, 
                  simple_mode: bool = False, log_file: str = None) -> logging.Logger:
    """
    Configure logging system (Rich + Logfire + File).
    
    ZMIANY:
    - Użycie ForceFlushStreamHandler dla workerów.
    - Obejście problemów z przekierowaniem stdout.
    """
    global _LOGGING_CONFIGURED
    
    # Zawsze rekonfiguruj dla workerów (simple_mode), aby mieć pewność, że handler jest podpięty
    if _LOGGING_CONFIGURED and not simple_mode and not log_file:
        return logging.getLogger(service_name)

    handlers = []

    # A. Simple Mode (Dla Workerów PBT)
    if simple_mode:
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)
        
        # Czyścimy stare handlery, aby uniknąć dublowania
        if root_logger.hasHandlers():
            root_logger.handlers.clear()
        
        # [FIX] Używamy surowego stdout, jeśli dostępny, by pominąć wrappery bibliotek
        try:
            stream = sys.__stdout__ if sys.__stdout__ else sys.stdout
        except AttributeError:
            stream = sys.stdout

        # [FIX] Podpinamy nasz wymuszający flush handler
        ch = ForceFlushStreamHandler(stream)
        ch.setLevel(logging.INFO)
        ch.setFormatter(logging.Formatter(
            f"%(asctime)s | {service_name} | %(name)s | %(levelname)s | %(message)s",
            datefmt="%H:%M:%S"
        ))
        root_logger.addHandler(ch)
        
        if log_file:
            try:
                os.makedirs(os.path.dirname(log_file), exist_ok=True)
                fh = logging.FileHandler(log_file, mode='a', encoding='utf-8')
                fh.setLevel(logging.DEBUG) 
                fh.setFormatter(logging.Formatter(
                    "%(asctime)s | %(name)s | %(levelname)s | %(message)s"
                ))
                root_logger.addHandler(fh)
            except Exception as e:
                sys.stderr.write(f"⚠️ Failed to setup log file {log_file}: {e}\n")
        
        _LOGGING_CONFIGURED = True
        return logging.getLogger(service_name)
        
    # B. Rich Mode (Dla Głównego Managera)
    else:
        logger = logging.getLogger(service_name)
        logger.setLevel(logging.INFO)
        
        if logger.hasHandlers():
            logger.handlers.clear()
        
        root_logger = logging.getLogger()
        if root_logger.hasHandlers():
            root_logger.handlers.clear()
        
        console = Console(width=160, stderr=False, force_terminal=True)
        rich_handler = RichHandler(
            console=console, 
            rich_tracebacks=True,
            markup=True,
            show_path=False,
            enable_link_path=True
        )
        handlers.append(rich_handler)
        
        if not skip_logfire:
            try:
                import logfire
                if os.getenv("LOGFIRE_TOKEN") or os.getenv("LOGFIRE_WRITE_TOKEN"):
                    logfire.configure(service_name=service_name)
                    logfire.instrument_pydantic()
                else:
                    logfire.configure(service_name=service_name, send_to_logfire=False)
            except ImportError:
                pass
            except Exception:
                pass

        if log_file:
            try:
                os.makedirs(os.path.dirname(log_file), exist_ok=True)
                fh = logging.FileHandler(log_file, mode='a', encoding='utf-8')
                fh.setFormatter(logging.Formatter("%(asctime)s | %(name)s | %(levelname)s | %(message)s"))
                handlers.append(fh)
            except Exception as e:
                sys.stderr.write(f"⚠️ Failed to setup log file {log_file}: {e}\n")

        logging.basicConfig(level="INFO", handlers=handlers, force=True)
        logger.propagate = False
        
        _LOGGING_CONFIGURED = True
        return logger

class CSVLogger:
    """Bulletproof CSV logger for numerical data"""
    def __init__(self, path: str):
        self.path = path
        self.file = None
        self.writer = None
        self.log_sys = logging.getLogger("CSVLogger")
        
        retries = 5
        attempt = 0
        while attempt < retries:
            try:
                os.makedirs(os.path.dirname(self.path), exist_ok=True)
                # buffering=1 -> Line Buffering (zapis na dysk co linię)
                self.file = open(self.path, "w", newline="", encoding="utf-8", buffering=1)
                self.writer = csv.writer(self.file)
                return
            except OSError as e:
                attempt += 1
                self.log_sys.warning(f"⚠️ [CSV ERROR] Attempt {attempt}/{retries} for {self.path}: {e}")
                time.sleep(0.5)
            except Exception as e:
                self.log_sys.error(f"❌ [CSV FATAL] Failed to init {self.path}: {e}")
                self.file = None
                break

    def log(self, data: Dict[str, Any] | List[Any]):
        """Universal write method supporting dict or list"""
        if not self.file: return

        row = list(data.values()) if isinstance(data, dict) else data
        
        try:
            self.writer.writerow(row)
        except ValueError:
            pass 
        except Exception as e:
            self.log_sys.error(f"⚠️ [CSV WRITE ERROR] {e}")

    def close(self):
        if self.file:
            try:
                self.file.flush()
                os.fsync(self.file.fileno())
                self.file.close()
            except: pass
            self.file = None