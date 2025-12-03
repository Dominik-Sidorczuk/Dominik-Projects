import os
import sys
import logging
import argparse
import traceback
import torch
import torch.multiprocessing as mp

# Konfiguracja zmiennych środowiskowych dla maksymalnej wydajności (bez zmian)
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["NUMBA_NUM_THREADS"] = "1"
os.environ['CUDA_LAUNCH_BLOCKING'] = "1"
os.environ["TOKENIZERS_PARALLELISM"] = "false"

# Wstępna konfiguracja logowania przed jakimkolwiek importem logiki
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
log = logging.getLogger("AutoRL-Main")

try:
    mp.set_start_method('spawn', force=True)
    mp.set_sharing_strategy('file_system')
except RuntimeError:
    pass

sys.path.append(os.getcwd())

try:
    from src.config import AutoRLConfig
    from src.utils.toolkit import setup_logging, set_global_seed
    from src.run.runners import PopulationManager, run_single
except ImportError as e:
    # [LIBERATION]: Zastąpiono sys.stderr.write i traceback.print_exc()
    log.critical(f"❌ Critical Import Error: {e}")
    log.exception("Pełny zrzut błędu inicjalizacji:")
    sys.exit(1)

def run(cfg: AutoRLConfig):
    """Main entry point"""    
    # Przeładowanie logowania zgodnie z konfiguracją środowiska
    os.makedirs(cfg.out_dir, exist_ok=True)
    log_path = os.path.join(cfg.out_dir, "auto_rl.log")
    setup_logging(cfg.env.id, log_file=log_path)
    
    set_global_seed(cfg)
    
    # Użycie logera zdefiniowanego na poziomie modułu
    main_log = logging.getLogger(__name__)
    
    main_log.info(f"🚀 Starting AutoRL in mode: {cfg.mode.upper()} on {cfg.device}")
    main_log.info(f"📂 Output Directory: {os.path.abspath(cfg.out_dir)}")
    main_log.info(f"🔧 Multiprocessing Start Method: {mp.get_start_method()}")
    main_log.info(f"🌱 Global Seed set to: {cfg.seed}")
    
    if torch.cuda.is_available():
        main_log.info(f"✅ CUDA Available: {torch.cuda.get_device_name(0)}")
        mem_alloc = torch.cuda.memory_allocated(0) / 1024**2
        main_log.info(f"   Memory Allocated: {mem_alloc:.2f} MB")
        
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        
        if cfg.cuda_deterministic:
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
            main_log.info("🔒 CUDA Deterministic Mode: ON")
        else:
            torch.backends.cudnn.benchmark = cfg.cuda_benchmark
            torch.backends.cudnn.deterministic = False
            main_log.info(f"🚀 CUDA Benchmark: {cfg.cuda_benchmark}")
    else:
        main_log.warning("⚠️ CUDA not available! Training will be slow on CPU.")

    if cfg.mode == "population":
        main_log.info(f"👥 PBT Manager (EVI-Aware) Start | Size: {cfg.pbt.population_size}")
        
        manager = PopulationManager(cfg)
        try:
            manager.run()
            manager.stop()
        except KeyboardInterrupt:
            main_log.info("🛑 PBT Manager stopped by user.")
            manager.stop()
        except Exception as e:
            # [LIBERATION]: Pełna obsługa wyjątku przez loger
            main_log.exception(f"💀 PBT Manager crashed: {e}")
            manager.stop()
            sys.exit(1)

    elif cfg.mode == "train":
        try:
            run_single(cfg)
        except KeyboardInterrupt:
            main_log.info("🛑 Training stopped by user.")
        except Exception as e:
            # [LIBERATION]: Pełna obsługa wyjątku przez loger
            main_log.exception(f"💀 Single Run Crashed: {e}")
            sys.exit(1)
            
    else:
        main_log.error(f"Unknown mode: {cfg.mode}")
        sys.exit(1)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="AutoRL Runner")
    parser.add_argument("--mode", type=str, default=None, help="Mode: train or population")
    parser.add_argument("--train.total_steps", dest="total_steps", type=int, help="Total training steps")
    parser.add_argument("--pbt.population_size", dest="pop_size", type=int, help="Population size")
    parser.add_argument("--seed", type=int, default=None, help="Override seed")
    
    args = parser.parse_args()

    # Bezpieczne ładowanie konfiguracji
    try:
        cfg = AutoRLConfig()

        if args.mode:
            cfg.mode = args.mode
            
        if args.total_steps:
            cfg.train.total_steps = args.total_steps
            
        if args.pop_size:
            cfg.pbt.population_size = args.pop_size
            
        if args.seed is not None:
            cfg.seed = args.seed

        run(cfg)
    except Exception as e:
        log.critical("FATAL: Configuration or Runtime Error.")
        log.exception(e)
        sys.exit(1)