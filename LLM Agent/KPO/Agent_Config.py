import torch
from typing import Dict, List, Optional, Tuple, Any, Type
from pydantic import BaseModel, Field as PydanticField, HttpUrl, ConfigDict

# Importy z pliku z promptami - upewniono się, że wszystko jest obecne
from Adaptive_Agent_Prompts import (
    SearchQueries,
    AnalysisReport,
    StrategicPlan,
    ExtractedFactList,
    CuriosityHypothesis,
    SimpleSummary,
    CoTStep1Claims,
    CoTStep2Evidence,
    CoTStep3Inconsistencies,
    CoTStep4RedFlags,
    Decision # Dodano brakujący import
)

# Stałe
DEFAULT_LOG_LEVEL = "INFO"
DEFAULT_TORCH_DTYPE = torch.bfloat16
DEFAULT_DEVICE_MAP = "cuda"
MIN_CONTENT_LENGTH = 60

class AgentLogicConfig(BaseModel):
    """Konfiguracja logiki agentów i parametrów przetwarzania."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    
    max_analysis_cycles: int = PydanticField(default=3, ge=1)
    deep_fetch_depth: int = PydanticField(default=1, ge=1)
    top_k_serp: int = PydanticField(default=9, ge=1)
    max_subpages_per_host: int = PydanticField(default=3, ge=1)
    max_analysis_attempts: int = PydanticField(default=2, ge=1)
    max_search_queries: int = PydanticField(default=22, ge=1)
    max_crawl_links: int = PydanticField(default=22, ge=1)
    max_docs_in_memory: int = PydanticField(default=1500, ge=100)
    summary_batch_size: int = PydanticField(default=24, ge=1)
    max_instructor_questions: int = PydanticField(default=2, ge=0)
    instructor_refinement_iterations: int = PydanticField(default=2, ge=0)

class SystemConfig(BaseModel):
    """Konfiguracja systemowa i ustawienia zasobów."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    max_workers: int = PydanticField(default=12, ge=1)
    log_level: str = PydanticField(default=DEFAULT_LOG_LEVEL)
    model_cache_size: int = PydanticField(default=2, ge=1, description="Liczba modeli trzymanych w pamięci VRAM (bufor LRU).")

class ModelConfig(BaseModel):
    """Konfiguracja modeli GGUF dla llama.cpp."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid', arbitrary_types_allowed=True)
    
    base_model_gguf_path: str = PydanticField(
        default="/run/media/dominik/F2B452FDB452C42F/LLM models/Qwen3-Esper3-Reasoning-Instruct-6B-Brainstorm20x-Enhanced-E32-192k-ctx.i1-Q4_K_M.gguf"
    )
    finance_model_gguf_path: str = PydanticField(
        default="/run/media/dominik/F2B452FDB452C42F/LLM models/qwen3-4B-Claude-Sonnet-4-Reasoning-Distill_Q4_K_M.gguf"
    )
    embedding_model_gguf_path: str = PydanticField(
        default="/run/media/dominik/F2B452FDB452C42F/LLM models/nomic-embed-text-v1.5-Q4_K_M.gguf"
    )
    
    max_new_tokens: int = PydanticField(default=4096, ge=1)
    embedding_context_size: int = PydanticField(default=8192, ge=1)
    context_size: int = PydanticField(default=8192, ge=512)
    # torch_dtype i device_map są specyficzne dla transformers, a nie llama.cpp,
    # ale zostawiam je, jeśli masz inne plany ich użycia.
    torch_dtype: Any = PydanticField(default=DEFAULT_TORCH_DTYPE)
    device_map: str = PydanticField(default=DEFAULT_DEVICE_MAP)
    
    agent_configs: Dict[str, Dict[str, Any]] = PydanticField(
        default_factory=lambda: {
            "decompose": {"schema": StrategicPlan, "model_type": "base"},
            "hunter": {"schema": SearchQueries, "model_type": "base"},
            "summarizer": {"schema": SimpleSummary, "model_type": "base"},
            "fact_extractor": {"schema": ExtractedFactList, "model_type": "base"},
            "curiosity": {"schema": CuriosityHypothesis, "model_type": "finance"},
            "cot_step1_claims": {"schema": CoTStep1Claims, "model_type": "finance"},
            "cot_step2_evidence": {"schema": CoTStep2Evidence, "model_type": "finance"},
            "cot_step3_inconsistencies": {"schema": CoTStep3Inconsistencies, "model_type": "finance"},
            "cot_step4_redflags": {"schema": CoTStep4RedFlags, "model_type": "finance"},
            "decider": {"schema": Decision, "model_type": "finance"},
            "report_compiler": {"schema": AnalysisReport, "model_type": "base"},
        }
    )

class FileConfig(BaseModel):
    """Konfiguracja ścieżek plików wejściowych i wyjściowych."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    input_csv: str = PydanticField(default="oczyszczone_dane.csv")
    output_csv: str = PydanticField(default="firmy_weryfikacja_final.csv")

class NetworkConfig(BaseModel):
    """Konfiguracja sieciowa, scrapowania i żądań HTTP."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    sleep_between_requests: Tuple[float, float] = PydanticField(default=(0.2, 4.2))
    worker_browser: str = PydanticField(default="chromium")
    max_links_per_search: int = PydanticField(default=8, ge=1)
    concurrency_limit: int = PydanticField(default=4, ge=1)
    request_timeout: int = PydanticField(default=24, ge=1)
    httpx_timeout: float = PydanticField(default=6.0, ge=1.0)
    cloudscraper_timeout: float = PydanticField(default=14.0, ge=1.0)
    playwright_timeout: float = PydanticField(default=42.0, ge=1.0)
    backoff_factor: int = PydanticField(default=5, ge=1)
    request_retries: int = PydanticField(default=2, ge=0)
    min_content_length: int = PydanticField(default=MIN_CONTENT_LENGTH, ge=0)
    user_agents: List[str] = PydanticField(
        default_factory=lambda: [
            "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/125.0.0.0 Safari/537.36",
            "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/125.0.0.0 Safari/537.36",
        ]
    )
    onion_url: HttpUrl = PydanticField(default="http://duckduckgogg42xjoc72x3sjasowoarfbgcmvfimaftt6twagswzczad.onion/")

class TorConfig(BaseModel):
    """Konfiguracja połączenia z siecią Tor."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    use_tor: bool = PydanticField(default=True)
    tor_socks_proxy: str = PydanticField(default="socks5://192.168.1.205:9150")
    tor_control_port: int = PydanticField(default=9151, ge=1, le=65535)
    tor_control_host: str = PydanticField(default="192.168.1.205")
    tor_control_password: str = PydanticField(default="123456789a")
    newnym_interval: int = PydanticField(default=3, ge=1)

class AppConfig(BaseModel):
    """Główna konfiguracja aplikacji łącząca wszystkie podkonfiguracje."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    models: ModelConfig = ModelConfig()
    files: FileConfig = FileConfig()
    network: NetworkConfig = NetworkConfig()
    tor: TorConfig = TorConfig()
    agent: AgentLogicConfig = AgentLogicConfig()
    system: SystemConfig = SystemConfig()

# --- KLUCZOWA POPRAWKA ---
# Ta linia tworzy instancję konfiguracji, której brakowało,
# co powodowało błąd `ImportError`.
config = AppConfig()

