import asyncio
import logging
import traceback
import gc
import os
from typing import Dict, Any, List, Optional, Tuple
from multiprocessing import Process, Queue
from queue import Empty
from pathlib import Path
from collections import OrderedDict

import numpy as np
from pydantic import BaseModel
from llama_cpp import Llama, LlamaGrammar, LLAMA_POOLING_TYPE_CLS, LLAMA_SPLIT_MODE_NONE

from Agent_Config import AppConfig
from Local_Client import EmbeddingAwareLocalAgentClient

logger = logging.getLogger("multi_model_handler")
model_server_logger = logging.getLogger("model_server")


# --- SERWER MODELI Z BUFOROWANIEM LRU ---
class ModelServer:
    """
    Zarządza cyklem życia modeli Llama w dedykowanym procesie.
    """
    def __init__(self, request_q: Queue, response_q: Queue, config: AppConfig):
        self.request_q = request_q
        self.response_q = response_q
        self.config = config
        self.model_configs: Dict[str, Dict[str, Any]] = self._build_model_configs()
        self.model_cache: OrderedDict[str, Llama] = OrderedDict()
        self.max_cache_size = config.system.model_cache_size

    def _build_model_configs(self) -> Dict[str, Dict[str, Any]]:
        """Dynamicznie buduje konfigurację modeli na podstawie AppConfig."""
        configs = {}
        configs["embedding"] = {
            "path": self.config.models.embedding_model_gguf_path,
            "is_embedding": True
        }
        agent_model_types = {agent['model_type'] for agent in self.config.agents.configs.values()}

        for model_type in agent_model_types:
            path_key = f"{model_type}_model_gguf_path"
            if not hasattr(self.config.models, path_key):
                raise ValueError(f"Brak ścieżki '{path_key}' w ModelConfig dla typu modelu '{model_type}' zdefiniowanego w AgentsConfig.")
            
            configs[model_type] = {
                "path": getattr(self.config.models, path_key),
                "is_embedding": False
            }
        
        model_server_logger.info(f"Zbudowano dynamiczną konfigurację dla modeli: {list(configs.keys())}")
        return configs

    def _evict_lru_model(self):
        if len(self.model_cache) >= self.max_cache_size:
            lru_model_name, lru_model = self.model_cache.popitem(last=False)
            model_server_logger.info(f"Bufor pełny. Zwalniam najdawniej używany model: '{lru_model_name}'...")
            del lru_model
            gc.collect()

    def _switch_model(self, required_model_name: str) -> Llama:
        if required_model_name in self.model_cache:
            self.model_cache.move_to_end(required_model_name)
            return self.model_cache[required_model_name]

        self._evict_lru_model()
        model_config = self.model_configs.get(required_model_name)
        if not model_config:
            raise ValueError(f"Brak konfiguracji dla modelu: {required_model_name}")

        path = model_config["path"]
        if not path or not Path(path).is_file():
            raise FileNotFoundError(f"Plik modelu '{required_model_name}' nie znaleziony w: {path}")

        model_server_logger.info(f"Ładowanie nowego modelu: '{required_model_name}'...")
        params = {"model_path": path, "n_gpu_layers": -1, "verbose": False, "split_mode": LLAMA_SPLIT_MODE_NONE}
        
        if model_config["is_embedding"]:
            params.update({"n_ctx": self.config.models.embedding_context_size, "embedding": True, "pooling_type": LLAMA_POOLING_TYPE_CLS})
        else:
            params.update({"n_ctx": self.config.models.context_size, "n_threads": self.config.system.max_workers, "main_gpu": 0})

        new_model = Llama(**params)
        self.model_cache[required_model_name] = new_model
        model_server_logger.info(f"✅ Model '{required_model_name}' załadowany. Rozmiar bufora: {len(self.model_cache)}/{self.max_cache_size}.")
        return new_model

    def run(self):
        model_server_logger.info(f"Serwer modeli uruchomiony. PID: {os.getpid()}.")
        try:
            self.response_q.put({"status": "ok", "message": "Serwer modeli jest gotowy."})
            while True:
                request = self.request_q.get()
                if request is None: break
                try:
                    req_type = request.get("type")
                    if req_type == "embed":
                        model = self._switch_model("embedding")
                        embedding = model.embed(request["text"])
                        self.response_q.put({"status": "ok", "data": embedding})
                    elif req_type == "generate":
                        model = self._switch_model(request["model_name"])
                        gen_params = request["params"]
                        if "grammar" in gen_params and isinstance(gen_params["grammar"], str):
                            gen_params["grammar"] = LlamaGrammar.from_string(gen_params["grammar"])
                        result = model(**gen_params)
                        safe_result = {"choices": [{"text": c.get("text")} for c in result.get("choices", [])]}
                        self.response_q.put({"status": "ok", "data": safe_result})
                    elif req_type == "health_check":
                        self.response_q.put({"status": "ok", "message": "Server is alive"})
                    else:
                        raise ValueError(f"Nieznany typ żądania: {req_type}")
                except Exception as e:
                    tb_str = traceback.format_exc()
                    model_server_logger.error(f"Błąd podczas przetwarzania żądania: {e}\n{tb_str}")
                    self.response_q.put({"status": "error", "message": str(e), "traceback": tb_str})
        finally:
            self.model_cache.clear(); gc.collect()
            model_server_logger.info("Serwer modeli został zamknięty.")

def model_server_process(request_q: Queue, response_q: Queue, config: AppConfig):
    server = ModelServer(request_q, response_q, config)
    server.run()


# --- OBIEKT PROXY ---
class InterProcessLlama(Llama):
    def __new__(cls, *args, **kwargs): return object.__new__(cls)
    def __init__(self, handler: 'MultiModelHandler', model_name: str):
        self.handler = handler; self.model_name = model_name
    
    def __call__(self, **kwargs: Any) -> Dict[str, Any]:
        if "grammar" in kwargs and isinstance(kwargs.get("grammar"), LlamaGrammar):
            kwargs["grammar"] = kwargs["grammar"]._grammar
        return self.handler.send_generation_request(self.model_name, kwargs)

    def embed(self, text: str) -> List[float]:
        return self.handler.get_embedding_sync(text)


# --- KLASA PAMIĘCI WEKTOROWEJ ---
MemoryEntry = Tuple[str, str, np.ndarray]

class VectorMemory:
    def __init__(self):
        self.entries: List[MemoryEntry] = []
    def add_entry(self, speaker: str, text: str, embedding: np.ndarray):
        self.entries.append((speaker, text, embedding))
    def search_similar(self, query_embedding: np.ndarray, top_k: int = 2, threshold: float = 0.65) -> List[MemoryEntry]:
        if not self.entries: return []
        # Implementacja logiki wyszukiwania ...
        return []


# --- GŁÓWNY HANDLER (KLIENT) ---
class MultiModelHandler:
    def __init__(self, app_config: AppConfig, request_q: Queue, response_q: Queue, server_process: Process):
        self.config = app_config
        self.clients: Dict[str, EmbeddingAwareLocalAgentClient] = {}
        self.agent_capability_embeddings: Dict[str, np.ndarray] = {}
        self.request_q = request_q
        self.response_q = response_q
        self.server_process = server_process
        self.request_lock = asyncio.Lock()
        self.main_loop: Optional[asyncio.AbstractEventLoop] = None
        self.vector_memory = VectorMemory()

    async def initialize(self):
        try: self.main_loop = asyncio.get_running_loop()
        except RuntimeError: raise RuntimeError("Handler musi być zainicjowany w pętli asyncio.")
        
        logger.info("Oczekiwanie na gotowość serwera modeli...")
        try:
            ready_signal = await self.main_loop.run_in_executor(None, lambda: self.response_q.get(timeout=180))
            if ready_signal.get("status") != "ok": raise RuntimeError(f"Błąd serwera modeli: {ready_signal.get('message')}")
            
            # NOWOŚĆ: Wykonaj health check
            health = await self._send_request({"type": "health_check"})
            if not health: raise RuntimeError("Serwer modeli nie odpowiedział na health check.")
            logger.info("✅ Serwer modeli jest gotowy i odpowiada.")

        except Empty: raise TimeoutError("Serwer modeli nie uruchomił się w wyznaczonym czasie.")
        
        self._initialize_agent_clients()
        await self._create_capability_embeddings()
        logger.info("✅ MultiModelHandler (klient) zainicjowany pomyślnie.")

    def _initialize_agent_clients(self):
        logger.info("Inicjalizuję klientów agentów...")
        for name, cfg in self.config.agents.configs.items():
            model_type = cfg.get("model_type", "base")
            self.clients[name] = EmbeddingAwareLocalAgentClient(
                handler=self,
                llama_model=InterProcessLlama(self, model_type),
                agent_name=name,
                schema=cfg.get("schema"),
                tool_name="", # Uproszczono
                config=self.config,
                model_lock=asyncio.Lock()
            )

    async def _send_request(self, request: Dict[str, Any]) -> Any:
        async with self.request_lock:
            if not self.server_process.is_alive(): raise RuntimeError("Proces serwera modeli został zakończony.")
            
            logger.debug(f"Wysyłanie żądania do serwera: {request.get('type')}")
            await self.main_loop.run_in_executor(None, self.request_q.put, request)
            
            logger.debug("Oczekiwanie na odpowiedź od serwera...")
            response = await self.main_loop.run_in_executor(None, lambda: self.response_q.get(timeout=600))
            
            if response.get("status") != "ok":
                raise RuntimeError(f"Błąd serwera modeli: {response.get('message')}")
            logger.debug("Otrzymano pomyślną odpowiedź.")
            return response.get("data")

    async def get_embedding(self, text: str) -> np.ndarray:
        embedding_list = await self._send_request({"type": "embed", "text": text})
        return np.array(embedding_list, dtype=np.float32)

    def get_embedding_sync(self, text: str) -> List[float]:
        future = asyncio.run_coroutine_threadsafe(self.get_embedding(text), self.main_loop)
        return future.result().tolist()

    def send_generation_request(self, model_name: str, params: Dict[str, Any]) -> Dict[str, Any]:
        request = {"type": "generate", "model_name": model_name, "params": params}
        future = asyncio.run_coroutine_threadsafe(self._send_request(request), self.main_loop)
        return future.result()

    async def _create_capability_embeddings(self):
        """Tworzy embeddingi opisów zdolności agentów do celów routingu."""
        pass
    
    async def invoke_agent(self, agent_name: str, messages: List[Dict[str, str]]) -> Tuple[Optional[BaseModel], str]:
        client = self.clients.get(agent_name)
        if not client: return None, f"Brak klienta dla agenta '{agent_name}'."
        return await client.create(messages, self.config.agent.max_analysis_attempts)

    def shutdown(self):
        logger.info("Zamykanie handlera i serwera modeli...")
        if self.server_process and self.server_process.is_alive():
            try:
                self.request_q.put(None)
                self.server_process.join(timeout=15)
            except Exception as e: logger.error(f"Błąd podczas zamykania: {e}")
            if self.server_process.is_alive():
                logger.warning("Serwer nie zamknął się poprawnie. Wymuszam zakończenie.")
                self.server_process.terminate()
