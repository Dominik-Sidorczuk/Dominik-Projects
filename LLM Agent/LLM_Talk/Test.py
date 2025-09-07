"""
Ulepszona wersja skryptu do symulacji rozmowy między modelami LLM.

Główne zmiany w wersji 8.0 (Lepsze Prompty i Parser):
1.  **Inteligentny Parser Odpowiedzi (`_parse_and_validate_response`)**:
    Zastępuje prostą funkcję czyszczącą. Potrafi teraz aktywnie usuwać
    całe bloki "myśli" (np. <think>...</think>) i wyodrębniać właściwą
    odpowiedź z potencjalnie zaszumionego wyjścia modelu.
2.  **Nowa Technika Promptowania ("Role-playing with examples")**:
    Prompt został całkowicie przebudowany. Zamiast listy suchych instrukcji,
    teraz jasno definiuje rolę agenta i, co najważniejsze, zawiera krótki
    przykład idealnej interakcji (tzw. few-shot prompting). Jest to
    znacznie skuteczniejsza metoda instruowania modeli.
3.  **Zwiększona Odporność**: Połączenie lepszego promptu na wejściu
    i inteligentniejszego parsera na wyjściu powinno radykalnie zwiększyć
    odporność symulacji na "lekkie" błędy modeli i zapewnić płynniejszy
    przebieg rozmowy.
"""
import logging
import asyncio
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass, field
import sys
import os
import numpy as np
import re
import time
import nest_asyncio

# --- Sprawdzenie dostępności llama_cpp i definicje klas pozornych ---
try:
    from llama_cpp import Llama, LlamaCache, LLAMA_SPLIT_MODE_NONE
    LLAMA_CPP_AVAILABLE = True
except ImportError:
    logging.warning("Biblioteka 'llama_cpp' nie znaleziona. Używam obiektów pozornych (mock).")
    LLAMA_CPP_AVAILABLE = False
    
    class Llama:
        def __init__(self, model_path: str = "mock_path", **kwargs):
            self._model_name = Path(model_path).name
            logging.info(f"[MOCK] Inicjalizacja Llama dla modelu: {self._model_name}")
        def create_completion(self, prompt: str, **kwargs) -> Dict[str, Any]:
            response_text = f"<think>Model {self._model_name} myśli...</think> A oto dynamiczna, pozorowana odpowiedź po polsku."
            return {'choices': [{'text': response_text}]}
        def embed(self, text: str) -> List[float]:
            logging.info(f"[MOCK] Tworzenie osadzeń dla tekstu: '{text[:40]}...'")
            np.random.seed(sum(ord(c) for c in text))
            return np.random.randn(128).tolist()

    class LlamaCache: pass
    LLAMA_SPLIT_MODE_NONE = 0

# --- Aliasy i Konfiguracja ---
ConversationHistory = List[Tuple[str, str]]
MemoryEntry = Tuple[str, str, List[float]]

@dataclass
class LlamaParams:
    n_gpu_layers: int = -1; main_gpu: int = 0; n_ctx: int = 4096
    n_threads: int = 8; embedding: bool = False; verbose: bool = False

@dataclass
class ModelConfig:
    path: str; is_embedding: bool = False

@dataclass
class ModelsConfig:
    """Konfiguracja wszystkich modeli."""
    models: Dict[str, ModelConfig] = field(default_factory=lambda: {
        "base": ModelConfig(path=os.getenv("BASE_MODEL_PATH", "/run/media/dominik/F2B452FDB452C42F/LLM models/Qwen3-Esper3-Reasoning-Instruct-6B-Brainstorm20x-Enhanced-E32-192k-ctx.i1-Q4_K_M.gguf")),
        "finance_instruct": ModelConfig(path=os.getenv("FINANCE_MODEL_PATH", "/run/media/dominik/F2B452FDB452C42F/LLM models/qwen3-4B-Claude-Sonnet-4-Reasoning-Distill_Q4_K_M.gguf")),
        "embedding": ModelConfig(path=os.getenv("EMBEDDING_MODEL_PATH", "/run/media/dominik/F2B452FDB452C42F/LLM models/nomic-embed-text-v1.5-Q4_K_M.gguf"), is_embedding=True)
    })

@dataclass
class AppConfig:
    models: ModelsConfig = field(default_factory=ModelsConfig)
    llama_params: LlamaParams = field(default_factory=LlamaParams)
    max_tokens: int = 256
    embedding_cache_size: int = 100
    memory_retrieval_threshold: float = 0.65
    repeat_penalty: float = 1.15

logging.basicConfig(level=logging.INFO, format="[%(asctime)s][%(name)s][%(levelname)s] %(message)s", force=True)
logger = logging.getLogger(__name__)

def cosine_similarity(v1: List[float], v2: List[float]) -> float:
    vec1, vec2 = np.array(v1), np.array(v2)
    dot = np.dot(vec1, vec2); norm = np.linalg.norm(vec1) * np.linalg.norm(vec2)
    return dot / norm if norm > 0 else 0.0

def preprocess_text(text: str) -> str:
    return text.strip().replace("\n", " ")[:512]

# --- Zarządzanie Modelami ---
class ModelLoader:
    def __init__(self, config: AppConfig):
        self.config = config; self.models: Dict[str, Llama] = {}
        self.locks: Dict[str, asyncio.Lock] = {}; self.embedding_cache: Dict[str, List[float]] = {}

    def _setup_single_model(self, model_name: str, model_config: ModelConfig) -> Optional[Llama]:
        path = Path(model_config.path)
        if not model_config.path or (LLAMA_CPP_AVAILABLE and not path.is_file()):
            logger.warning(f"Plik modelu '{model_name}' nie istnieje: {model_config.path}. Używam mocka.")
            return Llama(model_path=model_config.path, embedding=model_config.is_embedding)
        try:
            params = self.config.llama_params
            llm = Llama(model_path=model_config.path, n_gpu_layers=params.n_gpu_layers,
                        main_gpu=params.main_gpu, n_ctx=params.n_ctx, n_threads=params.n_threads,
                        embedding=model_config.is_embedding, split_mode=LLAMA_SPLIT_MODE_NONE,
                        cache=LlamaCache() if LLAMA_CPP_AVAILABLE else None, verbose=params.verbose)
            logger.info(f"Model '{model_name}' wczytany poprawnie.")
            return llm
        except Exception as e:
            logger.error(f"Błąd ładowania modelu '{model_name}': {e}"); return None

    async def load_all_models(self) -> bool:
        load_tasks = {name: asyncio.to_thread(self._setup_single_model, name, mc) for name, mc in self.config.models.models.items()}
        results = await asyncio.gather(*load_tasks.values())
        all_loaded = True
        for (name, _), model in zip(load_tasks.items(), results):
            if model: self.models[name], self.locks[name] = model, asyncio.Lock()
            else: all_loaded = False
        return all_loaded

    def unload_all_models(self):
        self.models.clear(); self.locks.clear(); self.embedding_cache.clear()

    def get_model_and_lock(self, name: str) -> Optional[Tuple[Llama, asyncio.Lock]]:
        return self.models.get(name), self.locks.get(name)

# --- Pamięć Wektorowa ---
class VectorMemory:
    def __init__(self):
        self.entries: List[MemoryEntry] = []

    def add_entry(self, speaker: str, text: str, embedding: List[float]):
        self.entries.append((speaker, text, embedding))
        logger.info(f"[Pamięć] Dodano wpis od '{speaker}'. Całkowita liczba wpisów: {len(self.entries)}.")

    def search_similar(self, query_embedding: List[float], top_k: int = 2, threshold: float = 0.65) -> List[MemoryEntry]:
        if not self.entries: return []
        similarities = [cosine_similarity(query_embedding, entry[2]) for entry in self.entries]
        sorted_indices = np.argsort(similarities)[::-1]
        results = []
        for i in sorted_indices:
            if len(results) >= top_k: break
            # Szukaj we wszystkich wpisach oprócz ostatniego (który jest samym zapytaniem)
            if similarities[i] >= threshold and i < len(self.entries) - 1:
                results.append(self.entries[i])
        return results

# --- Agent Konwersacyjny ---
class ConversationalAgent:
    def __init__(self, name: str, model_loader: ModelLoader, completion_model: str, memory: VectorMemory):
        self.name = name; self.model_loader = model_loader
        self.completion_model = completion_model; self.memory = memory

    async def _get_embedding(self, text: str) -> Optional[List[float]]:
        model, lock = self.model_loader.get_model_and_lock("embedding")
        if not model: return None
        preprocessed = preprocess_text(text)
        if preprocessed in self.model_loader.embedding_cache:
            return self.model_loader.embedding_cache[preprocessed]
        try:
            async with lock:
                embedding = await asyncio.to_thread(model.embed, preprocessed)
                if len(self.model_loader.embedding_cache) < self.model_loader.config.embedding_cache_size:
                    self.model_loader.embedding_cache[preprocessed] = embedding
                return embedding
        except Exception as e:
            logger.error(f"Błąd tworzenia osadzenia dla '{text[:30]}...': {e}"); return None

    def _parse_and_validate_response(self, text: str) -> Optional[str]:
        """Inteligentnie parsuje i waliduje odpowiedź modelu."""
        # 1. Usuń bloki <think>
        cleaned_text = re.sub(r'<think>.*?</think>', '', text, flags=re.DOTALL).strip()
        
        # 2. Podziel na linie i weź ostatnią niepustą
        lines = [line.strip() for line in cleaned_text.split('\n') if line.strip()]
        if not lines:
            logger.warning("Odpowiedź odrzucona (pusta po parsowaniu).")
            return None
        
        final_response = lines[-1]
        
        # 3. Dodatkowe czyszczenie artefaktów
        final_response = re.sub(r'^\s*(Użytkownik|Agent Ogólny|Agent Finansowy):\s*', '', final_response, flags=re.IGNORECASE).strip()
        
        # 4. Walidacja
        if len(final_response) < 15:
            logger.warning(f"Odpowiedź odrzucona (za krótka po parsowaniu): '{final_response}'")
            return None
        
        return final_response

    async def _generate_response(self, history: ConversationHistory, retrieved_memories: List[MemoryEntry]) -> Optional[str]:
        model, lock = self.model_loader.get_model_and_lock(self.completion_model)
        if not model: return None
        
        formatted_history = "\n".join([f"{s}: {t}" for s, t in history[-4:]])
        
        memory_section = "Brak."
        if retrieved_memories:
            memory_texts = [f"- {s} wcześniej powiedział: '{t}'" for s, t, _ in retrieved_memories]
            memory_section = "\n".join(memory_texts)

        prompt = (
            f"Twoim jedynym zadaniem jest odegrać rolę asystenta AI o nazwie '{self.name}' w rozmowie.\n"
            f"Odpowiedź MUSI być JEDYNIE tekstem wypowiedzi w języku polskim. NIE dodawaj swoich myśli, tagów ani niczego innego.\n\n"
            f"### PRZYKŁAD POPRAWNEJ ODPOWIEDZI:\n"
            f"Użytkownik: Myślę o nowym projekcie.\n"
            f"{self.name}: Brzmi interesująco, czy możesz opowiedzieć o nim coś więcej?\n\n"
            f"### KONIEC PRZYKŁADU ###\n\n"
            f"--- AKTUALNA ROZMOWA ---\n"
            f"### Kluczowe fakty z tej rozmowy:\n{memory_section}\n\n"
            f"### Ostatnie wiadomości:\n{formatted_history}\n\n"
            f"--- KONIEC ROZMOWY ---\n\n"
            f"{self.name}:"
        )

        try:
            async with lock:
                stop = [f"{name}:" for name in ["Użytkownik", "Agent Ogólny", "Agent Finansowy"]]
                output = await asyncio.to_thread(model.create_completion, prompt,
                    max_tokens=self.model_loader.config.max_tokens,
                    temperature=0.7,
                    top_p=0.9,
                    stop=stop,
                    repeat_penalty=self.model_loader.config.repeat_penalty
                )
                raw_text = output['choices'][0]['text'].strip()
                return self._parse_and_validate_response(raw_text)
        except Exception as e:
            logger.error(f"Błąd generowania odpowiedzi: {e}"); return None

    async def respond(self, history: ConversationHistory) -> Optional[str]:
        if not history: return None
        last_utterance = history[-1][1]
        query_embedding = await self._get_embedding(last_utterance)
        if not query_embedding:
            logger.warning("Nie udało się stworzyć wektora zapytania. Odpowiadam bez pamięci.")
            return await self._generate_response(history, [])
        retrieved_memories = self.memory.search_similar(query_embedding, top_k=2, threshold=self.model_loader.config.memory_retrieval_threshold)
        if retrieved_memories:
            logger.info(f"[{self.name}] Odnaleziono {len(retrieved_memories)} istotnych wspomnień.")
        else:
            logger.info(f"[{self.name}] Brak istotnych wspomnień, odpowiadam na podstawie bieżącego kontekstu.")
        return await self._generate_response(history, retrieved_memories)

# --- Symulator Rozmowy ---
class ConversationSimulator:
    def __init__(self, agents: List[ConversationalAgent], memory: VectorMemory, initial_prompt: Tuple[str, str], max_turns: int = 4):
        self.agents = agents; self.memory = memory
        self.history: ConversationHistory = []
        self.initial_prompt = initial_prompt
        self.max_turns = max_turns

    async def initialize(self):
        speaker, text = self.initial_prompt
        embedding = await self.agents[0]._get_embedding(text)
        if embedding:
            # Dodajemy do historii i pamięci od razu
            self.history.append((speaker, text))
            self.memory.add_entry(speaker, text, embedding)
        else:
            raise RuntimeError("Nie udało się zwektoryzować początkowego promptu.")

    async def run(self):
        await self.initialize()
        logger.info("--- Rozpoczynam symulację rozmowy (v8 Lepsze Prompty i Parser) ---")
        logger.info(f"\033[1;33m{self.history[0][0]}: {self.history[0][1]}\033[0m")
        
        current_agent_idx = 0
        for _ in range(self.max_turns * len(self.agents)):
            agent = self.agents[current_agent_idx]
            response = await agent.respond(self.history)
            
            if response:
                embedding = await agent._get_embedding(response)
                if embedding:
                    self.history.append((agent.name, response))
                    self.memory.add_entry(agent.name, response, embedding)
                    color = "\033[1;34m" if current_agent_idx == 0 else "\033[1;32m"
                    logger.info(f"{color}{agent.name}: {response}\033[0m")
                else:
                    logger.warning(f"Nie udało się zwektoryzować ZWALIDOWANEJ odpowiedzi od {agent.name}.")
            else:
                logger.warning(f"{agent.name} nie wygenerował poprawnej odpowiedzi. Koniec tury.")
            
            current_agent_idx = (current_agent_idx + 1) % len(self.agents)
            await asyncio.sleep(1)
        logger.info("--- Koniec symulacji rozmowy ---")

# --- Główna Funkcja Aplikacji ---
async def main():
    config = AppConfig()
    loader = ModelLoader(config)
    try:
        if await loader.load_all_models():
            vector_memory = VectorMemory()
            agent_a = ConversationalAgent("Agent Ogólny", loader, "base", vector_memory)
            agent_b = ConversationalAgent("Agent Finansowy", loader, "finance_instruct", vector_memory)
            
            initial_prompt = ("Użytkownik", "Nasz nowy system logistyczny oparty na AI zredukował koszty o 20%, ale wymaga drogiej infrastruktury chmurowej. Zastanawiam się nad jego rentownością.")
            simulator = ConversationSimulator([agent_a, agent_b], vector_memory, initial_prompt, max_turns=3)
            await simulator.run()
    finally:
        loader.unload_all_models()

if __name__ == "__main__":
    if 'ipykernel' in sys.modules:
        nest_asyncio.apply()
    asyncio.run(main())
