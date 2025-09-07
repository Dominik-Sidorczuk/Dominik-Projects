import asyncio
import json
import logging
import re
from typing import Dict, List, Optional, Tuple, Type, TYPE_CHECKING

from llama_cpp import Llama, LlamaGrammar
from orjson import loads as json_loads
from pydantic import BaseModel, ValidationError
from pydantic_gbnf_grammar_generator import (
    generate_gbnf_grammar_and_documentation,
)

# Importy z finalnych, poprawionych wersji komponentów
from Agent_Config import AppConfig

if TYPE_CHECKING:
    from Adaptive_Agent_Handler import MultiModelHandler

# Konfiguracja loggerów
agent_logger = logging.getLogger("agent_client")
thought_logger = logging.getLogger("agent_thoughts")


class EmbeddingAwareLocalAgentClient:
    def __init__(
        self,
        handler: "MultiModelHandler",
        llama_model: Llama,
        agent_name: str,
        schema: Type[BaseModel],
        config: AppConfig,
        model_lock: asyncio.Lock,
    ):
        if not isinstance(llama_model, Llama):
            raise TypeError("llama_model musi być instancją Llama.")
        if not issubclass(schema, BaseModel):
            raise TypeError(f"Schemat dla agenta '{agent_name}' musi być modelem Pydantic.")

        self.handler = handler
        self.llama = llama_model
        self.agent_name = agent_name
        self.schema = schema
        self.config = config
        self.model_lock = model_lock
        self.grammar, self.documentation = self._initialize_grammar_and_docs()

    def _initialize_grammar_and_docs(self) -> Tuple[Optional[LlamaGrammar], str]:
        """Generuje gramatykę GBNF i dokumentację na podstawie schematu Pydantic."""
        try:
            gbnf_grammar, documentation = generate_gbnf_grammar_and_documentation(
                [self.schema]
            )
            grammar = LlamaGrammar.from_string(gbnf_grammar, verbose=False)
            agent_logger.info(
                f"[{self.agent_name}] Pomyślnie zainicjowano gramatykę GBNF dla schematu {self.schema.__name__}."
            )
            return grammar, documentation
        except Exception as e:
            agent_logger.error(
                f"[{self.agent_name}] Nie udało się zainicjować gramatyki GBNF: {e}",
                exc_info=True,
            )
            return None, ""

    def _format_messages(self, messages: List[Dict[str, str]]) -> str:
        """Formatuje wiadomości do formatu ChatML oczekiwanego przez model."""
        formatted = [
            f"<|im_start|>{msg.get('role', 'user')}\n{msg.get('content', '').strip()}<|im_end|>"
            for msg in messages
            if msg.get("content", "").strip()
        ]
        return "\n".join(formatted) + "\n<|im_start|>assistant\n"

    def _log_agent_thought_process(
        self, prompt: str, raw_response: str, final_output: Dict, status: str, attempt: int
    ):
        """Loguje kompletny proces myślowy agenta dla celów diagnostycznych."""
        log_message = (
            f"\n--- PROCES MYŚLOWY AGENTA [{self.agent_name}] ---\n"
            f"STATUS: {status} (po {attempt} próbach)\n"
            f"--- PROMPT WEJŚCIOWY ---\n{prompt}\n"
            f"--- SUROWA ODPOWIEDŹ LLM ---\n{raw_response}\n"
            f"--- SFINALIZOWANY WYNIK ---\n{json.dumps(final_output, indent=2, ensure_ascii=False)}\n"
            f"--- KONIEC PROCESU [{self.agent_name}] ---\n"
        )
        thought_logger.debug(log_message)

    def _fix_and_validate_response(self, raw_text: str) -> BaseModel:
        """Próbuje sparsować i zwalidować odpowiedź, zwracając instancję modelu Pydantic."""
        raw_text = raw_text.strip()
        try:
            # Pierwsza próba: załóż, że tekst to czysty JSON
            return self.schema(**json_loads(raw_text))
        except (ValidationError, Exception):
            # Druga próba: poszukaj obiektu JSON wewnątrz tekstu (np. otoczonego wyjaśnieniami)
            if json_match := re.search(r"\{.*\}", raw_text, re.DOTALL):
                try:
                    return self.schema(**json_loads(json_match.group(0)))
                except (ValidationError, Exception) as e:
                    raise ValueError(f"Nie udało się naprawić i zweryfikować JSON: {e}") from e
        raise ValueError("Nie znaleziono poprawnego obiektu JSON w odpowiedzi modelu.")

    async def create(
        self, messages: List[Dict[str, str]], max_retries: int
    ) -> Tuple[Optional[BaseModel], str]:
        """
        Główna metoda wykonawcza agenta. Tworzy prompt, wywołuje LLM, waliduje
        i zwraca ustrukturyzowaną odpowiedź.
        """
        system_message = (
            f"Jesteś wyspecjalizowanym asystentem AI o nazwie '{self.agent_name}'. "
            "Twoim zadaniem jest wygenerowanie poprawnego obiektu JSON na podstawie dostarczonego kontekstu. "
            f"Oczekiwany schemat wyjściowy to {self.schema.__name__}:\n\n{self.documentation}\n\n"
            "Odpowiedz TYLKO i wyłącznie obiektem JSON. Nie dodawaj żadnych wyjaśnień ani tekstu przed lub po JSON."
        )
        prompt = f"<|im_start|>system\n{system_message}<|im_end|>\n{self._format_messages(messages)}"

        raw_text = ""
        validated_model: Optional[BaseModel] = None
        status = "FAILURE"
        
        for attempt in range(1, max_retries + 1):
            try:
                async with self.model_lock:
                    response = await asyncio.to_thread(
                        self.llama,
                        prompt=prompt,
                        max_tokens=self.config.models.max_new_tokens,
                        temperature=0.01,
                        top_p=0.95,
                        stop=["</s>", "<|im_end|>"],
                        grammar=self.grammar,
                    )
                raw_text = response["choices"][0]["text"].strip()
                if not raw_text:
                    agent_logger.warning(
                        f"[{self.agent_name}] Próba {attempt}/{max_retries}: Model zwrócił pustą odpowiedź."
                    )
                    continue

                validated_model = self._fix_and_validate_response(raw_text)
                status = "SUCCESS"
                agent_logger.info(f"[{self.agent_name}] Pomyślnie wygenerowano i zwalidowano odpowiedź w próbie {attempt}.")
                break
            except Exception as e:
                agent_logger.error(
                    f"[{self.agent_name}] Błąd w próbie {attempt}/{max_retries}: {e}"
                )
                if attempt < max_retries:
                    await asyncio.sleep(0.5 * attempt) # Prosty exponential backoff

        final_log_output = (
            validated_model.model_dump()
            if validated_model
            else {"error": f"Nie udało się po {max_retries} próbach.", "last_raw_response": raw_text}
        )
        self._log_agent_thought_process(prompt, raw_text, final_log_output, status, attempt)

        return validated_model, raw_text
