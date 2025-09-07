import orjson
from typing import Any, Dict, List, Optional, Union
from pydantic import BaseModel, Field, field_validator, model_validator, ConfigDict
from enum import Enum

# --- Funkcje Pomocnicze i Enumy ---

def validate_unique_list(v: List[str], field_name: str) -> List[str]:
    """Sprawdza, czy wszystkie elementy na liście stringów są unikalne."""
    if len(v) != len(set(v)):
        raise ValueError(f"Wartości w polu '{field_name}' muszą być unikalne.")
    return v

# --- Schematy Danych dla Agentów ---

class StrategicPlan(BaseModel):
    """Definiuje plan strategiczny złożony z konkretnych podcelów."""
    model_config = ConfigDict(validate_assignment=True, str_strip_whitespace=True, extra='forbid')
    subgoals: List[str] = Field(..., description="Lista unikalnych, zróżnicowanych i wykonalnych podcelów.", min_length=1, max_length=10)

    @field_validator('subgoals')
    @classmethod
    def validate_unique_subgoals(cls, v: List[str]) -> List[str]:
        return validate_unique_list(v, 'subgoals')

class SearchQueries(BaseModel):
    """Definiuje zapytania wyszukiwania dla agenta Hunter."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    queries: List[str] = Field(..., description="Lista maksymalnie 5 unikalnych zapytań do wyszukiwarki.", min_length=1, max_length=5)

    @field_validator('queries')
    @classmethod
    def validate_unique_queries(cls, v: List[str]) -> List[str]:
        return validate_unique_list(v, 'queries')

class SimpleSummary(BaseModel):
    """Reprezentuje ustrukturyzowane podsumowanie tekstu."""
    model_config = ConfigDict(validate_assignment=True, str_strip_whitespace=True, extra='forbid')
    content: str = Field(..., description="Zwięzłe, obiektywne podsumowanie tekstu skoncentrowane na ryzyku.")

class Fact(BaseModel):
    """Reprezentuje pojedynczy, ustrukturyzowany fakt."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    key: str = Field(..., description="Nazwa faktu (np. 'Data rejestracji').")
    value: Union[str, int, float] = Field(..., description="Wartość faktu.")
    confidence: float = Field(..., description="Poziom pewności ekstrakcji (0.0-1.0).", ge=0.0, le=1.0)
    source_url: Optional[str] = Field(None, description="URL źródła faktu.")

class ExtractedFactList(BaseModel):
    """Lista faktów wyekstrahowanych z tekstu."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    facts: List[Fact] = Field(..., description="Lista obiektów Fact.")

class CoTStep1Claims(BaseModel):
    """Krok 1 CoT: Identyfikacja kluczowych deklaracji i twierdzeń."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    claims: List[str] = Field(..., description="Lista kluczowych, weryfikowalnych deklaracji firmy.", min_length=1)

class Evidence(BaseModel):
    """Reprezentuje pojedynczy dowód powiązany z deklaracją."""
    claim: str = Field(..., description="Deklaracja, której dotyczy dowód.")
    supporting_evidence: List[str] = Field(default_factory=list)
    contradicting_evidence: List[str] = Field(default_factory=list)

class CoTStep2Evidence(BaseModel):
    """Krok 2 CoT: Zbieranie dowodów za i przeciw każdej deklaracji."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    evidence_list: List[Evidence] = Field(..., description="Lista dowodów dla każdej analizowanej deklaracji.")

class Inconsistency(BaseModel):
    """Reprezentuje pojedynczą wykrytą niespójność."""
    description: str = Field(..., description="Zwięzły opis wykrytej niespójności lub braku danych.")
    related_claims: List[str] = Field(..., description="Deklaracje, których dotyczy niespójność.")

class CoTStep3Inconsistencies(BaseModel):
    """Krok 3 CoT: Identyfikacja niespójności między deklaracjami a dowodami."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    inconsistencies: List[Inconsistency] = Field(..., description="Lista zidentyfikowanych niespójności.")

class RedFlag(BaseModel):
    """Reprezentuje pojedyncze zidentyfikowane ryzyko ('czerwoną flagę')."""
    opis: str = Field(..., description="Zwięzły opis zidentyfikowanego ryzyka.", min_length=10)
    poziom_zaufania: float = Field(..., description="Poziom pewności co do ryzyka (0.0-1.0).", ge=0.0, le=1.0)
    dowod: str = Field(..., description="Weryfikowalny dowód w postaci cytatu lub podsumowania niespójności.", min_length=20)

class CoTStep4RedFlags(BaseModel):
    """Krok 4 CoT: Formułowanie 'czerwonych flag' na podstawie niespójności."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    red_flags: List[RedFlag] = Field(default_factory=list, description="Lista sformułowanych czerwonych flag.")

class CuriosityHypothesis(BaseModel):
    """Definiuje hipotezę dla nowego kierunku dochodzenia."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    is_interesting: bool = Field(...)
    reasoning: str = Field(...)
    suggested_subgoal: Optional[str] = Field(None)

    @model_validator(mode='after')
    def validate_subgoal_if_interesting(self) -> 'CuriosityHypothesis':
        if self.is_interesting and not self.suggested_subgoal:
            raise ValueError("Proponowany podcel jest wymagany, gdy wątek jest interesujący.")
        return self

class Decision(BaseModel):
    """Reprezentuje decyzję podjętą przez agenta Decider."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    is_ready_for_report: bool = Field(..., description="Czy zebrane informacje są wystarczające do stworzenia raportu końcowego?")
    reasoning: str = Field(..., description="Zwięzłe uzasadnienie decyzji.")
    new_search_queries: Optional[List[str]] = Field(default=None, description="Lista nowych, precyzyjnych zapytań do wyszukiwarki, jeśli `is_ready_for_report` jest fałszywe.")

    @model_validator(mode='after')
    def validate_new_queries(self) -> 'Decision':
        if not self.is_ready_for_report and not self.new_search_queries:
            raise ValueError("Nowe zapytania są wymagane, jeśli raport nie jest gotowy.")
        return self

class AnalysisReport(BaseModel):
    """Kompleksowy raport analityczny."""
    model_config = ConfigDict(validate_assignment=True, extra='forbid')
    profil_dzialalnosci: str = Field(default="Brak wystarczających danych.")
    ocena_logiki_biznesowej_wniosku: str = Field(default="Brak danych do oceny.")
    spojnosc_informacji: str = Field(default="Brak danych do analizy.")
    czerwone_flagi: List[RedFlag] = Field(default_factory=list)
    ocena_wiarygodnosci: int = Field(default=1, ge=1, le=10)
    uzasadnienie_oceny: str = Field(default="Brak danych do uzasadnienia.")
    rekomendacja: str = Field(default='WARUNKOWO')
    warunki_rekomendacji: Optional[List[str]] = Field(default=None)
    zrodla: List[str] = Field(default_factory=list)

    @field_validator('rekomendacja')
    @classmethod
    def validate_recommendation(cls, v: str) -> str:
        valid_values = ['TAK', 'NIE', 'WARUNKOWO']
        v_upper = v.upper()
        if v_upper not in valid_values:
            raise ValueError(f"Rekomendacja musi być jedną z: {', '.join(valid_values)}")
        return v_upper

    @model_validator(mode='after')
    def validate_conditional_recommendation(self) -> 'AnalysisReport':
        if self.rekomendacja == 'WARUNKOWO' and not self.warunki_rekomendacji:
            raise ValueError("Warunki są wymagane dla rekomendacji 'WARUNKOWO'.")
        if self.rekomendacja != 'WARUNKOWO':
            self.warunki_rekomendacji = None
        return self

class Prompts:
    @staticmethod
    def get_main_goal(company_name: str, grant_topic: str) -> str:
        return f"Zweryfikuj firmę '{company_name}' pod kątem potencjalnego ryzyka wyłudzenia w kontekście wniosku o dotację na temat: '{grant_topic}'."
    
    @staticmethod
    def _get_json_format_instruction(schema: Any) -> str:
        schema_name = getattr(schema, "__name__", str(schema))
        return (f"\nTwoja odpowiedź MUSI być pojedynczym, poprawnym obiektem JSON zgodnym ze schematem Pydantic '{schema_name}'. "
                "Nie dodawaj wyjaśnień, komentarzy ani dodatkowego tekstu. Tylko JSON.")

    @staticmethod
    def truncate_text(text: str, max_chars: int = 1500) -> str:
        return text[:max_chars] + "... [skrócono]" if len(text) > max_chars else text

    @staticmethod
    def get_decompose_prompt(main_goal: str, **kwargs) -> str:
        return ("Jesteś ekspertem od due diligence. Twoim zadaniem jest stworzenie zróżnicowanego planu weryfikacji dla podanego celu.\n"
                f"**Cel główny:** \"{main_goal}\"\n"
                "**Wskazówki:** Rozłóż cel na unikalne i zróżnicowane podcele."
                + Prompts._get_json_format_instruction(StrategicPlan))

    @staticmethod
    def get_hunter_prompt(company_name: str, grant_topic: Optional[str], search_history: List[str], **kwargs) -> str:
        history = "\n- ".join(search_history) if search_history else "Brak"
        return ("Jesteś analitykiem ds. wywiadu gospodarczego. Wygeneruj precyzyjne zapytania do wyszukiwarki.\n"
                f"**Firma:** {company_name}\n"
                f"**Temat dotacji:** {Prompts.truncate_text(grant_topic or 'Brak', 100)}\n"
                f"**Historia zapytań:**\n- {history}\n"
                "**Wskazówki:** Generuj zapytania unikalne w stosunku do historii."
                + Prompts._get_json_format_instruction(SearchQueries))

    @staticmethod
    def get_summarize_prompt(text_content: str, **kwargs) -> str:
        return ("Jesteś analitykiem ryzyka. Stwórz zwięzłe streszczenie, skupiając się na faktach wskazujących na ryzyko.\n"
                f"**Tekst do streszczenia:**\n{Prompts.truncate_text(text_content)}\n"
                + Prompts._get_json_format_instruction(SimpleSummary))

    @staticmethod
    def get_fact_extract_prompt(text_content: str, source_url: str, **kwargs) -> str:
        return ("Jesteś ekstraktorem faktów. Wyodrębnij weryfikowalne dane z tekstu.\n"
                f"**URL źródła:** {source_url}\n"
                f"**Tekst do analizy:**\n{Prompts.truncate_text(text_content)}\n"
                "**Wskazówki:** Oceń pewność każdego faktu."
                + Prompts._get_json_format_instruction(ExtractedFactList))

    @staticmethod
    def get_cot_step1_claims_prompt(summaries: Dict[str, str], facts: List[Dict[str, Any]], grant_topic: str) -> str:
        summaries_str = "\n".join([f"- {url}: {summary}" for url, summary in summaries.items()])
        return (
            "Jesteś analitykiem ryzyka. Zidentyfikuj kluczowe deklaracje firmy na podstawie dostępnych danych.\n"
            f"**Kontekst dotacji:** {grant_topic}\n"
            f"**Podsumowania ze źródeł:**\n{Prompts.truncate_text(summaries_str, 2000)}\n"
            "**Instrukcja:** Wyodrębnij listę konkretnych, weryfikowalnych twierdzeń firmy."
            + Prompts._get_json_format_instruction(CoTStep1Claims)
        )

    @staticmethod
    def get_cot_step2_evidence_prompt(claims: List[str], facts: List[Dict[str, Any]]) -> str:
        claims_str = "\n- ".join(claims)
        facts_str = orjson.dumps(facts, option=orjson.OPT_INDENT_2).decode("utf-8")
        return (
            "Jesteś detektywem analitycznym. Znajdź dowody dla podanych deklaracji w dostarczonym zbiorze faktów.\n"
            f"**Deklaracje do weryfikacji:**\n- {claims_str}\n"
            f"**Dostępne fakty:**\n{Prompts.truncate_text(facts_str, 2500)}\n"
            "**Instrukcja:** Dla każdej deklaracji, znajdź dowody, które ją wspierają lub jej zaprzeczają."
            + Prompts._get_json_format_instruction(CoTStep2Evidence)
        )

    @staticmethod
    def get_cot_step3_inconsistencies_prompt(evidence_list: List[Dict[str, Any]]) -> str:
        evidence_str = orjson.dumps(evidence_list, option=orjson.OPT_INDENT_2).decode("utf-8")
        return (
            "Jesteś audytorem. Wykryj niespójności, sprzeczności lub braki w danych na podstawie zebranych dowodów.\n"
            f"**Zestawienie dowodów:**\n{Prompts.truncate_text(evidence_str, 3000)}\n"
            "**Instrukcja:** Wskaż, gdzie informacje są sprzeczne lub gdzie brakuje dowodów."
            + Prompts._get_json_format_instruction(CoTStep3Inconsistencies)
        )

    @staticmethod
    def get_cot_step4_redflags_prompt(inconsistencies: List[Dict[str, Any]]) -> str:
        inconsistencies_str = orjson.dumps(inconsistencies, option=orjson.OPT_INDENT_2).decode("utf-8")
        return (
            "Jesteś ekspertem ds. ryzyka. Przekształć zidentyfikowane niespójności w 'czerwone flagi'.\n"
            f"**Wykryte niespójności:**\n{Prompts.truncate_text(inconsistencies_str, 3000)}\n"
            "**Instrukcja:** Na podstawie każdej niespójności sformułuj 'czerwoną flagę' opisującą ryzyko."
            + Prompts._get_json_format_instruction(CoTStep4RedFlags)
        )

    @staticmethod
    def get_decider_prompt(main_goal: str, red_flags: List[Dict[str, Any]], search_history: List[str]) -> str:
        flags_str = orjson.dumps(red_flags, option=orjson.OPT_INDENT_2).decode("utf-8")
        history_str = "\n- ".join(search_history)
        return (
            "Jesteś krytycznym analitykiem. Oceń dotychczasową analizę i zdecyduj o kolejnych krokach.\n"
            f"**Cel główny:** {main_goal}\n"
            f"**Dotychczas wykonane zapytania:**\n- {history_str}\n"
            f"**Zidentyfikowane czerwone flagi:**\n{Prompts.truncate_text(flags_str, 2000)}\n"
            "**Instrukcja:**\n"
            "1. Oceń, czy zebrane informacje są wystarczające do stworzenia wiarygodnego raportu.\n"
            "2. Jeśli tak, ustaw `is_ready_for_report` na `true`.\n"
            "3. Jeśli nie, ustaw `is_ready_for_report` na `false` i wygeneruj listę BARDZO PRECYZYJNYCH, nowych zapytań, które uzupełnią braki."
            + Prompts._get_json_format_instruction(Decision)
        )

    @staticmethod
    def get_curiosity_prompt(main_goal: str, facts: List[Dict[str, Any]], red_flags: List[Dict[str, Any]], **kwargs) -> str:
        facts_str = orjson.dumps(facts[:10], option=orjson.OPT_INDENT_2).decode("utf-8")
        flags_str = orjson.dumps(red_flags, option=orjson.OPT_INDENT_2).decode("utf-8")
        return ("Jesteś detektywem finansowym. Zidentyfikuj nieoczywiste, ryzykowne wątki na podstawie danych.\n"
                f"**Cel:** {main_goal}\n"
                f"**Fakty:**\n{facts_str}\n"
                f"**Czerwone Flagi:**\n{flags_str}\n"
                "**Wskazówki:** Szukaj anomalii. Ustaw `is_interesting` na `false`, jeśli brak ciekawych wątków."
                + Prompts._get_json_format_instruction(CuriosityHypothesis))

    @staticmethod
    def get_report_compiler_prompt(company_name: str, grant_topic: str, all_urls: List[str], extra_data: Dict[str, Any], **kwargs) -> str:
        # Usunięto serializację extra_data, ponieważ cały stan jest przekazywany w Swarm_v3
        return (f"Jesteś głównym audytorem. Stwórz obiektywny raport końcowy o ryzyku wyłudzenia dla firmy '{company_name}' w kontekście '{grant_topic}'.\n"
                f"**Pełne dane analityczne (stan):**\n{Prompts.truncate_text(str(extra_data), 3000)}\n"
                f"**Źródła:**\n- {', '.join(all_urls)}\n"
                "**Wskazówki:** Opisz profil firmy, oceń logikę wniosku, wylistuj flagi, przyznaj ocenę 1-10 i wydaj rekomendację."
                + Prompts._get_json_format_instruction(AnalysisReport))
