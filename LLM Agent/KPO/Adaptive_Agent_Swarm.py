import asyncio
import logging
import time
from typing import List, Optional, Dict, Any, TypedDict
from langgraph.graph import StateGraph, END
from pydantic import BaseModel, Field
from collections import OrderedDict
import numpy as np

# Importy z finalnych wersji komponentów projektu
from Agent_Config import AppConfig
from Agent_Scraper import WebScraper
from Adaptive_Agent_Handler import MultiModelHandler
from Adaptive_Agent_Prompts import (
    Prompts, ExtractedFactList, SimpleSummary,
    CoTStep1Claims, CoTStep2Evidence, CoTStep3Inconsistencies, CoTStep4RedFlags,
    AnalysisReport, SearchQueries, Decision
)

# Konfiguracja loggera
logger = logging.getLogger("swarm_workflow")
thought_store_logger = logging.getLogger("thought_store")

# --- Struktury Danych ---

class Thought(BaseModel):
    """Reprezentuje pojedynczą "myśl" w systemie."""
    id: int
    embedding: List[float]
    source_text: str = ""
    origin_agent: str
    timestamp: float = Field(default_factory=time.time)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    class Config: arbitrary_types_allowed = True

class ThoughtProcessState(TypedDict):
    """Definiuje stan grafu dla LangGraph."""
    main_goal: str
    company_name: str
    search_queries: List[str]
    search_history: List[str]
    current_thought: Thought
    thought_history: List[Thought]
    summaries: Dict[str, str]
    extracted_facts: List[Dict[str, Any]]
    cot_claims: List[str]
    cot_evidence: List[Dict]
    cot_inconsistencies: List[Dict]
    cot_red_flags: List[Dict]
    final_report: Optional[AnalysisReport]
    iteration_count: int
    error: Optional[str]
    decision: Optional[Decision]

class ThoughtVectorStore:
    """Prosta, wewnątrz-pamięciowa baza wektorowa."""
    def __init__(self):
        self.thoughts: OrderedDict[int, Thought] = OrderedDict()
        self.vectors: Optional[np.ndarray] = None
        self._next_id = 0
    def add(self, thought: Thought) -> int:
        thought.id = self._next_id
        self.thoughts[self._next_id] = thought
        new_vector = np.array(thought.embedding).reshape(1, -1)
        self.vectors = np.vstack([self.vectors, new_vector]) if self.vectors is not None else new_vector
        self._next_id += 1
        return thought.id

# --- KLASA ZAWIERAJĄCA LOGIKĘ WĘZŁÓW GRAFU ---

class GraphNodes:
    """Hermetyzuje logikę wykonawczą dla każdego węzła w grafie LangGraph."""
    def __init__(self, scraper: WebScraper, llm_handler: MultiModelHandler, prompts: Prompts, thought_store: ThoughtVectorStore):
        self.scraper = scraper
        self.llm_handler = llm_handler
        self.prompts = prompts
        self.thought_store = thought_store

    async def _invoke_agent(self, agent_name: str, prompt_messages: List[Dict[str, str]], expected_model: type) -> Optional[BaseModel]:
        try:
            validated_model, raw_text = await self.llm_handler.invoke_agent(agent_name, prompt_messages)
            if isinstance(validated_model, expected_model):
                return validated_model
            logger.error(f"Agent '{agent_name}' zwrócił nieoczekiwany typ. Oczekiwano {expected_model.__name__}, otrzymano {type(validated_model).__name__}.")
            return None
        except Exception as e:
            logger.critical(f"Krytyczny błąd podczas wywoływania agenta '{agent_name}': {e}", exc_info=True)
            return None

    async def hunter_node(self, state: ThoughtProcessState) -> Dict[str, Any]:
        logger.info(f">>> Węzeł [ITERACJA {state['iteration_count']}]: Hunter (Generator Zapytań)")
        prompt = self.prompts.get_hunter_prompt(state['company_name'], state['main_goal'], state['search_history'])
        queries_model = await self._invoke_agent("hunter", [{"role": "user", "content": prompt}], SearchQueries)
        if not queries_model or not queries_model.queries:
             return {"error": "Hunter nie zdołał wygenerować zapytań."}
        new_queries = [q for q in queries_model.queries if q not in state['search_history']]
        return {"search_queries": new_queries, "search_history": state['search_history'] + new_queries}

    async def fact_extractor_node(self, state: ThoughtProcessState) -> Dict[str, Any]:
        logger.info(f">>> Węzeł [ITERACJA {state['iteration_count']}]: Ekstraktor Faktów (Równolegle)")
        if not state['search_queries']:
            return {}

        results = await self.scraper.run_searches(queries=state['search_queries'], max_links_per_query=3)
        if not results: return {"error": "Scraper nie zwrócił żadnych danych."}

        # --- OPTYMALIZACJA: Tworzenie zadań do wykonania równoległego ---
        summary_tasks = []
        fact_tasks = []
        valid_results = [r for r in results if r and r.text]

        for r in valid_results:
            summary_tasks.append(self._invoke_agent("summarizer", [{"role": "user", "content": self.prompts.get_summarize_prompt(r.text)}], SimpleSummary))
            fact_tasks.append(self._invoke_agent("fact_extractor", [{"role": "user", "content": self.prompts.get_fact_extract_prompt(r.text, r.url)}], ExtractedFactList))

        # --- OPTYMALIZACJA: Równoległe wykonanie wszystkich zadań ---
        logger.info(f"Uruchamiam {len(summary_tasks)} zadań streszczenia i {len(fact_tasks)} zadań ekstrakcji równolegle...")
        all_results = await asyncio.gather(*summary_tasks, *fact_tasks, return_exceptions=True)
        
        # Rozdzielenie wyników
        summary_results = all_results[:len(summary_tasks)]
        fact_results = all_results[len(summary_tasks):]

        # Agregacja wyników z uwzględnieniem błędów
        all_summaries = state.get('summaries', {})
        all_facts = state.get('extracted_facts', [])

        for i, res in enumerate(summary_results):
            if isinstance(res, SimpleSummary):
                all_summaries[valid_results[i].url] = res.content
            elif isinstance(res, Exception):
                logger.warning(f"Zadanie streszczenia dla {valid_results[i].url} nie powiodło się: {res}")

        for i, res in enumerate(fact_results):
            if isinstance(res, ExtractedFactList):
                all_facts.extend([f.model_dump() for f in res.facts])
            elif isinstance(res, Exception):
                logger.warning(f"Zadanie ekstrakcji faktów dla {valid_results[i].url} nie powiodło się: {res}")

        if not all_summaries: return {"error": "Nie udało się wygenerować żadnych streszczeń."}
            
        aggregated_summary = " ".join(all_summaries.values())
        embedding = await self.llm_handler.get_embedding(aggregated_summary)
        new_thought = Thought(id=-1, embedding=embedding.tolist(), source_text=aggregated_summary, origin_agent="fact_extractor")
        
        return {"summaries": all_summaries, "extracted_facts": all_facts, "current_thought": new_thought, "thought_history": state['thought_history'] + [new_thought]}

    async def reasoner_node(self, state: ThoughtProcessState) -> Dict[str, Any]:
        logger.info(f">>> Węzeł [ITERACJA {state['iteration_count']}]: Reasoner CoT")
        claims_model = await self._invoke_agent("cot_step1_claims", [{"role": "user", "content": self.prompts.get_cot_step1_claims_prompt(state['summaries'], state['extracted_facts'], state['main_goal'])}], CoTStep1Claims)
        if not claims_model: return {"error": "CoT Krok 1 (Claims) nie powiódł się."}
        
        evidence_model = await self._invoke_agent("cot_step2_evidence", [{"role": "user", "content": self.prompts.get_cot_step2_evidence_prompt(claims_model.claims, state['extracted_facts'])}], CoTStep2Evidence)
        if not evidence_model: return {"error": "CoT Krok 2 (Evidence) nie powiódł się."}

        incons_model = await self._invoke_agent("cot_step3_inconsistencies", [{"role": "user", "content": self.prompts.get_cot_step3_inconsistencies_prompt([e.model_dump() for e in evidence_model.evidence_list])}], CoTStep3Inconsistencies)
        if not incons_model: return {"error": "CoT Krok 3 (Inconsistencies) nie powiódł się."}

        redflags_model = await self._invoke_agent("cot_step4_redflags", [{"role": "user", "content": self.prompts.get_cot_step4_redflags_prompt([i.model_dump() for i in incons_model.inconsistencies])}], CoTStep4RedFlags)
        if not redflags_model: return {"error": "CoT Krok 4 (RedFlags) nie powiódł się."}

        return {"cot_claims": claims_model.claims, "cot_evidence": [e.model_dump() for e in evidence_model.evidence_list], "cot_inconsistencies": [i.model_dump() for i in incons_model.inconsistencies], "cot_red_flags": [rf.model_dump() for rf in redflags_model.red_flags]}

    async def decider_node(self, state: ThoughtProcessState) -> Dict[str, Any]:
        logger.info(f">>> Węzeł [ITERACJA {state['iteration_count']}]: Decider (Krytyk)")
        prompt = self.prompts.get_decider_prompt(state['main_goal'], state['cot_red_flags'], state['search_history'])
        decision_model = await self._invoke_agent("decider", [{"role": "user", "content": prompt}], Decision)
        if not decision_model: return {"error": "Decider nie podjął decyzji."}
        logger.info(f"Decyzja: Gotowy do raportu? {decision_model.is_ready_for_report}. Powód: {decision_model.reasoning}")
        return {"decision": decision_model}

    async def synthesizer_node(self, state: ThoughtProcessState) -> Dict[str, Any]:
        logger.info(">>> Węzeł: Syntezator Raportu")
        prompt = self.prompts.get_report_compiler_prompt(state['company_name'], state['main_goal'], list(state['summaries'].keys()), state)
        report_model = await self._invoke_agent("report_compiler", [{"role": "user", "content": prompt}], AnalysisReport)
        if not report_model: return {"error": "Nie udało się wygenerować raportu końcowego."}
        return {"final_report": report_model}

# --- GŁÓWNY ORKESTRATOR ROJU Z LANGGRAPH ---

class ThoughtfulSwarmOrchestrator:
    def __init__(self, scraper: WebScraper, llm_handler: MultiModelHandler, app_config: AppConfig, max_iterations: int = 3):
        self.prompts = Prompts()
        self.thought_store = ThoughtVectorStore()
        self.max_iterations = max_iterations
        self.nodes = GraphNodes(scraper, llm_handler, self.prompts, self.thought_store)
        self.app = self._build_graph()
        logger.info(f"Orkiestrator Roju Myśli (z pętlą analityczną v3.0) zainicjalizowany.")

    def _router_node(self, state: ThoughtProcessState) -> str:
        if state.get('error'): return END
        
        decision = state.get('decision')
        if not decision: return END # Na wszelki wypadek

        if decision.is_ready_for_report:
            return "synthesizer"
        if state['iteration_count'] >= self.max_iterations:
            logger.warning(f"Osiągnięto maksymalną liczbę iteracji ({self.max_iterations}). Przechodzę do syntezy.")
            return "synthesizer"

        state['iteration_count'] += 1
        return "hunter"

    def _build_graph(self) -> "StateGraph":
        workflow = StateGraph(ThoughtProcessState)
        workflow.add_node("hunter", self.nodes.hunter_node)
        workflow.add_node("fact_extractor", self.nodes.fact_extractor_node)
        workflow.add_node("reasoner", self.nodes.reasoner_node)
        workflow.add_node("decider", self.nodes.decider_node)
        workflow.add_node("synthesizer", self.nodes.synthesizer_node)
        
        workflow.set_entry_point("hunter")
        workflow.add_edge("hunter", "fact_extractor")
        workflow.add_edge("fact_extractor", "reasoner")
        workflow.add_edge("reasoner", "decider")
        workflow.add_conditional_edges("decider", self._router_node, {"hunter": "hunter", "synthesizer": "synthesizer", END: END})
        workflow.add_edge("synthesizer", END)
        
        return workflow.compile()

    async def run(self, company_name: str, main_goal: str) -> Dict[str, Any]:
        logger.info(f"Rozpoczynam nowy proces myślowy dla '{company_name}' | Cel: '{main_goal}'")
        try:
            initial_embedding = await self.llm_handler.get_embedding(main_goal)
            initial_thought = Thought(id=-1, embedding=initial_embedding.tolist(), source_text=main_goal, origin_agent="user_input")
            initial_thought.id = self.thought_store.add(initial_thought)
            
            initial_state: ThoughtProcessState = {
                "main_goal": main_goal, "company_name": company_name, "search_queries": [],
                "search_history": [], "current_thought": initial_thought, "thought_history": [initial_thought],
                "summaries": {}, "extracted_facts": [], "cot_claims": [], "cot_evidence": [],
                "cot_inconsistencies": [], "cot_red_flags": [], "final_report": None,
                "error": None, "iteration_count": 1, "decision": None
            }
            
            final_state = await self.app.ainvoke(initial_state)
            return final_state
        except Exception as e:
            logger.critical(f"Proces myślowy przerwany przez krytyczny błąd: {e}", exc_info=True)
            return {"error": str(e)}
