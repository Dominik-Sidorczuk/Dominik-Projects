import asyncio
import argparse
import logging
import traceback
from typing import Dict, Any, List
import pandas as pd 
from datetime import datetime
from pathlib import Path
import multiprocessing as mp
import os
import faulthandler
import orjson
from tqdm.asyncio import tqdm_asyncio

# Importy z finalnych, poprawionych wersji wszystkich komponentów
from Agent_Config import AppConfig
from Adaptive_Agent_Handler import MultiModelHandler, model_server_process
from Agent_Scraper import WebScraper
from Adaptive_Agent_Swarm import ThoughtfulSwarmOrchestrator
from Adaptive_Agent_Prompts import AnalysisReport

# Konfiguracja loggera
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[
        logging.FileHandler("agent_run_final.log", mode='w', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("agent_run")


async def process_company(company_name: str, main_goal: str, orchestrator: ThoughtfulSwarmOrchestrator) -> Dict[str, Any]:
    """Przetwarza pojedynczą firmę i zwraca cały końcowy stan z orkiestratora."""
    try:
        final_state = await orchestrator.run(company_name=company_name, main_goal=main_goal)
        final_state["nazwa_firmy"] = company_name
        final_state["temat_dotacji"] = main_goal # Przechowujemy pełny cel
        
        if final_state.get("error"):
            logger.error(f"[{company_name}] Proces myślowy zakończony błędem: {final_state['error']}")
        elif isinstance(final_state.get("final_report"), AnalysisReport):
            logger.info(f"[{company_name}] Pomyślnie wygenerowano raport końcowy.")
        else:
            logger.warning(f"[{company_name}] Proces zakończony, ale nie wygenerowano raportu.")
            final_state["error"] = "Proces zakończony bez wygenerowania raportu."
            
        return final_state
    except Exception as e:
        error_trace = traceback.format_exc()
        logger.error(f"[{company_name}] Krytyczny, nieobsłużony błąd: {e}\n{error_trace}")
        return {"error": f"Analiza nieudana: {e}", "nazwa_firmy": company_name, "temat_dotacji": main_goal}

def save_results(results: List[Dict[str, Any]], output_file: str, format: str):
    """Zapisuje wyniki do pliku w wybranym formacie."""
    logger.info(f"Zapisywanie {len(results)} wyników do '{output_file}' w formacie {format.upper()}...")
    if format == 'csv':
        report_data = []
        for res in results:
            item = {"nazwa_firmy": res.get("nazwa_firmy"), "temat_dotacji": res.get("temat_dotacji")}
            if res.get("final_report"):
                report_obj = res["final_report"]
                report_dict = report_obj.model_dump() if isinstance(report_obj, AnalysisReport) else report_obj
                item.update(report_dict)
            elif res.get("error"):
                 item["error"] = res["error"]
            report_data.append(item)
        if report_data:
            pd.json_normalize(report_data, sep='_').to_csv(output_file, index=False, encoding='utf-8-sig')
    elif format == 'jsonl':
        with open(output_file, 'w', encoding='utf-8') as f:
            for item in results:
                # Serializuj obiekty Pydantic i usuń te, które nie są serializowalne
                if 'final_report' in item and isinstance(item.get('final_report'), AnalysisReport):
                    item['final_report'] = item['final_report'].model_dump()
                if 'decision' in item and isinstance(item.get('decision'), BaseModel):
                    item['decision'] = item['decision'].model_dump()
                item.pop('current_thought', None)
                item.pop('thought_history', None)
                f.write(orjson.dumps(item, option=orjson.OPT_APPEND_NEWLINE).decode('utf-8'))

async def main(args: argparse.Namespace) -> None:
    start_time = datetime.now()
    logger.info(f"Główny proces uruchomiony. PID: {os.getpid()}")
    faulthandler.enable()
    
    llm_handler = None
    model_server_proc = None
    ctx = mp.get_context("spawn")
    request_q = ctx.Queue()
    response_q = ctx.Queue()

    try:
        input_path = Path(args.input_file)
        if not input_path.is_file():
            raise FileNotFoundError(f"Plik wejściowy nie został znaleziony: {args.input_file}")

        model_server_proc = ctx.Process(target=model_server_process, args=(request_q, response_q, config), daemon=True)
        model_server_proc.start()
        
        async with WebScraper(config) as scraper:
            llm_handler = MultiModelHandler(app_config=config, request_q=request_q, response_q=response_q, server_process=model_server_proc)
            await llm_handler.initialize()
            orchestrator = ThoughtfulSwarmOrchestrator(scraper, llm_handler, config)

            df = pd.read_csv(input_path)
            if args.company_col not in df.columns or args.grant_col not in df.columns:
                raise ValueError(f"Brak wymaganych kolumn w pliku '{args.input_file}'. Oczekiwano '{args.company_col}' i '{args.grant_col}'.")

            tasks = []
            for _, row in df.iterrows():
                company_name = str(row.get(args.company_col, "")).strip()
                grant_topic = str(row.get(args.grant_col, "")).strip()
                if company_name and grant_topic:
                    main_goal = orchestrator.prompts.get_main_goal(company_name=company_name, grant_topic=grant_topic)
                    tasks.append(process_company(company_name, main_goal, orchestrator))

            logger.info(f"Rozpoczynanie analizy dla {len(tasks)} firm...")
            results = []
            for task in tqdm_asyncio.as_completed(tasks, total=len(tasks), desc="Analiza firm"):
                result = await task
                results.append(result)
                if len(results) % 5 == 0 or len(results) == len(tasks):
                    save_results(results, args.output_file, args.output_format)

            logger.info(f"Analiza zakończona. Finalne wyniki w '{args.output_file}'.")

    except Exception as e:
        logger.critical(f"Krytyczny błąd w funkcji `main`: {e}", exc_info=True)
    finally:
        logger.info("Rozpoczynanie procedury zwalniania zasobów...")
        if llm_handler:
            llm_handler.shutdown()
        duration = datetime.now() - start_time
        logger.info(f"Wszystkie zasoby zwolnione. Całkowity czas analizy: {duration}")

if __name__ == "__main__":
    try:
        mp.set_start_method("spawn", force=True)
    except RuntimeError:
        pass
    
    parser = argparse.ArgumentParser(description="Uruchamia rój agentów do analizy firm.", formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--input-file", type=str, required=True, help="Ścieżka do wejściowego pliku CSV.")
    parser.add_argument("--output-file", type=str, required=True, help="Ścieżka do wyjściowego pliku (CSV lub JSONL).")
    parser.add_argument("--company-col", type=str, default="nazwa_firmy", help="Nazwa kolumny z nazwą firmy.")
    parser.add_argument("--grant-col", type=str, default="temat_wniosku", help="Nazwa kolumny z tematem dotacji.")
    parser.add_argument("--output-format", type=str, choices=['csv', 'jsonl'], default='csv', help="Format pliku wyjściowego:\n  csv   - Spłaszczony raport, dobry do arkuszy kalkulacyjnych.\n  jsonl - Pełny stan końcowy, idealny do analizy i debugowania.")
    
    try:
        asyncio.run(main(parser.parse_args()))
    except KeyboardInterrupt:
        logger.warning("Skrypt przerwany przez użytkownika (Ctrl+C).")
