#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import asyncio
import hashlib
import json
import logging
import os
import re
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Set, Optional
from urllib.parse import urlparse
from playwright.async_api import async_playwright, Error as PlaywrightError, Page, Response

# ==============================================================================
# OODA v5.4 — WARSTWA 0: OBSERVER BRIDGE (TRUE SOTA 2026 / SENSORY)
# - Podłącza się do instancji Chrome przez CDP (Auto-Heal)
# - DOM Stabilization Engine: Ignoruje partial-renders i spinnery ładowania
# - Layout-Aware Native Extraction: 100% pasywna ekstrakcja, zero psucia SPA
# - Deep Stable Hashing: Poprawiony błąd deduplikacji (zliczanie całego payloadu)
# ==============================================================================

logging.basicConfig(
    level=os.environ.get("OBSERVER_LOG_LEVEL", "INFO"),
    format="%(asctime)s | %(levelname)-8s | Warstwa 0 | %(message)s"
)
logger = logging.getLogger("ooda-layer0")


@dataclass
class BridgeConfig:
    workspace_dir: Path
    cdp_url: str = field(default_factory=lambda: os.environ.get("OBSERVER_CDP", "http://127.0.0.1:9222"))
    
    # Mechanizmy Anti-Spam i Caching
    dedup_ttl_seconds: int = field(default_factory=lambda: int(os.environ.get("OBSERVER_DEDUP_TTL_SECONDS", "3600")))
    dedup_max_items: int = field(default_factory=lambda: int(os.environ.get("OBSERVER_DEDUP_MAX_ITEMS", "2048")))
    max_text_chars: int = field(default_factory=lambda: int(os.environ.get("OBSERVER_MAX_TEXT_CHARS", "100000"))) # Zwiększono limit by objąć ogromne aplikacje
    
    inbox_dir: Path = field(init=False)
    observations_file: Path = field(init=False)

    def __post_init__(self) -> None:
        self.inbox_dir = self.workspace_dir / "inbox"
        self.observations_file = self.inbox_dir / "observations.jsonl"
        self.inbox_dir.mkdir(parents=True, exist_ok=True)


class TTLFingerprintCache:
    """Odrętwienie na duplikaty z szybkim dostępem O(1)."""
    def __init__(self, ttl_seconds: int, max_items: int):
        self.ttl_seconds = ttl_seconds
        self.max_items = max_items
        self._data: "OrderedDict[str, float]" = OrderedDict()

    def _cleanup(self) -> None:
        now = time.time()
        expired = []
        for key, ts in self._data.items():
            if now - ts > self.ttl_seconds:
                expired.append(key)
            else:
                break
        for key in expired:
            self._data.pop(key, None)

        while len(self._data) > self.max_items:
            self._data.popitem(last=False)

    def seen_recently(self, key: str) -> bool:
        self._cleanup()
        if key in self._data:
            self._data.move_to_end(key)
            self._data[key] = time.time()
            return True
        self._data[key] = time.time()
        self._cleanup()
        return False


class ObservationWriter:
    def __init__(self, observations_file: Path):
        self.observations_file = observations_file

    def append(self, record: Dict) -> None:
        with self.observations_file.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")


class ObserverBridge:
    def __init__(self, config: BridgeConfig):
        self.config = config
        self.writer = ObservationWriter(config.observations_file)
        self.fingerprint_cache = TTLFingerprintCache(
            ttl_seconds=config.dedup_ttl_seconds,
            max_items=config.dedup_max_items,
        )
        self.tracked_pages: Set[Page] = set()

    def infer_domain_name(self, url: str) -> str:
        host = urlparse(url).netloc.lower().replace("www.", "")
        if "linkedin" in host: return "linkedin"
        if "mail.google.com" in host: return "gmail"
        if "github" in host: return "github"
        return host or "generic-web"

    def infer_project_name(self, url: str) -> str:
        host = urlparse(url).netloc.lower().replace("www.", "")
        return host.replace(":", "_") or "browser-observer"

    def fingerprint(self, url: str, title: str, screen_text: str) -> str:
        """
        True SOTA Stable Hash:
        Usunięto wadliwe obcięcie do 4000 znaków. W nowoczesnych SPA (np. Gmail), boczny panel 
        zajmuje potężną ilość miejsca. Obcięcie powodowało, że hasz dla dwóch różnych maili 
        był IDENTYCZNY, przez co skrypt je ignorował. 
        Teraz haszujemy CAŁOŚĆ. Usuwamy jedynie zmienne cyfry (zegary, liczniki powiadomień).
        """
        stable_text = re.sub(r'\b\d+\b', '', screen_text)
        normalized = f"{url}|{title}|{stable_text}"
        return hashlib.sha256(normalized.encode("utf-8")).hexdigest()

    async def extract_semantic_text(self, page: Page) -> str:
        """
        Natywna ekstrakcja Layout-Aware Engine.
        Nie używamy mutacji DOM. Przeglądarka ma potężny silnik C++ który sam wylicza
        właściwość `innerText` ignorując to co schowane (display: none - czyli np. skrzynkę Gmail
        kiedy czytasz maila). My jedynie wyciągamy rdzeń (`main`), jeśli istnieje.
        """
        script = r"""
            () => {
                try {
                    // Znajdujemy główny kontener by pominąć statyczny lewy sidebar
                    // Jeśli nie ma tagu semantycznego, bierzemy całe body.
                    let target = document.querySelector('main, [role="main"], article, .application-form');
                    if (!target) {
                        target = document.body;
                    }
                    
                    // Natywny `innerText` ignoruje wszystko co schowane (display:none)
                    // oraz doskonale radzi sobie ze znakami nowej linii dla paragrafów.
                    let rawText = target.innerText || "";
                    
                    // Delikatna kompresja linii bez puszczania "zupy słownej"
                    return rawText.split('\n')
                        .map(line => line.trim().replace(/[ \t]+/g, ' '))
                        .filter(line => line.length > 0)
                        .join('\n');
                        
                } catch (e) {
                    return "";
                }
            }
        """
        try:
            text = await page.evaluate(script)
            return (text or "").strip()[: self.config.max_text_chars]
        except Exception as e:
            logger.debug("Błąd ekstrakcji semantycznej: %s", e)
            return ""

    def emit_observation(self, url: str, title: str, screen_text: str, fp: str) -> None:
        """Emituje poprawny rekord do bazy systemu OODA."""
        domain = self.infer_domain_name(url)
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "source": "browser",
            "url": url,
            "title": title,
            "screen_text": screen_text,
            "fingerprint": fp,
            "tags": [domain, "browser-tab"], 
            "domain": domain,
            "project": self.infer_project_name(url),
        }
        self.writer.append(record)
        logger.info("👀 Wykryto nowy, stabilny bodziec: %s", title[:60])

    async def sensory_loop(self, page: Page) -> None:
        """
        Continuous Sensory Loop with DOM Stabilization (Anti-Spinner Engine).
        System już nie "ślepo" wyrzuca zdarzenia przy zmianie URL, tylko czeka aż
        DOM aplikacji React/Angular/Gmail przestanie "drgać" (np. skończy kręcić spinnerem).
        """
        last_fp = None
        candidate_fp = None
        stable_cycles = 0
        
        while not page.is_closed():
            try:
                # Poller sprawdza stan matrycy co 1 sekundę
                await asyncio.sleep(1.0)
                
                url = page.url or ""
                if not url or url.startswith("chrome://") or url == "about:blank":
                    continue

                title = await page.title()
                screen_text = await self.extract_semantic_text(page)
                
                if len(screen_text) < 50:
                    continue

                fp = self.fingerprint(url, title, screen_text)
                
                # Maszyna stanów dla stabilizacji widoku:
                if fp != last_fp:
                    if fp == candidate_fp:
                        stable_cycles += 1
                        # Jeśli widok nie zmienił się przez 2 sekundy (2 cykle) -> Załadowano!
                        if stable_cycles >= 2:
                            if not self.fingerprint_cache.seen_recently(fp):
                                self.emit_observation(url, title, screen_text, fp)
                            # Zapisujemy nowy, potwierdzony stan główny
                            last_fp = fp 
                    else:
                        # Właśnie coś kliknąłeś (URL się zmienił lub wszedł nowy e-mail w tle)
                        candidate_fp = fp
                        stable_cycles = 1
                else:
                    # Strona pozostaje w znanym, już wyemitowanym stanie
                    candidate_fp = None
                    stable_cycles = 0
                    
            except PlaywrightError:
                # Karta zamknięta przez użytkownika
                break
            except Exception as e:
                logger.debug("Błąd w pętli sensorycznej: %s", e)

    async def setup_page_listeners(self, page: Page) -> None:
        """Rejestruje nową stronę i uruchamia silnik ciągłego skanowania."""
        if page in self.tracked_pages or page.is_closed():
            return
        
        self.tracked_pages.add(page)
        logger.debug("Podłączono zmysły do: %s", page.url)
        
        # Pasywny silnik działa asynchronicznie w tle dla każdej karty
        asyncio.create_task(self.sensory_loop(page))

    async def monitor_contexts(self, browser) -> None:
        """Dynamicznie asymiluje nowe okna i zakładki otwierane przez użytkownika."""
        logger.info("📡 Podłączono do kory mózgowej przeglądarki. Nasłuchuję...")
        while True:
            try:
                for context in browser.contexts:
                    context.on("page", lambda p: asyncio.create_task(self.setup_page_listeners(p)))
                    for page in context.pages:
                        await self.setup_page_listeners(page)
            except Exception as e:
                logger.error("Błąd synchronizacji kontekstów CDP: %s", e)
                
            await asyncio.sleep(5)

    async def run(self) -> None:
        logger.info("🔭 Warstwa 0: Observer Bridge v5.4 (DOM Stabilization Engine) ONLINE")
        logger.info("📂 Cel zrzutu: %s", self.config.observations_file)
        logger.info("🌐 CDP target: %s", self.config.cdp_url)

        async with async_playwright() as p:
            retry_count = 0
            while True: # Auto-Heal Loop
                try:
                    browser = await p.chromium.connect_over_cdp(self.config.cdp_url)
                    retry_count = 0
                    await self.monitor_contexts(browser)
                except Exception as e:
                    retry_count += 1
                    wait_time = min(2 ** retry_count, 60)
                    logger.error("Błąd połączenia CDP. Przeglądarka wyłączona? Ponawiam za %ds... (%s)", wait_time, str(e)[:100])
                    await asyncio.sleep(wait_time)


def main() -> None:
    parser = argparse.ArgumentParser(description="OODA v5.4 - Observer Bridge (Warstwa 0 SoTA)")
    parser.add_argument("--workspace", required=True, help="Ścieżka do WORKSPACE_DIR wygenerowanego przez bootstrap")
    parser.add_argument("--cdp-url", default=os.environ.get("OBSERVER_CDP", "http://127.0.0.1:9222"), help="Adres Chrome CDP")
    args = parser.parse_args()

    workspace_path = Path(args.workspace).resolve()
    
    config = BridgeConfig(workspace_dir=workspace_path, cdp_url=args.cdp_url)
    bridge = ObserverBridge(config)
    
    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        logger.info("Observer Bridge zatrzymany ręcznie.")

if __name__ == "__main__":
    main()