import asyncio
import logging
import os
import random
import re
import time
from contextlib import asynccontextmanager
from datetime import datetime
from typing import Optional, List, Dict, Any
from urllib.parse import urlparse, quote_plus, parse_qs, unquote
"""ABCABC"""
import cloudscraper
import httpx
from pydantic import BaseModel
from selectolax.parser import HTMLParser
from trafilatura import extract
from playwright.sync_api import sync_playwright, TimeoutError as PlaywrightTimeoutError
from playwright_stealth import Stealth
from Agent_Config import AppConfig

# --- Konfiguracja ---
config = AppConfig()

# --- Ustawienia Logowania ---
logging.basicConfig(level=logging.INFO, format="[%(asctime)s][%(levelname)s] %(message)s")
for logger_name in ["httpx", "selenium", "urllib3", "playwright"]:
    logging.getLogger(logger_name).setLevel(logging.WARNING)
logger = logging.getLogger("ULTIMATE_SCRAPER")

# --- Katalog na Zrzuty Ekranu i Błędy ---
SCREENSHOT_DIR = "screenshots"
ERROR_DIR = "errors"
os.makedirs(SCREENSHOT_DIR, exist_ok=True)
os.makedirs(ERROR_DIR, exist_ok=True)

class ScrapedData:
    def __init__(self, url: str, text: str, source: str):
        self.url = url
        self.text = text
        self.source = source

    def __getitem__(self, key: str) -> Any:
        return getattr(self, key)

class UltimateScraper:
    """
    Finalna, stabilna wersja scrapera z izolacją sesji dla zapytań równoległych.
    Zarządza własnym, odizolowanym klientem HTTP dla każdego zadania `run`.
    """
    def __init__(self, app_config: AppConfig):
        self.config = app_config
        self._semaphore = asyncio.Semaphore(self.config.network.concurrency_limit)
        self.search_engines = [{"name": "DuckDuckGo HTML", "url": "https://html.duckduckgo.com/html/?q={query}", "selector": "a.result__a", "decode": True}]

    async def _rotate_ip(self, client: Optional[httpx.AsyncClient]) -> Optional[httpx.AsyncClient]:
        """Zamyka istniejącego klienta i wysyła sygnał NEWNYM do Tora."""
        if not self.config.tor.use_tor:
            return client

        logger.info("🧅 Wysyłam żądanie nowej tożsamości Tor (NEWNYM)...")
        writer = None
        try:
            reader, writer = await asyncio.open_connection(
                self.config.tor.tor_control_host, self.config.tor.tor_control_port
            )
            auth_cmd = f'AUTHENTICATE "{self.config.tor.tor_control_password}"\r\n'.encode()
            writer.write(auth_cmd)
            await writer.drain()
            if b"250 OK" not in await reader.read(4096):
                logger.error("❌ Błąd autoryzacji w ControlPort Tora.")
                return client

            writer.write(b"SIGNAL NEWNYM\r\n")
            await writer.drain()
            if b"250 OK" in await reader.read(4096):
                logger.info("✅ Nowa tożsamość Tora przyznana. Resetuję sesję klienta.")
                if client and not client.is_closed:
                    await client.aclose()
                return None  # Sygnał do stworzenia nowego klienta
            else:
                logger.error("❌ Nie udało się wysłać sygnału NEWNYM.")
                return client
        except Exception as e:
            logger.error(f"❌ Krytyczny błąd podczas rotacji IP Tora: {e}", exc_info=True)
            return client
        finally:
            if writer:
                writer.close()
                await writer.wait_closed()

    def _get_client(self) -> httpx.AsyncClient:
        """Tworzy i zwraca nową instancję klienta httpx."""
        proxy = self.config.tor.tor_socks_proxy if self.config.tor.use_tor else None
        timeout = httpx.Timeout(30.0, connect=10.0)
        transport = httpx.AsyncHTTPTransport(proxy=proxy, retries=1) if proxy else httpx.AsyncHTTPTransport(retries=1)
        logger.info(f"Zainicjowano nową sesję klienta httpx. Proxy: {proxy}")
        return httpx.AsyncClient(transport=transport, verify=False, timeout=timeout, follow_redirects=True, http2=True)

    def _decode_ddg_url(self, url: str) -> str:
        if "duckduckgo.com/l/" in url:
            try:
                qs = parse_qs(urlparse(url).query)
                if decoded := qs.get('uddg', [''])[0]:
                    return unquote(decoded)
            except Exception:
                return url
        return url

    def _extract_content(self, html: str) -> Optional[str]:
        content = extract(html, include_comments=False, include_tables=True, deduplicate=True)
        if content and len(content) > 100:
            return content
        try:
            parser = HTMLParser(html)
            if (body := parser.body) and (text := body.text(separator=' ', strip=True)) and len(text) > 100:
                return text
        except Exception:
            return None
        return None

    def _fetch_playwright_sync(self, url: str) -> Optional[str]:
        with sync_playwright() as p:
            proxy_settings = {"server": self.config.tor.tor_socks_proxy} if self.config.tor.use_tor else None
            browser = p.chromium.launch(headless=True, proxy=proxy_settings)
            context = browser.new_context(user_agent=random.choice(self.config.network.user_agents))
            page = context.new_page()
            try:
                logger.info(f"Playwright nawiguje do: {url}")
                page.goto(url, timeout=45000, wait_until='domcontentloaded')
                return page.content()
            except Exception as e:
                logger.error(f"Playwright ogólny błąd dla {url}: {e}", exc_info=False) # Zmniejszono szczegółowość logu
                return None
            finally:
                browser.close()

    async def search(self, client: httpx.AsyncClient, query: str) -> List[str]:
        for engine in self.search_engines:
            search_url = engine['url'].format(query=quote_plus(query))
            logger.info(f"Wyszukuję '{query}' używając {engine['name']}...")
            try:
                response = await client.get(search_url, headers={"User-Agent": random.choice(self.config.network.user_agents)})
                if response.status_code == 200:
                    links = {self._decode_ddg_url(node.attributes.get('href')) for node in HTMLParser(response.text).css(engine['selector']) if node.attributes.get('href')}
                    valid_links = [url for url in links if url and url.startswith('http')]
                    if valid_links:
                        logger.info(f"✅ {engine['name']} (httpx) znalazł {len(valid_links)} linków.")
                        return valid_links[:self.config.network.MAX_LINKS_PER_SEARCH]
            except Exception as e:
                logger.warning(f"Wyszukiwanie przez httpx nie powiodło się: {e}")
        return []

    async def fetch(self, client: httpx.AsyncClient, url: str, max_retries: int = 2) -> Optional[Dict[str, Any]]:
        for retry in range(max_retries):
            logger.info(f"Próba fetch {retry + 1}/{max_retries} dla {url}")
            async with self._semaphore:
                # Krok 1: httpx
                try:
                    response = await client.get(url, headers={"User-Agent": random.choice(self.config.network.user_agents)}, timeout=10.0)
                    if response.status_code == 200 and "js-challenge" not in response.text:
                        if content := self._extract_content(response.text):
                            logger.info(f"✅ Sukces (httpx): {url}")
                            return {"url": url, "content": content, "source": "httpx"}
                except (httpx.TimeoutException, httpx.NetworkError, httpx.ProxyError) as e:
                    logger.warning(f"httpx błąd sieci/timeout (10s) dla {url}: {e}.")
                except Exception as e:
                    logger.error(f"httpx napotkał błąd dla {url}: {e}")

                # Krok 2: cloudscraper
                try:
                    loop = asyncio.get_running_loop()
                    scraper = cloudscraper.create_scraper()
                    proxies = {"http": self.config.tor.tor_socks_proxy, "https": self.config.tor.tor_socks_proxy} if self.config.tor.use_tor else None
                    response = await loop.run_in_executor(None, lambda: scraper.get(url, proxies=proxies, timeout=25.0))
                    if response.status_code == 200:
                        if content := self._extract_content(response.text):
                            logger.info(f"✅ Sukces (cloudscraper): {url}")
                            return {"url": url, "content": content, "source": "cloudscraper"}
                except Exception as e:
                    logger.warning(f"Cloudscraper zawiódł (timeout 25s) dla {url}: {e}")

                # Krok 3: Playwright
                try:
                    loop = asyncio.get_running_loop()
                    html = await asyncio.wait_for(loop.run_in_executor(None, self._fetch_playwright_sync, url), timeout=50.0)
                    if html and (content := self._extract_content(html)):
                        logger.info(f"✅ Sukces (Playwright): {url}")
                        return {"url": url, "content": content, "source": "playwright"}
                except asyncio.TimeoutError:
                    logger.error(f"❌ Playwright przekroczył limit czasu 50s dla {url}.")
                except Exception as e:
                    logger.error(f"Playwright napotkał błąd dla {url}: {e}")

            if retry < max_retries - 1:
                logger.warning(f"Wszystkie metody zawiodły dla {url}. Ponawiam próbę z tym samym IP.")
                await asyncio.sleep(1) # Krótka przerwa przed ponowieniem

        logger.critical(f"Pobranie {url} nie powiodło się po {max_retries} próbach.")
        return None

    async def get_public_ip(self, client: httpx.AsyncClient) -> Optional[str]:
        try:
            response = await client.get("https://check.torproject.org/api/ip", timeout=10.0)
            response.raise_for_status()
            data = response.json()
            logger.info(f"Kontrola IP: Twój publiczny adres IP to {data.get('IP')} (Używasz Tora: {data.get('IsTor', False)})")
            return data.get('IP')
        except Exception as e:
            logger.error(f"Błąd podczas kontroli IP: {e}")
            return None

    async def run(self, query: str, max_links: int = 6, failure_threshold: int = 3) -> List[ScrapedData]:
        start_time = time.time()
        client = None
        tasks = [] # Initialize tasks list here
        valid_results = []
        try:
            async with asyncio.timeout(180):
                client = self._get_client()
                await self.get_public_ip(client)
                client = await self._rotate_ip(client)
                if client is None:
                    client = self._get_client()
                links = await self.search(client, query)
                if not links:
                    logger.warning(f"Nie udało się znaleźć żadnych linków dla zapytania: '{query}'.")
                    return []
                
                # --- THIS IS THE FIX ---
                # Create Task objects, which can be cancelled, instead of just coroutines.
                tasks = [asyncio.create_task(self.fetch(client, url)) for url in links[:max_links]]
                
                failed_count = 0
                for future in asyncio.as_completed(tasks):
                    if res := await future:
                        valid_results.append(ScrapedData(url=res['url'], text=res['content'], source=res['source']))
                    else:
                        failed_count += 1
                    if failed_count >= failure_threshold:
                        logger.critical(f"Przekroczono próg błędów ({failed_count}). Zakańczam pobieranie dla '{query}'.")
                        for task in tasks:
                            if not task.done():
                                task.cancel()
                        break
                return valid_results
        except asyncio.TimeoutError:
            logger.critical(f"Przekroczono globalny limit czasu 180s dla zapytania '{query}'. Zwracam {len(valid_results)} wyników.")
            for task in tasks:
                if not task.done():
                    task.cancel()
            return valid_results
        except Exception as e:
            logger.error(f"Nieoczekiwany błąd w `run` dla zapytania '{query}': {e}", exc_info=True)
            for task in tasks:
                if not task.done():
                    task.cancel()
            return valid_results
        finally:
            logger.info(f"--- Zakończono '{query}' w {time.time() - start_time:.2f}s ---")
            if client and not client.is_closed:
                await client.aclose()
                
class WebScraper:
    """
    Orkiestrator, który zarządza klasą UltimateScraper,
    uruchamiając wyszukiwania dla wielu zapytań w sposób SEKWENCYJNY.
    """
    def __init__(self, app_config: AppConfig):
        self.config = app_config
        self.scraper = UltimateScraper(app_config)

    async def __aenter__(self):
        logger.info("Orkiestrator WebScraper: Rozpoczynam pracę.")
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        logger.info("Orkiestrator WebScraper: Zakończono pracę.")

    async def run_searches(self, queries: List[str], max_links_per_query: int = 4) -> List[ScrapedData]:
        """
        Uruchamia proces wyszukiwania dla listy zapytań SEKWENCYJNIE (jedno po drugim),
        aby uniknąć przeciążenia zasobów.
        """
        logger.info(f"Rozpoczynam wyszukiwanie dla {len(queries)} głównych zapytań (przetwarzanie sekwencyjne).")
        
        all_results = []
        for query in queries:
            results_for_query = await self.scraper.run(
                query=query, 
                max_links=max_links_per_query
            )
            all_results.extend(results_for_query)

        # Usunięcie duplikatów na podstawie adresu URL.
        seen_urls = set()
        unique_results = []
        for result in all_results:
            if result.url not in seen_urls:
                seen_urls.add(result.url)
                unique_results.append(result)

        logger.info(f"Zakończono. Zebrano łącznie {len(unique_results)} unikalnych wyników.")
        return unique_results
