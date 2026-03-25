# OODA V5.3 - Sovereign AGI Kernel

System OODA V5.3 to zaawansowany silnik AGI operujący w pętli Observe-Orient-Decide-Act, zintegrowany z WhatsApp oraz systemem wizyjnym (Observer).

## Instrukcja uruchomienia

### 1. Przygotowanie OpenClaw
OODA wykorzystuje OpenClaw Gateway do komunikacji z WhatsApp.
```bash
# Instalacja OpenClaw
npm install -g openclaw@latest

# Połączenie z WhatsApp (zeskanuj kod QR)
openclaw channels login --channel whatsapp
```

### 2. Instalacja środowiska
Uruchom skrypt instalacyjny, który przygotuje Python venv i zainstaluje zależności.
```bash
chmod +x Install_V5.sh
./Install_V5.sh
```
**Ważne:** Podczas instalacji zostaniesz poproszony o klucz **Gemini API**. Możesz go również później edytować w pliku `ats_v5/.env`. Upewnij się, że w tym samym pliku pod `WA_TARGET_NUMBER` znajduje się Twój numer telefonu.

### 3. Przygotowanie modułów
Przenieś mostek wizyjny do folderu źródłowego projektu:
```bash
mkdir -p ats_v5/src
cp Observer_Bridge_v5.py ats_v5/src/
```
### 3. Uruchomienie podglądu w przeglądarce
Zakładająć, żę wykorzystujesz google chrome stwórz nową instancje dla przeglądarki która będzie obserwowana przez AGI
```bash
google-chrome \ --remote-debugging-port=9222 \ --user-data-dir="$HOME/.openclaw-chrome-profile" \
 --no-first-run \ --no-default-browser-check \
 >/tmp/openclaw-chrome.log 2>&1 &
```
### 4. Uruchomienie systemu
Użyj launchera do startu wszystkich komponentów:
```bash
chmod +x Start_V5.sh
./Start_V5.sh up
```

### Przydatne komendy:
- `./Start_V5.sh status` - Sprawdzenie stanu węzłów.
- `./Start_V5.sh logs` - Podgląd logów w czasie rzeczywistym.
- `./Start_V5.sh down` - Zatrzymanie całego systemu.
