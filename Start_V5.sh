#!/usr/bin/env bash
# ==============================================================================
# start_v5.sh — OODA V5.3 Sovereign Launcher (State of Art 2026)
# - Up/Down/Status/Logs/Supervise/Doctor for:
#   Observer Bridge → WhatsApp Ingestor → OODA Kernel
# - LLM autodetect: gemini | openai_compat (external/local llama-server)
# - Workspace-scoped env, PID files, structured logs
# - WhatsApp auto-link (OpenClaw) + Gateway health checks
# ==============================================================================
set -Eeuo pipefail
IFS=$'\n\t'

# ----------------------------- CONSTANTS & COLORS -----------------------------
C_BOLD='\033[1m'; C_RST='\033[0m'
C_CYAN='\033[1;36m'; C_GRN='\033[1;32m'; C_YEL='\033[1;33m'; C_RED='\033[1;31m'; C_PRP='\033[1;35m'; C_BLU='\033[1;34m'
log()      { printf "${3}${C_BOLD}[%s]${C_RST} %s\n" "$1" "$2"; }
log_info() { log INFO  "$1" "$C_CYAN"; }
log_ok()   { log OK    "$1" "$C_GRN"; }
log_warn() { log WARN  "$1" "$C_YEL"; }
log_fail() { log FAIL  "$1" "$C_RED"; exit 1; }
log_phase(){ log PHASE "$1" "$C_PRP"; }

need_cmd(){ command -v "$1" >/dev/null 2>&1 || log_fail "Brak wymaganej komendy: $1"; }
port_ready(){ local h="$1" p="$2" t=${3:-40}; for ((i=0;i<t;i++)); do (echo > "/dev/tcp/${h}/${p}" ) >/dev/null 2>&1 && return 0 || true; sleep 0.25; done; return 1; }

is_pid_running(){ local p="$1"; [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null; }
read_pid_file(){ local f="$1"; [[ -f "$f" ]] && cat "$f" || true; }
stop_if_running(){ local name="$1" pidfile="$2" p; p="$(read_pid_file "$pidfile")"; if [[ -n "${p:-}" ]] && is_pid_running "$p"; then log_warn "Zatrzymuję [$name] (PID=$p)"; kill "$p" 2>/dev/null || true; sleep 1; is_pid_running "$p" && { log_warn "[$name] uparty — SIGKILL"; kill -9 "$p" 2>/dev/null || true; }; fi; rm -f "$pidfile"; }

# ----------------------------- WORKSPACE CONTEXT ------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

# AUTO-WYKRYWANIE: Jeśli uruchamiasz skrypt z folderu nadrzędnego (gdzie nie ma .venv, ale jest w podfolderze ats_v5)
if [[ ! -d "$WORKSPACE_DIR/.venv" ]] && [[ -d "$WORKSPACE_DIR/ats_v5/.venv" ]]; then
    log_warn "Wykryto uruchomienie z katalogu nadrzędnego. Automatycznie przełączam workspace na: $WORKSPACE_DIR/ats_v5"
    WORKSPACE_DIR="$WORKSPACE_DIR/ats_v5"
fi

VENV_DIR="${VENV_DIR:-$WORKSPACE_DIR/.venv}"
SRC_DIR="${SRC_DIR:-$WORKSPACE_DIR/src}"
LOG_DIR="${LOG_DIR:-$WORKSPACE_DIR/logs}"
RUN_DIR="${RUN_DIR:-$WORKSPACE_DIR/.run}"
INBOX_FILE="$WORKSPACE_DIR/inbox/observations.jsonl"
mkdir -p "$WORKSPACE_DIR" "$LOG_DIR" "$RUN_DIR" "$WORKSPACE_DIR/inbox" "$LOG_DIR/run"

OODA_LOG="$LOG_DIR/run/ooda.out"; WA_INGESTOR_LOG="$LOG_DIR/run/wa_ingestor.out"; SUPERVISOR_LOG="$LOG_DIR/run/supervise.out"; OBSERVER_LOG="$LOG_DIR/run/observer.out"
OODA_PID="$RUN_DIR/ooda.pid"; WA_INGESTOR_PID="$RUN_DIR/wa_ingestor.pid"; LLM_PID="$RUN_DIR/llama.pid"; OBSERVER_PID="$RUN_DIR/observer.pid"

ENV_FILE="$WORKSPACE_DIR/.env"
if [[ -f "$ENV_FILE" ]]; then set -a; # ensure Settings() picks it up
  export ENV_FILE
  # shellcheck disable=SC1090
  source "$ENV_FILE"; set +a
fi

# WhatsApp defaults
export AUTO_LINK_WHATSAPP="${AUTO_LINK_WHATSAPP:-0}"

# LLM options
LLM_PROVIDER="${LLM_PROVIDER:-gemini}"
LLAMA_BACKEND="${LLAMA_BACKEND:-external}"
LLAMA_SERVER_HOST="${LLAMA_SERVER_HOST:-127.0.0.1}"
LLAMA_SERVER_PORT="${LLAMA_SERVER_PORT:-8080}"
OPENAI_BASE_URL="${OPENAI_BASE_URL:-http://${LLAMA_SERVER_HOST}:${LLAMA_SERVER_PORT}}"

# ------------------------------ PREP ENV --------------------------------------
prep_env(){
  log_phase "Przygotowanie środowiska (V5)"
  cd "$WORKSPACE_DIR" || log_fail "Nie można przejść do $WORKSPACE_DIR"

  [[ -d "$VENV_DIR" ]] || log_fail "Brak venv: $VENV_DIR (najpierw uruchom install_v5.sh)"
  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"

  # Zabezpieczenie ścieżek dla Pythona
  export PYTHONPATH="$WORKSPACE_DIR:$SRC_DIR:${PYTHONPATH:-}"

  # Uciszenie nieistotnych ostrzeżeń (m.in. HF_TOKEN)
  export HF_HUB_DISABLE_TELEMETRY=1
  export HF_HUB_DISABLE_SYMLINKS_WARNING=1
  export TOKENIZERS_PARALLELISM=false

  need_cmd python
  need_cmd nohup
  need_cmd curl

  if [[ "$LLM_PROVIDER" == "gemini" ]]; then
    [[ -n "${GEMINI_API_KEY:-}" ]] || log_fail "Brak GEMINI_API_KEY w .env"
    log_ok "Dostawca: Gemini (klucz obecny)"
  elif [[ "$LLM_PROVIDER" == "openai_compat" || "$LLM_PROVIDER" == "llamacpp" ]]; then
    port_ready "$LLAMA_SERVER_HOST" "$LLAMA_SERVER_PORT" 10 || log_warn "llama-server pod ${LLAMA_SERVER_HOST}:${LLAMA_SERVER_PORT} nie odpowiada (kontynuuję, może to external)"
    log_ok "Dostawca: OpenAI Compat / Llama.cpp"
  else
    log_warn "Nieznany LLM_PROVIDER=$LLM_PROVIDER"
  fi
}

# ------------------------------ LLM (optional) --------------------------------
start_llm(){
  [[ "$LLM_PROVIDER" == "openai_compat" || "$LLM_PROVIDER" == "llamacpp" ]] || return 0
  if [[ "$LLAMA_BACKEND" == "external" || "$LLAMA_BACKEND" == "skip" ]]; then
    log_info "LLM: używam zewnętrznego serwera LLM pod ${OPENAI_BASE_URL}"
    return 0
  fi
  [[ -n "${LLAMA_SERVER_BIN:-}" ]] || LLAMA_SERVER_BIN=llama-server
  [[ -x "$(command -v "$LLAMA_SERVER_BIN" || true)" ]] || log_fail "Brak wykonywalnego llama-server w PATH"
  [[ -f "${LLAMA_MODEL_PATH:-}" ]] || log_fail "Brak modelu GGUF: $LLAMA_MODEL_PATH"
  stop_if_running "llama" "$LLM_PID"
  log_phase "Uruchamiam lokalny llama-server"

  cd "$WORKSPACE_DIR" || true
  nohup "$LLAMA_SERVER_BIN" -m "$LLAMA_MODEL_PATH" \
    -c "${LLAMA_CTX:-16384}" -b "${LLAMA_BATCH:-512}" -t "${LLAMA_THREADS:-$(nproc)}" \
    --host "$LLAMA_SERVER_HOST" --port "$LLAMA_SERVER_PORT" \
    >>"$LOG_DIR/run/llama.out" 2>&1 & echo $! > "$LLM_PID"
  sleep 2; is_pid_running "$(cat "$LLM_PID")" && log_ok "llama-server online (PID=$(cat "$LLM_PID"))" || log_fail "llama-server nie wstał (sprawdź logs/run/llama.out)"
}

# ------------------------------ OBSERVER BRIDGE -------------------------------
start_observer(){
  stop_if_running "observer" "$OBSERVER_PID"
  log_phase "Start Observer Bridge (Warstwa 0)"

  local OBS_SCRIPT="$SRC_DIR/Observer_Bridge_v5.py"
  if [[ ! -f "$OBS_SCRIPT" ]]; then
      log_warn "Brak pliku $OBS_SCRIPT. Upewnij się, że przeniosłeś Observer_Bridge_v5.py do $SRC_DIR. Pomijam uruchomienie."
      return 0
  fi

  cd "$WORKSPACE_DIR" || true
  nohup env PYTHONPATH="$WORKSPACE_DIR:$SRC_DIR:${PYTHONPATH:-}" python "$OBS_SCRIPT" --workspace "$WORKSPACE_DIR" \
      >>"$OBSERVER_LOG" 2>&1 & echo $! > "$OBSERVER_PID"
  sleep 1; is_pid_running "$(cat "$OBSERVER_PID")" && log_ok "Observer online (PID=$(cat "$OBSERVER_PID"))" || log_warn "Observer nie wstał. Sprawdź logs/run/observer.out"
}

# ------------------------------ WHATSAPP --------------------------------------
ensure_gateway(){
  command -v openclaw >/dev/null 2>&1 || { log_warn "Brak openclaw w PATH — pomijam gateway"; return 0; }
  local st
  st="$(openclaw gateway status 2>/dev/null || true)"
  if grep -q "Runtime: running" <<<"$st"; then
    log_ok "OpenClaw Gateway: running"
  else
    log_warn "OpenClaw Gateway nieaktywny — próbuję uruchomić"
    openclaw gateway start >/dev/null 2>&1 || true
    sleep 2
    st="$(openclaw gateway status 2>/dev/null || true)"
    grep -q "Runtime: running" <<<"$st" && log_ok "Gateway uruchomiony" || log_warn "Gateway nie wstał (może wymagać terminala)"
  fi
}

maybe_link_whatsapp(){
  command -v openclaw >/dev/null 2>&1 || return 0
  local ch
  ch="$(openclaw channels status --probe 2>/dev/null || true)"
  if grep -Eiq 'whatsapp.*(linked|running|connected|ok)' <<<"$ch"; then
    log_ok "WhatsApp linked i aktywny"
  elif [[ "${AUTO_LINK_WHATSAPP}" == "1" ]]; then
    log_warn "WhatsApp wymaga parowania — wywołuję login QR"
    openclaw channels login --channel whatsapp || true
  fi
}

start_wa_ingestor(){
  command -v openclaw >/dev/null 2>&1 || { log_info "Brak OpenClaw — pomijam WA Ingestor"; return 0; }
  ensure_gateway
  maybe_link_whatsapp
  stop_if_running "wa_ingestor" "$WA_INGESTOR_PID"
  log_phase "Start WhatsApp Ingestor (V5)"

  cd "$WORKSPACE_DIR" || true
  nohup env PYTHONPATH="$WORKSPACE_DIR:$SRC_DIR:${PYTHONPATH:-}" python "$WORKSPACE_DIR/channels/whatsapp_ingestor.py" --workspace "$WORKSPACE_DIR" \
      >>"$WA_INGESTOR_LOG" 2>&1 & echo $! > "$WA_INGESTOR_PID"
  sleep 1; is_pid_running "$(cat "$WA_INGESTOR_PID")" && log_ok "WA Ingestor online (PID=$(cat "$WA_INGESTOR_PID"))" || log_warn "WA Ingestor nie wstał"
}

# ------------------------------ OODA ------------------------------------------
start_ooda(){
  log_phase "Start OODA V5.3 Kernel (System-2)"
  stop_if_running "ooda" "$OODA_PID"

  cd "$WORKSPACE_DIR" || true
  nohup env PYTHONPATH="$WORKSPACE_DIR:$SRC_DIR:${PYTHONPATH:-}" python -m ooda_v5.ooda_v5 \
      >>"$OODA_LOG" 2>&1 & echo $! > "$OODA_PID"
  sleep 1; is_pid_running "$(cat "$OODA_PID")" && log_ok "OODA Kernel online (PID=$(cat "$OODA_PID"))" || log_fail "OODA nie wstał. Sprawdź logs/run/ooda.out"
}

# ------------------------------ STATUS / LOGS ---------------------------------
status(){
  echo -e "${C_PRP}${C_BOLD}>>> STAN WĘZŁÓW (V5) <<<${C_RST}"
  for N in llama wa_ingestor observer ooda; do
    case $N in
      llama) F="$LLM_PID";;
      wa_ingestor) F="$WA_INGESTOR_PID";;
      observer) F="$OBSERVER_PID";;
      ooda) F="$OODA_PID";;
    esac
    if [[ -f "$F" ]]; then P="$(cat "$F")"; is_pid_running "$P" && echo -e "${C_GRN}[ ONLINE ]${C_RST} $N (PID=$P)" || echo -e "${C_RED}[ CRASH ]${C_RST} $N (PID orphan)"; else echo -e "${C_YEL}[ OFF   ]${C_RST} $N"; fi
  done
}

logs(){
  tail -n 200 -f "$WA_INGESTOR_LOG" "$OBSERVER_LOG" "$OODA_LOG"
}

# ------------------------------ DOWN / STOP -----------------------------------
stop_all(){
  echo -e "${C_YEL}${C_BOLD}>>> WYGASZANIE STACKU V5 <<<${C_RST}"
  for N in wa_ingestor observer ooda llama; do
    case $N in
      wa_ingestor) F="$WA_INGESTOR_PID";;
      observer) F="$OBSERVER_PID";;
      ooda) F="$OODA_PID";;
      llama) F="$LLM_PID";;
    esac
    [[ -f "$F" ]] || continue
    P="$(cat "$F")"; if is_pid_running "$P"; then kill "$P" 2>/dev/null || true; sleep 1; is_pid_running "$P" && kill -9 "$P" 2>/dev/null || true; log_ok "Stop: $N"; fi
    rm -f "$F"
  done
}

# ------------------------------ SUPERVISE LOOP --------------------------------
supervise(){
  log_phase "Tryb supervise — auto-restart komponentów"
  while true; do
    [[ -f "$OODA_PID" ]] && P="$(cat "$OODA_PID")" && is_pid_running "$P" || start_ooda
    [[ -f "$WA_INGESTOR_PID" ]] && P="$(cat "$WA_INGESTOR_PID")" && is_pid_running "$P" || start_wa_ingestor
    [[ -f "$OBSERVER_PID" ]] && P="$(cat "$OBSERVER_PID")" && is_pid_running "$P" || start_observer
    sleep 3
  done >>"$SUPERVISOR_LOG" 2>&1
}

# ------------------------------ DOCTOR ----------------------------------------
doctor(){
  log_phase "Doctor — V5 Health Check"
  need_cmd python; need_cmd curl
  [[ -f "$ENV_FILE" ]] && log_ok ".env: OK" || log_warn ".env: brak (zostaną użyte domyślne)"
  if [[ "$LLM_PROVIDER" == "gemini" ]]; then [[ -n "${GEMINI_API_KEY:-}" ]] && log_ok "GEMINI_API_KEY: OK" || log_fail "GEMINI_API_KEY: brak"; fi
  if [[ "$LLM_PROVIDER" != "gemini" ]]; then port_ready "$LLAMA_SERVER_HOST" "$LLAMA_SERVER_PORT" 5 && log_ok "llama-server port: OK" || log_warn "llama-server port: closed"; fi
  [[ -f "$INBOX_FILE" ]] && log_ok "Inbox: gotowy ($INBOX_FILE)" || log_warn "Inbox nie istnieje (zostanie utworzony przy pierwszej wiadomości)"

  if command -v openclaw >/dev/null 2>&1; then
      log_ok "OpenClaw CLI: zainstalowane"
  else
      log_warn "OpenClaw CLI: brak (Agent nie wyśle/odbierze WhatsApp)"
  fi
}

# ------------------------------ MAIN ROUTER -----------------------------------
cmd="${1:-up}"
case "$cmd" in
  up)
    prep_env; start_llm; start_observer; start_wa_ingestor; start_ooda; status; log_info "Logi: $LOG_DIR/run" ;;
  down) stop_all ;;
  status) status ;;
  logs) logs ;;
  supervise) prep_env; start_observer; start_wa_ingestor; start_ooda; supervise ;;
  doctor) prep_env; doctor ;;
  *) echo "Użycie: $0 {up|down|status|logs|supervise|doctor}"; exit 2 ;;
esac
