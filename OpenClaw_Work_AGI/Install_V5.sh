#!/usr/bin/env bash
set -Eeuo pipefail
IFS=$'\n\t'

# ==============================================================================
# OODA V5.3 SOVEREIGN AGI KERNEL — INSTALLER (STATE OF ART 2026)
# Zoptymalizowany pod strukturę hybrydową (SQLite + FAISS) i Swarm Gating
# Zaktualizowany o instalację Playwright dla Observer_Bridge (Warstwa 0)
# ==============================================================================

# ------------------------------ USTAWIENIA DOMYŚLNE ----------------------------
WORKSPACE_DIR="${WORKSPACE_DIR:-$PWD/ats_v5}"

# Struktura workspace (Dostosowana do Bootstrap_V5.py)
VENV_DIR="$WORKSPACE_DIR/.venv"
BIN_DIR="$WORKSPACE_DIR/bin"
LOG_DIR="$WORKSPACE_DIR/logs"
INBOX_DIR="$WORKSPACE_DIR/inbox"
DOCS_DIR="$WORKSPACE_DIR/dokumenty"
POLICY_DIR="$WORKSPACE_DIR/self-improving"
STATE_DIR="$WORKSPACE_DIR/.state"
VECTOR_DIR="$WORKSPACE_DIR/vectorstore"
MODELS_DIR="$WORKSPACE_DIR/models"

ENV_FILE="$WORKSPACE_DIR/.env"
COGNITION_LOG="$LOG_DIR/cognition_stream.jsonl"
WA_AUDIT_LOG="$LOG_DIR/wa_audit.jsonl"
INBOX_FILE="$INBOX_DIR/observations.jsonl"

# Cache'y narzędzi *w* workspace
HF_HOME="$WORKSPACE_DIR/.hf_home"
TRANSFORMERS_CACHE="$HF_HOME/transformers"
SENTENCE_TRANSFORMERS_HOME="$HF_HOME/sentence_transformers"

# LLM provider: 'gemini' lub 'openai_compat' (np. dla lokalnego llama.cpp)
LLM_PROVIDER="${LLM_PROVIDER:-gemini}"

# Gemini
GEMINI_API_KEY="${GEMINI_API_KEY:-XXXXXXXXXXXXXXXXXXXXXXXXX}"    #### <----- API key do wpisania
GEMINI_MODEL_REASON="${GEMINI_MODEL_REASON:-gemini-3-flash-preview}"

# OpenAI-compat / llama.cpp
LLAMA_BACKEND="${LLAMA_BACKEND:-external}"
LLAMA_SERVER_BIN="${LLAMA_SERVER_BIN:-llama-server}"
LLAMA_MODEL_PATH="${LLAMA_MODEL_PATH:-$MODELS_DIR/Qwen-3_5-27B-Reasoning-Distilled-i1-Q3_K_M.gguf}"
LLAMA_SERVER_HOST="${LLAMA_SERVER_HOST:-127.0.0.1}"
LLAMA_SERVER_PORT="${LLAMA_SERVER_PORT:-8080}"
LLAMA_CTX="${LLAMA_CTX:-16384}"
LLAMA_THREADS="${LLAMA_THREADS:-$(nproc)}"

# Vector embeddings (FAISS)
EMBED_MODEL_NAME="${EMBED_MODEL_NAME:-BAAI/bge-m3}"

# WhatsApp / OpenClaw
WA_TARGET_NUMBER="${WA_TARGET_NUMBER:-+48XXXXXXXXXX}"            #### <----- nr telefonu do wpisania
WA_RATE_PER_MIN="${WA_RATE_PER_MIN:-6}"
AUTO_LINK_WHATSAPP="${AUTO_LINK_WHATSAPP:-1}"

# Parametry agenta V5
HEARTBEAT_EVERY_MIN="${HEARTBEAT_EVERY_MIN:-15}"
SWARM_MAX_RETRIES="${SWARM_MAX_RETRIES:-2}"
IDLE_MINUTES_TRIGGER="${IDLE_MINUTES_TRIGGER:-10.0}"
DEDUP_TTL_SECONDS="${DEDUP_TTL_SECONDS:-3600}"

# ------------------------------ LOGGING & UTILS --------------------------------
CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'
log()      { printf "$1%s${NC}\n" "$2"; }
log_info() { log "${CYAN}${BOLD}[INFO] " "$1"; }
log_ok()   { log "${GREEN}${BOLD}[ OK ] " "$1"; }
log_warn() { log "${YELLOW}${BOLD}[WARN] " "$1"; }
log_fail() { log "${RED}${BOLD}[FAIL] " "$1"; exit 1; }

need_cmd()  { command -v "$1" >/dev/null 2>&1 || log_fail "Brak wymaganej komendy: $1"; }
ensure_dir(){ mkdir -p "$1"; }
port_free() { ! lsof -iTCP:"$1" -sTCP:LISTEN >/dev/null 2>&1; }

# ----------------------------- KONFIGURACJA LLM --------------------------------
check_llm_provider() {
  echo -e "\n${YELLOW}${BOLD}=== KONFIGURACJA SILNIKA KOGNITYWNEGO V5 ===${NC}"
  if [[ "$LLM_PROVIDER" == "openai_compat" || "$LLM_PROVIDER" == "llamacpp" ]]; then
    LLM_PROVIDER="openai_compat" # Normalizacja dla V5
    log_ok "Wybrano 'openai_compat'. Agent będzie używał lokalnego serwera (np. llama.cpp)."
  elif [[ "$LLM_PROVIDER" == "gemini" ]]; then
    log_ok "Wybrano 'gemini' jako silnik chmurowy."
    if [[ -z "$GEMINI_API_KEY" ]]; then
      read -rp "Wklej klucz Gemini API: " input_key
      GEMINI_API_KEY="$input_key"
    fi
  else
    log_fail "Nieznany LLM_PROVIDER: $LLM_PROVIDER. Użyj 'gemini' lub 'openai_compat'."
  fi
  echo "=========================================="
}

# ----------------------------- KATALOGI WORKSPACE ------------------------------
prepare_dirs() {
  for d in "$WORKSPACE_DIR" "$BIN_DIR" "$LOG_DIR" "$INBOX_DIR" \
           "$DOCS_DIR" "$POLICY_DIR" "$STATE_DIR" "$VECTOR_DIR" "$MODELS_DIR" \
           "$HF_HOME" "$TRANSFORMERS_CACHE" "$SENTENCE_TRANSFORMERS_HOME"; do
    ensure_dir "$d"
  done

  : > "$COGNITION_LOG" || true
  : > "$WA_AUDIT_LOG" || true
  log_ok "Struktura workspace gotowa: $WORKSPACE_DIR"
}

# ------------------------------ WYMAGANIA SYSTEMU ------------------------------
check_prereqs() {
  need_cmd python3
  need_cmd node
  need_cmd npm
  need_cmd git

  if [[ "$LLAMA_BACKEND" != "external" && "$LLAMA_BACKEND" != "skip" ]]; then
    need_cmd cmake
    need_cmd make
  fi
}

# -------------------------------- PYTHON (VENV) --------------------------------
bootstrap_python() {
  log_info "Tworzę Python venv: $VENV_DIR"

  if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
      rm -rf "$VENV_DIR" || true
      python3 -m venv "$VENV_DIR"
  fi

  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"
  export HF_HOME TRANSFORMERS_CACHE SENTENCE_TRANSFORMERS_HOME

  python -m pip install --upgrade pip wheel setuptools >/dev/null

  # Zależności ściśle dopasowane do kodu Bootstrap_V5.py oraz Observer_Bridge_v5.py
  PKG=(
    "pydantic>=2"
    "pydantic-settings>=2"
    "requests"
    "faiss-cpu"
    "sentence-transformers>=3"
    "numpy"
    "playwright"
  )

  echo -e "${CYAN}${BOLD}[INFO] Pobieram biblioteki AI (Pydantic, Faiss, S-Transformers, Playwright). To potrwa kilka minut...${NC}"
  python -m pip install "${PKG[@]}"

  # Instalacja binarek przeglądarki dla Playwrighta (Wymagane dla Observer Bridge)
  echo -e "${CYAN}${BOLD}[INFO] Instaluję Chromium dla Playwright (Observer Bridge)...${NC}"
  python -m playwright install chromium

  deactivate
  log_ok "Python venv gotowy. Środowisko przygotowane pod rdzeń V5."
}

# ---------------------------- POBRANIE MODELU WEKTOROWEGO ----------------------
preload_embeddings() {
  log_info "Preload modelu wektorowego: $EMBED_MODEL_NAME..."
  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"
  export HF_HOME TRANSFORMERS_CACHE SENTENCE_TRANSFORMERS_HOME
  python - <<'PY'
import os
from sentence_transformers import SentenceTransformer
name = os.environ.get("EMBED_MODEL_NAME", "BAAI/bge-m3")
SentenceTransformer(name)
print(f"[OK] Prefetched {name}")
PY
  deactivate
  log_ok "Embedding zbuforowany."
}

# ---------------------------- OPENCLAW / WA ------------------------------------
ensure_openclaw_tools() {
  if ! command -v openclaw >/dev/null 2>&1; then
    log_info "Instalacja OpenClaw przez npm (wymagane dla tools/wa_client.py)..."
    npm install -g openclaw@latest >/dev/null
  fi
}

configure_whatsapp() {
  log_info "Konfiguruję bazę WhatsApp dla OpenClaw..."
  openclaw doctor </dev/null >/dev/null || true
  if ! openclaw plugins list 2>/dev/null | grep -Eq '@openclaw/whatsapp|whatsapp'; then
    openclaw plugins install @openclaw/whatsapp >/dev/null || true
  fi

  openclaw config set channels.whatsapp.enabled true >/dev/null || true

  if [[ "$AUTO_LINK_WHATSAPP" == "1" ]]; then
    openclaw channels login --channel whatsapp || true
  fi
  log_ok "Środowisko WhatsApp gotowe (agent V5 steruje nim via subprocess)."
}

# --------------------------------- LLAMA.CPP (Opcjonalnie) ---------------------
install_llamacpp() {
  if [[ "$LLAMA_BACKEND" == "external" || "$LLAMA_BACKEND" == "skip" ]]; then
    log_info "LLAMA_BACKEND=${LLAMA_BACKEND}. Pomijam budowanie llama.cpp."
    return 0
  fi

  log_info "Buduję llama.cpp (server) lokalnie..."
  ( cd "$WORKSPACE_DIR"
    [[ -d llama.cpp ]] || git clone https://github.com/ggerganov/llama.cpp.git >/dev/null
    cd llama.cpp && mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release .. >/dev/null
    make -j"$(nproc)" llama-server >/dev/null
    cp bin/llama-server "$LLAMA_SERVER_BIN" 2>/dev/null || cp llama-server "$LLAMA_SERVER_BIN" 2>/dev/null
  )
}

# ----------------------------------- .ENV --------------------------------------
write_workspace_env() {
  log_info "Zapisuję konfigurację do .env pod pydantic-settings (V5)..."
  cat > "$ENV_FILE" <<EOF
# ========================================
# OODA V5.3 Workspace .env (SoA 2026)
# ========================================

# --- Backend Kognitywny ---
LLM_PROVIDER=${LLM_PROVIDER}

# --- Gemini API (jeśli LLM_PROVIDER=gemini) ---
GEMINI_API_KEY=${GEMINI_API_KEY}
GEMINI_MODEL_REASON=${GEMINI_MODEL_REASON}

# --- OpenAI-compat API (jeśli LLM_PROVIDER=openai_compat) ---
OPENAI_BASE_URL=http://${LLAMA_SERVER_HOST}:${LLAMA_SERVER_PORT}
OPENAI_API_KEY=sk-local

# --- Vector Store ---
EMBED_MODEL=${EMBED_MODEL_NAME}

# --- WhatsApp Tool ---
WA_TARGET_NUMBER=${WA_TARGET_NUMBER}
WA_RATE_PER_MIN=${WA_RATE_PER_MIN}

# --- Agent Tunables ---
HEARTBEAT_EVERY_MIN=${HEARTBEAT_EVERY_MIN}
SWARM_MAX_RETRIES=${SWARM_MAX_RETRIES}
IDLE_MINUTES_TRIGGER=${IDLE_MINUTES_TRIGGER}
DEDUP_TTL_SECONDS=${DEDUP_TTL_SECONDS}

# --- Paths ---
WORKSPACE_DIR=${WORKSPACE_DIR}
EOF
  log_ok "Utworzono plik .env zgodny z obiektem Settings (config.py)."
}

# -------------------------------------- MAIN -----------------------------------
main() {
  clear
  log_info "Start instalacji środowiska (OODA V5.3 - Sovereign AGI Kernel)"
  check_llm_provider
  prepare_dirs
  check_prereqs
  bootstrap_python
  preload_embeddings
  ensure_openclaw_tools
  configure_whatsapp
  install_llamacpp
  write_workspace_env

  echo
  printf "${GREEN}${BOLD}INSTALLER V5 — ZAKOŃCZONY SUKCESEM${NC}\n"
  echo "Workspace        : $WORKSPACE_DIR"
  echo "Dostawca AI      : $LLM_PROVIDER"
  echo "--------------------------------------------------------"
  echo -e "${CYAN}Teraz przenieś plik Bootstrap_V5.py do $WORKSPACE_DIR i uruchom:${NC}"
  echo -e "  source $VENV_DIR/bin/activate"
  echo -e "  python Bootstrap_V5.py --workspace $WORKSPACE_DIR"
  echo -e "  ./Start_V5.sh up"
}

main "$@"
