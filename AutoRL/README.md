# AutoRL v1: Autonomiczny Ekosystem Uczenia (System Architecture) 🏗️

## Wstęp: Problem i Rozwiązanie

Tradycyjne uczenie ze wzmocnieniem (RL) jest kruche. Wymaga ręcznego dobierania hiperparametrów, jest wrażliwe na błędy sprzętowe (np. sterowniki GPU) i często kończy się "cichą śmiercią" agenta (loss = 0, brak postępów).

AutoRL v1.0 rozwiązuje te problemy poprzez inwersję kontroli. Zamiast statycznego skryptu, system działa jako autonomiczna pętla sprzężenia zwrotnego, która:
- Sama wykrywa anomalie (system immunologiczny).
- Sama optymalizuje wydajność kodu (kompilacja JIT).
- Sama ewoluuje najlepsze konfiguracje (PBT).

---

## 1. Warstwa Danych: "Zero-Copy Dataflow" ⚡

**Cel:** Eliminacja wąskiego gardła Python GIL podczas interakcji z wieloma środowiskami.

Tradycyjne podejście w Pythonie tworzy tysiące małych obiektów przy każdym kroku symulacji, co dławi Garbage Collector.

**Rozwiązanie (`src/core/vector_runner.py`):** Zastosowano architekturę pre-alokowaną.
- Pamięć RAM na bufory (obs, rew, actions) jest rezerwowana tylko raz przy starcie.
- **Silnik Numba (`src/core/kernels.py`):** Krytyczne operacje zapisu danych i maskowania stanów końcowych (terminated/truncated) są wykonywane przez skompilowane funkcje C++, które piszą bezpośrednio do pamięci, omijając narzut Pythona.

**Efekt:** System osiąga maksymalną przepustowość (Steps Per Second) ograniczoną jedynie przez CPU, a nie przez interpreter języka.

---

## 2. Warstwa Obliczeniowa: Architektura Hybrydowa 🧠

**Cel:** Rozwiązanie problemu "Partial Observability" (niepełna informacja o stanie gry).

Zwykłe sieci (DQN) widzą tylko jedną klatkę. AutoRL v2.3 implementuje pamięć sekwencyjną.

**Model (`src/core/models/drqn_c51.py`):**
- **Enkoder ResMLP:** Głęboka sieć z połączeniami rezydualnymi (jak w ResNet), co pozwala na trenowanie głębszych struktur bez zanikania gradientu.
- **Rdzeń Rekurencyjny (LSTM/GRU):** Utrzymuje "stan umysłu" (hidden state) przez 40-80 kroków czasowych. Pozwala to agentowi rozumieć dynamikę obiektów (np. prędkość, kierunek), a nie tylko ich pozycję.
- **Mechanizm Uwagi (Attention):** Opcjonalny moduł pozwalający modelowi "skupić się" na kluczowych momentach w historii sekwencji.
- **Dystrybucja C51:** Zamiast przewidywać jedną średnią wartość nagrody, sieć przewiduje rozkład prawdopodobieństwa (histogram). Daje to stabilniejszy sygnał uczenia w chaotycznych środowiskach.

---

## 3. Warstwa Stabilności: "Safety Engineering" 🛡️

**Cel:** Zapobieganie awariom treningu bez przerywania procesu.

RL na kartach konsumenckich (szczególnie AMD/ROCm) jest podatne na błędy numeryczne (NaN, Inf).

**Immune System (`src/orchestrators/immune.py`):** Działa jak system PID w automatyce.
- Monitoruje metryki w czasie rzeczywistym używając bufora pierścieniowego w Numbie (`VitalSignsHP`).
- Wykrywa trendy: "Exploding Gradients" (niestabilność) lub "Entropy Collapse" (stagnacja).
- **Interwencje:** Automatycznie modyfikuje Learning Rate lub wstrzykuje szum (entropię) do agenta, działając jak dynamiczny scheduler.

**Gradient Guard (`src/core/trainer.py`):** Jeśli podczas propagacji wstecznej pojawią się wartości NaN (Not a Number), system automatycznie odrzuca wadliwy krok i zmniejsza skalę precyzji (AMP scaler), zamiast wyrzucić błąd i przerwać pracę.

---

## 4. Warstwa Ewolucji: Population Based Training (PBT) 🧬

**Cel:** Automatyczne dostrajanie hiperparametrów (Learning Rate, Gamma) w locie.

Zamiast puszczać jeden trening i modlić się o wynik, uruchamiamy populację 4-24 równoległych procesów.

**Mechanizm (`src/orchestrators/pbt.py`):**
- **Async Execution (`src/run/runners.py`):** Każdy agent działa w pełni asynchronicznie. Nie ma bariery synchronizacji - szybkie agenty nie czekają na wolne.
- **Ewaluacja:** Co określony interwał (np. 30k kroków) system porównuje wyniki agentów.
- **Eksploatacja (NeuralSurgeon):** Słabsze agenty kopiują wagi od mentorów używając inteligentnego "przeszczepu" (`src/orchestrators/knowledge_transfer.py`), który resetuje stan optymalizatora, aby uniknąć konfliktów pędu (momentum mismatch).
- **Eksploracja (Explore):** Sklonowane agenty otrzymują zmutowane hiperparametry.

**Fix na "Pułapkę Stabilności":** Zaimplementowano twarde granice mutacji (`MutationRule` w `config.py`). Zapobiega to ewolucji w kierunku "martwych" agentów (z LR=0), którzy nie popełniają błędów, ale też się nie uczą.

---

## 5. Przepływ Sterowania (Lifecycle) 🔄

Proces działania systemu w uproszczeniu:

1. **Inicjalizacja (`main.py`):** Walidacja konfiguracji (Pydantic), ustawienie seedów, alokacja pamięci.
2. **Zbieranie Danych (`VectorRunner`):** Wykonanie 80 kroków w środowisku. Dane trafiają do bufora RAM (CPU).
3. **Transfer (CPU -> GPU):** Paczka danych (Batch) jest przenoszona do pamięci VRAM.
4. **Trening (`Trainer`):**
    - Obliczenie straty (R2D2 Loss) na sekwencjach.
    - Wymuszenie trybu `.train()` (Fix dla AMD).
    - Obliczenie gradientów i weryfikacja przez Gradient Guard.
5. **Diagnoza (`Immune`):** Sprawdzenie, czy trening jest zdrowy. Ewentualna korekta parametrów.
6. **Ewolucja (`PopulationManager`):** Asynchroniczna weryfikacja i mutacja populacji bez blokowania wątku głównego.

---

## Obsługa

Modyfikacja parametrów odbywać się może przez CLI jak również w momencie uruchamiania kodu. Uruchamia pełny ekosystem z 24 agentami, systemem immunologicznym i ewolucją.

```bash
pip install -e .

conda run -n rl python src/run/main.py --mode population --train.total_steps 1000000 --pbt.population_size 16
```

## Struktura Projektu

```
src/
├── config.py             # ⚙️ Mózg konfiguracji (Pydantic models)
├── main.py               # 🚀 Entrypoint (start systemu)
├── core/
│   ├── agent.py          # Logika decyzyjna i cykle epsilon
│   ├── trainer.py        # Pętla uczenia, obsługa gradientów i AMD Fix
│   ├── vector_runner.py  # Silnik zbierania danych (Zero-Copy)
│   ├── kernels.py        # Kompilowane funkcje Numba (C++)
│   ├── metrics.py        # Szybkie liczenie EVI i n-step targets
│   ├── models/           # Architektury sieci (DRQN, C51, ResMLP)
│   └── replay/           # Bufor sekwencyjny (PerSequenceHP)
├── orchestrators/
│   ├── immune.py         # System odpornościowy (Lekarz)
│   ├── pbt.py            # Logika ewolucji i mutacji
│   ├── nas.py            # Neural Architecture Search
│   └── knowledge_transfer.py # Przeszczepianie wag (Neurochirurgia)
├── envs/                 # Wrappery środowisk (Gymnasium)
└── utils/                # Logowanie, monitoring, narzędzia
```
