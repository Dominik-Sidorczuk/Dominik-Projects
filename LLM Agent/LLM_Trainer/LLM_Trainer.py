# --- Import bibliotek ---
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, TrainingArguments
from datasets import load_dataset
from peft import LoraConfig, get_peft_model
from trl import SFTTrainer
import os

# --- Ustawienie zmiennej środowiskowej, aby uniknąć problemów z tokenizers ---
os.environ["TOKENIZERS_PARALLELISM"] = "false"

# --- POPRAWKA: Automatyczne ustawienie backendu dla ROCm ---
# `accelerate` może błędnie wybierać 'nccl' (NVIDIA) zamiast 'rccl' (AMD).
# Ta sekcja programowo ustawia poprawny backend, jeśli wykryto środowisko ROCm,
# co jest kluczowe dla poprawnego działania treningu rozproszonego na kartach AMD.
if torch.cuda.is_available() and hasattr(torch.version, 'hip') and torch.version.hip is not None:
    print("INFO: Wykryto środowisko ROCm. Ustawianie backendu komunikacyjnego na 'rccl'.")
    os.environ["TORCH_DISTRIBUTED_BACKEND"] = "rccl"

# --- Definicje ---
model_id = "ValiantLabs/Qwen3-4B-ShiningValiant3"
dataset_id = "Josephgflowers/Finance-Instruct-500k"
new_model_name = "Qwen3-4B-Finance-SFT-Final"


# --- Ładowanie tokenizera ---
print("INFO: Ładowanie tokenizera...")
tokenizer = AutoTokenizer.from_pretrained(model_id)
if tokenizer.pad_token is None:
    tokenizer.pad_token = tokenizer.eos_token
if tokenizer.chat_template is None:
    tokenizer.chat_template = "{% for message in messages %}\n{{'<|im_start|>' + message['role'] + '\n' + message['content'] + '<|im_end|>'}}{% endfor %}"

# --- Ładowanie modelu ---
print("INFO: Ładowanie modelu...")
model = AutoModelForCausalLM.from_pretrained(
    model_id,
    torch_dtype=torch.bfloat16,
    trust_remote_code=True,
)

# --- Przygotuj model do treningu LoRA ---
lora_config = LoraConfig(
    r=16,
    lora_alpha=32,
    target_modules=["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"],
    lora_dropout=0.1,
    bias="none",
    task_type="CAUSAL_LM",
)
model = get_peft_model(model, lora_config)
print("INFO: Model przygotowany do treningu z adapterem LoRA.")
model.print_trainable_parameters()


# --- Załaduj i przygotuj zbiór danych ---
print(f"INFO: Ładowanie i formatowanie zbioru danych '{dataset_id}'...")
dataset = load_dataset(dataset_id, split="train")
dataset = dataset.select(range(50000))

def create_prompt(example):
    messages = []
    if 'system' in example and example['system']:
        messages.append({"role": "system", "content": example['system']})
    if 'user' in example and example['user']:
        messages.append({"role": "user", "content": example['user']})
    if 'assistant' in example and example['assistant']:
        messages.append({"role": "assistant", "content": example['assistant']})
    
    return {"text": tokenizer.apply_chat_template(messages, tokenize=False) if len(messages) >= 2 else ""}

dataset = dataset.map(create_prompt, remove_columns=list(dataset.features))
dataset = dataset.filter(lambda example: len(example['text']) > 0)


# --- POPRAWKA: Dynamiczne argumenty treningowe w zależności od liczby procesów ---
# Skrypt automatycznie wykrywa, czy jest uruchamiany na jednym GPU czy w trybie rozproszonym.
world_size = int(os.environ.get("WORLD_SIZE", "1"))

training_args_dict = {
    "output_dir": new_model_name,
    "per_device_train_batch_size": 2,
    "gradient_accumulation_steps": 8,
    "learning_rate": 2e-4,
    "logging_steps": 20,
    "num_train_epochs": 1,
    "save_strategy": "epoch",
    "bf16": True,
}

if world_size > 1:
    print(f"INFO: Wykryto {world_size} procesów. Włączam FSDP dla treningu rozproszonego.")
    training_args_dict["fsdp"] = "full_shard"
    training_args_dict["fsdp_config"] = {
        "fsdp_offload_params": True,
        "fsdp_transformer_layer_cls_to_wrap": ['Qwen2DecoderLayer'],
        "fsdp_activation_checkpointing": "FULL",
    }
else:
    print("INFO: Wykryto 1 proces. Trening zostanie uruchomiony w trybie standardowym (bez FSDP).")
    training_args_dict["gradient_checkpointing"] = True


training_args = TrainingArguments(**training_args_dict)

# --- Stwórz trenera ---
trainer = SFTTrainer(
    model=model,
    train_dataset=dataset,
    peft_config=lora_config,
    args=training_args,
)

# --- Uruchom trening ---
print("INFO: Rozpoczynam trening...")
trainer.train()
print("INFO: Trening zakończony!")
trainer.save_model(new_model_name)
print(f"INFO: Model został zapisany w katalogu: {new_model_name}")

