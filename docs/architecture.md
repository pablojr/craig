# Architecture Review

## Speech to Speech

Using [huggingface speech-to-speech](https://github.com/huggingface/speech-to-speech) framework to handle speech interaction

### Installation (for CPU only, avoid CUDA components)
python3 -m venv .venv
source .venv/bin/activate
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install "speech-to-speech[faster-whisper,pocket]"

#### Check installation (No CUDA)
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'Device: {torch.device(\"cpu\")}')"
CUDA available: False
Device: cpu

### Run pipeline
OPENAI_API_KEY="unused" uv run speech-to-speech serve \
    --stt faster-whisper \
    --llm_backend chat-completions \
    --responses-api-base-url http://10.0.0.202:11434/v1 \
    --model_name "qwen2.5vl:3b" \
    --tts pocket \
    --pocket_tts_device cpu \
    --host 0.0.0.0 \
    --port 8765
