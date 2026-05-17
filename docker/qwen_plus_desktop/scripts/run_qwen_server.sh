#!/usr/bin/env bash
set -e

SERVER_BIN="${LLAMA_SERVER_BIN:-/opt/llama.cpp/build/bin/llama-server}"
if [[ ! -x "${SERVER_BIN}" ]]; then
  SERVER_BIN="/opt/llama.cpp/build/llama-server"
fi

MODEL="${QWEN_MODEL:-/models/Qwen3-VL-4B-Instruct-Q4_K_M.gguf}"
MMPROJ="${QWEN_MMPROJ:-/models/Qwen3-VL-4B-Instruct-mmproj.gguf}"

exec "${SERVER_BIN}" \
  -m "${MODEL}" \
  --mmproj "${MMPROJ}" \
  --host 0.0.0.0 \
  --port "${QWEN_PORT:-8081}" \
  --ctx-size "${QWEN_CTX_SIZE:-1024}" \
  --gpu-layers "${QWEN_GPU_LAYERS:-0}" \
  --no-mmproj-offload \
  --jinja \
  --reasoning off
