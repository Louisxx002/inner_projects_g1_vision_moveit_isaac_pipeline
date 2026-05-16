#!/usr/bin/env bash
set -euo pipefail

echo "[1/5] nvidia-smi"
nvidia-smi >/dev/null
echo "  ok"

echo "[2/5] docker version"
docker --version

echo "[3/5] nvidia-container-toolkit packages"
if dpkg -l nvidia-container-toolkit nvidia-container-toolkit-base libnvidia-container1 libnvidia-container-tools >/dev/null 2>&1; then
  dpkg -l nvidia-container-toolkit nvidia-container-toolkit-base libnvidia-container1 libnvidia-container-tools
else
  echo "  missing NVIDIA Container Toolkit packages"
fi

echo "[4/5] nvidia-ctk"
if command -v nvidia-ctk >/dev/null 2>&1; then
  nvidia-ctk --version
else
  echo "  missing nvidia-ctk"
fi

echo "[5/5] docker GPU test"
IMAGE="${ISAACSIM_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
docker run --rm --gpus all --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}" \
  -e NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}" \
  -e __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}" \
  "${IMAGE}" \
  -lc nvidia-smi
