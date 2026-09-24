#!/usr/bin/env bash
set -euo pipefail

# Add standard tool paths
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"

echo "================================================================="
echo "               AVR-Map Environment Inspection                    "
echo "================================================================="

# Operating System & Architecture
OS_NAME="$(uname -s)"
ARCH_NAME="$(uname -m)"
echo "Operating System : ${OS_NAME} (${ARCH_NAME})"

if [[ "${OS_NAME}" == "Darwin" ]]; then
  PRODUCT_VERSION="$(sw_vers -productVersion 2>/dev/null || echo 'Unknown')"
  echo "macOS Version    : ${PRODUCT_VERSION}"
elif [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  echo "Linux Distro     : ${PRETTY_NAME:-Unknown}"
fi

echo "-----------------------------------------------------------------"
echo "Core C++ & Development Toolchain (Required for M0):"

# Git
if command -v git &>/dev/null; then
  echo "  [OK] Git       : $(git --version)"
else
  echo "  [FAIL] Git     : Not found"
fi

# CMake
if command -v cmake &>/dev/null; then
  echo "  [OK] CMake     : $(cmake --version | head -n 1)"
else
  echo "  [FAIL] CMake   : Not found in PATH"
fi

# C++ Compiler
if command -v clang++ &>/dev/null; then
  echo "  [OK] Compiler  : $(clang++ --version | head -n 1)"
elif command -v g++ &>/dev/null; then
  echo "  [OK] Compiler  : $(g++ --version | head -n 1)"
else
  echo "  [FAIL] Compiler: No C++ compiler (clang++ or g++) found"
fi

echo "-----------------------------------------------------------------"
echo "Interpreter & Web Runtimes:"

# Python
if command -v python3 &>/dev/null; then
  echo "  [OK] Python    : $(python3 --version)"
else
  echo "  [WARN] Python  : python3 not found"
fi

# Node.js
if command -v node &>/dev/null; then
  echo "  [OK] Node.js   : $(node --version)"
else
  echo "  [WARN] Node.js : node not found"
fi

# npm
if command -v npm &>/dev/null; then
  echo "  [OK] npm       : $(npm --version)"
else
  echo "  [WARN] npm     : npm not found"
fi

echo "-----------------------------------------------------------------"
echo "Robotics & Hardware Runtimes (Canonical Ubuntu / Required Later):"

# ROS 2
if command -v ros2 &>/dev/null; then
  echo "  [OK] ROS 2     : $(ros2 --version 2>/dev/null || echo 'Installed')"
else
  echo "  [INFO] ROS 2   : NOT REQUIRED FOR M0 (Ubuntu 24.04 target)"
fi

# Docker
if command -v docker &>/dev/null; then
  echo "  [OK] Docker    : $(docker --version 2>/dev/null || echo 'Installed')"
else
  echo "  [INFO] Docker  : NOT REQUIRED FOR M0"
fi

# NVIDIA CUDA / GPU
if command -v nvcc &>/dev/null; then
  echo "  [OK] CUDA      : $(nvcc --version | grep release | awk '{print $5}' | sed 's/,//')"
else
  echo "  [INFO] CUDA    : NOT REQUIRED FOR M0"
fi

echo "================================================================="
