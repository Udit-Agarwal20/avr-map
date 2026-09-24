#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/backend/cpp/build"

echo "================================================================="
echo "Cleaning AVR-Map C++ Build Directories"
echo "================================================================="

if [[ -d "${BUILD_DIR}" ]]; then
  echo "Removing: ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
  rm -f "${REPO_ROOT}/compile_commands.json" "${REPO_ROOT}/backend/cpp/compile_commands.json"
  echo "Clean complete."
else
  rm -f "${REPO_ROOT}/compile_commands.json" "${REPO_ROOT}/backend/cpp/compile_commands.json"
  echo "Build directory does not exist (${BUILD_DIR}). Nothing to clean."
fi
