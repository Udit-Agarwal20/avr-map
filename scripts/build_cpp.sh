#!/usr/bin/env bash
set -euo pipefail

# Add homebrew path for cmake if needed
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CPP_DIR="${REPO_ROOT}/backend/cpp"

PRESET="${1:-dev}"

case "${PRESET}" in
  dev|debug|release)
    ;;
  *)
    echo "Usage: $0 [dev|debug|release]"
    exit 1
    ;;
esac

echo "================================================================="
echo "Building AVR-Map C++ Core with preset: '${PRESET}'"
echo "================================================================="

cd "${CPP_DIR}"

echo ">>> Configuring preset '${PRESET}'..."
cmake --preset "${PRESET}"

echo ">>> Compiling preset '${PRESET}'..."
cmake --build --preset "${PRESET}" --parallel

echo ">>> Exposing compilation database for IDE language servers..."
if [[ -f "${CPP_DIR}/build/${PRESET}/compile_commands.json" ]]; then
  ln -sf "${CPP_DIR}/build/${PRESET}/compile_commands.json" "${REPO_ROOT}/compile_commands.json"
  ln -sf "build/${PRESET}/compile_commands.json" "${CPP_DIR}/compile_commands.json"
fi

echo "================================================================="
echo "Build succeeded for preset '${PRESET}'."
echo "================================================================="
