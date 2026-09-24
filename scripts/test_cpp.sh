#!/usr/bin/env bash
set -euo pipefail

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
echo "Running AVR-Map C++ Tests with preset: '${PRESET}'"
echo "================================================================="

cd "${CPP_DIR}"

ctest --preset "${PRESET}" --output-on-failure

echo "================================================================="
echo "All tests passed for preset '${PRESET}'."
echo "================================================================="
