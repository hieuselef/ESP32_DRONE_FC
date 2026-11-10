#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p .host-tests
for t in tests/test_*.cpp; do
  out=".host-tests/$(basename "${t%.cpp}")"
  echo "[HOST] ${t}"
  g++ -std=c++17 -Wall -Wextra -O2 -Iinclude "$t" -o "$out"
  "$out"
done
