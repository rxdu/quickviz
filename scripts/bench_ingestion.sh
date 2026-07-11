#!/usr/bin/env bash
#
# scripts/bench_ingestion.sh — one-command runner for the viz-plane
# ingestion benchmark suite (docs/ingestion_contract.md).
#
# Builds the bench target if needed, then runs it. When no display is
# available, the windowed groups are run under xvfb-run (offscreen X server,
# software GL) — the render mode is recorded in the JSON report's hardware
# context and does not affect ingestion-side numbers if decoupling holds,
# which is exactly what the suite verifies.
#
# Usage:
#   ./scripts/bench_ingestion.sh [--smoke] [--filter=<substring>] \
#                                [--out=<report.json>] [--build-dir=<dir>]

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
PASSTHROUGH=()

for arg in "$@"; do
  case "${arg}" in
    --build-dir=*) BUILD_DIR="${arg#--build-dir=}" ;;
    *) PASSTHROUGH+=("${arg}") ;;
  esac
done

if [ ! -d "${BUILD_DIR}" ]; then
  echo "-- configuring ${BUILD_DIR}"
  cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
fi

echo "-- building bench_ingestion"
cmake --build "${BUILD_DIR}" --target bench_ingestion -j "$(nproc)"

BIN="${BUILD_DIR}/bin/bench_ingestion"
if [ ! -x "${BIN}" ]; then
  # Fall back to wherever the build placed it.
  BIN="$(find "${BUILD_DIR}" -name bench_ingestion -type f | head -n1)"
fi

RUNNER=()
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  if command -v xvfb-run > /dev/null 2>&1; then
    echo "-- no display detected: running under xvfb-run (offscreen X)"
    RUNNER=(xvfb-run -a)
  else
    echo "-- WARNING: no display and no xvfb-run; windowed groups will skip"
  fi
fi

exec "${RUNNER[@]}" "${BIN}" "${PASSTHROUGH[@]}"
