#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$ROOT/results"
python3 "$ROOT/scripts/control_analysis.py" "$ROOT/results/telemetry.csv"
python3 "$ROOT/scripts/validate_models.py" "$ROOT/results/risk_model_metrics.json"
python3 "$ROOT/scripts/system_identification.py" "$ROOT/results/telemetry.csv"
python3 "$ROOT/scripts/drone_benchmark.py" "$ROOT/results/speed_sweep.csv"
echo "All analysis complete in $ROOT/results"
