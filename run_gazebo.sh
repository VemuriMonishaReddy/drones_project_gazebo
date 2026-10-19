#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
command -v gz >/dev/null || { echo "Gazebo (gz) is not installed."; exit 1; }
rm -f "$ROOT/results/telemetry.csv" "$ROOT/results/metrics.csv" "$ROOT/results"/*.png "$ROOT/results"/*.json "$ROOT/results"/*.csv
mkdir -p "$ROOT/results"
echo "Launching B-SPLINE AUTONOMOUS DRONE DEMO"
echo "Alpha: cubic B-spline + Bayesian/Logistic risk + TTC + LiDAR + cascaded PID"
echo "Bravo: parallel B-spline companion + physical wind/torque recovery"
gz sim -r "$ROOT/worlds/drone_obstacle_course.sdf" & GZ_PID=$!
trap 'kill "$GZ_PID" 2>/dev/null || true' EXIT
sleep 8
python3 "$ROOT/scripts/drone_autonomy_controller.py"
RC=$?
if [ "$RC" -eq 0 ]; then
  python3 "$ROOT/scripts/control_analysis.py" "$ROOT/results/telemetry.csv" || true
  python3 "$ROOT/scripts/validate_models.py" "$ROOT/results/risk_model_metrics.json" || true
  python3 "$ROOT/scripts/system_identification.py" "$ROOT/results/telemetry.csv" || true
  python3 "$ROOT/scripts/drone_benchmark.py" "$ROOT/results/speed_sweep.csv" || true
fi
echo "=== DEMO COMPLETE ==="
echo "Results: $ROOT/results"
sleep 10
exit $RC
