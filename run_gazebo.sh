#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
command -v gz >/dev/null || { echo "Gazebo (gz) is not installed."; exit 1; }
rm -f "$ROOT/results/telemetry.csv" "$ROOT/results/alpha_avoidance_path.png" "$ROOT/results/bravo_stabilization.png" "$ROOT/results/alpha_lidar_clearance.png"
echo "Launching PROFESSOR DEMO v14: BRIGHT TWO-DRONE VALIDATION"
echo "Alpha = LiDAR obstacle avoidance | Bravo = physical wind + self-stabilization"
echo "Green floor = planned safe corridor | Blue arrows = visible wind direction"
gz sim -r "$ROOT/worlds/obstacle_course.sdf" & GZ_PID=$!
trap 'kill "$GZ_PID" 2>/dev/null || true' EXIT
sleep 8
python3 "$ROOT/scripts/gazebo_autonomy_controller.py"
RC=$?
echo
echo "=== DEMO COMPLETE ==="
echo "Results: $ROOT/results"
echo "Gazebo remains open for 90 seconds so you can inspect the final state."
sleep 90
exit $RC
