# Control-system analysis

The final project keeps Gazebo's multicopter motor/attitude controller as the inner loop and adds a transparent outer-loop position controller in `scripts/drone_autonomy_controller.py`.

## Added control elements
- Classical PID terms P, I and D for X/Y/Z position error.
- Anti-windup integral limiting.
- Explicit `Kp`, `Ki`, `Kd` values.
- Integral time `Ti = Kp/Ki` and derivative time `Td = Kd/Kp`.
- Nonlinear quadratic drag compensation `-c*v*|v|`.
- Nonlinear command saturation and command acceleration limiting.
- Finite-difference velocity estimation.
- Live LiDAR clearance and obstacle safety supervisor.
- Physical wind force + roll/pitch torque disturbance and recovery.
- Complete telemetry for desired state, actual state, PID terms, nonlinear terms and commands.

## Graphs
Run `./run_analysis.sh` after a successful Gazebo experiment. It creates tracking-error, actual-vs-desired, P/I/D, PID-output, Kp/Ki/Kd, Ti/Td, nonlinear-dynamics, stabilization, LiDAR, trajectory and separation graphs plus `metrics.csv`.

The graphs are generated from real telemetry; no fabricated measurements are included in the project.
