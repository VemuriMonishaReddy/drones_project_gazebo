# Gazebo Drone Project

This is the Gazebo implementation of the uploaded PyBullet drone project. It keeps the v15 visual/weather environment and adds the missing algorithmic components from the PyBullet reference.

## Run
```bash
chmod +x run_gazebo.sh
./run_gazebo.sh
```

Then:
```bash
./run_analysis.sh
```

## Two drones
- **Alpha**: continuous waypoint flight + LiDAR risk-aware obstacle avoidance.
- **Bravo**: continuous flight + same risk supervisor + physical wind/torque disturbance and recovery.

## Matching PyBullet components
- Logistic Regression risk model
- Bayesian Network risk model
- distance / closing speed / TTC / roll / pitch features
- expected-cost decision theory
- CONTINUE / SLOW_DOWN / AVOID
- side-selection hysteresis
- cascaded PID mathematics
- Kp/Ki/Kd, integral time Ti, derivative time Td
- nonlinear drag and saturation
- physical wind disturbance
- ML/system identification
- speed-sweep benchmark

## Gazebo-specific implementation choice
Gazebo's supported `MulticopterVelocityControl` remains the physical inner-loop flight controller and its motor models remain the true actuator/plant. The project therefore does not falsely claim that the supervisory PID gains are Gazebo's internal motor PID gains.

See `docs/PYBULLET_TO_GAZEBO_PORT.md` for the exact mapping.
