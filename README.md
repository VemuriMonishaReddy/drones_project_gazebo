# Gazebo Professor Demo v14 — Two Drones

A presentation-focused Gazebo Sim demonstration of two core autonomous-drone capabilities:

1. **Obstacle avoidance** — Alpha uses live GPU LiDAR, slows/brakes as the red obstacle closes, and follows a conservative, collision-safe waypoint corridor around the obstacle. The LiDAR rays are visible in the GUI.
2. **Self-stabilization under wind** — Bravo hovers in a clear area, receives a physical lateral force plus roll/pitch torque, then recovers through the Gazebo multicopter attitude/rate controller. Bright blue wind arrows make the disturbance direction visible.

## Visual design

- Bright sky-blue background and light ground.
- High-contrast red, blue, orange, green and purple obstacles.
- Yellow Alpha and blue Bravo quadrotors.
- Visible LiDAR rays on Alpha.
- Green visual-only safe corridor for the obstacle experiment.
- Large cyan wind arrows beside Bravo.
- Wide initial camera view; use Gazebo Entity Tree → `alpha` → Follow for the avoidance close-up, then `bravo` → Follow for the wind section.

## Demonstration timeline

- 0–8 s: both drones establish stable hover.
- 8–20 s: Alpha approaches the red obstacle and uses live LiDAR.
- 20–32 s: Alpha moves into the wide green safe corridor and crosses around the obstacle.
- 32–42 s: Alpha clears the obstacle and returns to its route.
- 47–51 s: Bravo receives a real force + roll/pitch torque gust.
- 51–60 s: gust is removed; Bravo continues the closed-loop recovery to level hover.
- 60–105 s: post-demonstration stable flight.
- Gazebo remains open for 90 s after telemetry is saved.

## Collision-safety design

The Alpha path is state/waypoint based rather than time-cutting. The route keeps the complete quadrotor well outside an inflated footprint of the red box. A separate emergency supervisor prevents commands toward the obstacle if the vehicle unexpectedly enters the inflated safety envelope. LiDAR remains the sensing/evidence layer; the conservative corridor is the safety guarantee for the professor demonstration.

## Run

```bash
pkill -f gz
cd ~/Downloads/gazebo_drone_professor_demo_v14
chmod +x run_gazebo.sh
./run_gazebo.sh
```

Do not press Ctrl+C during the experiment.

## Results

The controller writes:

- `results/telemetry.csv`
- `results/alpha_avoidance_path.png`
- `results/bravo_stabilization.png`
- `results/alpha_lidar_clearance.png`


## v15 weather visualization
This version adds a brighter outdoor environment with trees, visible wind arrows, wind-borne leaf and paper particle streams, and a rain-storm particle emitter. The physical wind disturbance remains applied to Bravo as a force/torque experiment; the debris is visual evidence of the wind direction. Rain is activated later in the run so the professor can see a second environmental condition without hiding the obstacle-avoidance experiment. Gazebo's ParticleEmitter system supports configurable rate, velocity, lifetime and visual particles.
