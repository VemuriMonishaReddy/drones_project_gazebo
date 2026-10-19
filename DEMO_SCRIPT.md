# 90-second professor narration

### 0–8 s — baseline
"We first establish a stable hover. There are only two vehicles so the behavior of each controller is easy to observe."

### 8–42 s — obstacle avoidance
"Alpha is deliberately commanded toward the red obstacle. The green strip is the safe corridor. The visible LiDAR rays show that the vehicle is sensing the environment rather than simply flying through it. As clearance decreases, the controller reduces forward motion, then moves into the clear corridor, crosses the obstacle and returns to the mission path."

### 47–60 s — wind and stabilization
"Bravo is now in an obstacle-free area. The blue arrows show the direction of the external gust. Gazebo applies an actual force and roll/pitch torque to the vehicle. You can see the drone tilt, then the closed-loop multicopter controller produces the corrective response and brings it back toward level hover."

### Evidence
"The telemetry records position, attitude, LiDAR clearance and the disturbance interval. The plots provide quantitative evidence in addition to the live visualization."


### Weather demonstration (v15)
At ~44 s, point out the moving leaves and paper scraps: these are visual wind indicators. At 47-51 s, Bravo receives the physical gust + roll/pitch torque. At ~62 s, visible rain begins; explain that the flight controller remains active and the drone maintains its commanded hover while the environment changes. The rain particles are visual-only and are deliberately kept out of LiDAR scattering so the obstacle-avoidance experiment remains deterministic.
