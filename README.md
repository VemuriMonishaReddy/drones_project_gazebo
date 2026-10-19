<p align="center">
  <img width="441" height="114" alt="logo-branding-amrita-universiy-2024" src="https://github.com/user-attachments/assets/d939ec0b-574a-4d8e-8140-80275a9c9fa7" />
</p>

<div align="center"> <h2> AUTONOMOUS SELF-STABILIZING DRONE WITH OBSTACLE AVOIDANCE </h2> </div>

<p align="center">
  <img src="./results/amrita_logo.png" alt=""C:\Users\DEVI SRI\OneDrive\Desktop\Downloads\logo-branding-amrita-universiy-2024.jpeg"" width="200">
</p>

<p align="center">
  <b>AMRITA VISHWA VIDYAPEETHAM</b><br>
  Coimbatore, Tamil Nadu<br>
  Department of Computer Science and Engineering(Artificial Intelligence)
</p>

---

## TEAM MEMBER DETAILS

| S.No. | Name | Roll Number | Email |
|---|---|---|---|
| 1 | Devisri | CB.SC.U4AIE24163 | devisri7142@gmail.com |
| 2 | Monisha | CB.SC.U4AIE24157 | vemurimonishareddy@gmail.com |
| 3 | Myagi | CB.SC.U4AIE24143 | patimamyagi@gmail.com |

**Team Number:** AB14

---
## ABSTRACT

This project builds an autonomous quadrotor that stabilizes itself and avoids obstacles, implemented in **Python and
PyBullet**. It has two modules. **Self-stabilization:** a mathematical cascaded PID controller (P, I and D terms
computed and logged separately) turns roll, pitch, velocity and altitude errors into individual motor thrusts; a
data-driven system identification study learns the drone's dynamics from flight data, first as a linear model
`x(k+1) = A x(k) + B u(k)` using the pseudo-inverse and then as a nonlinear sparse model (SINDy).
**Intelligent obstacle avoidance:** distance, closing speed and time-to-collision are combined with roll and pitch and
fed to a Logistic Regression model and a pgmpy Bayesian network that predict SAFE / WARNING / DANGER; a decision
controller selects CONTINUE / SLOW_DOWN / AVOID and the avoidance logic changes the drone's velocity. Everything is
integrated in PyBullet, including a realistic world with trees, buildings, a moving bird and turbulent wind.

---

## METHODOLOGY (STEP BY STEP)

### Step 0 — Overall pipeline

```
obstacles + drone state
   -> features (distance, closing speed, time-to-collision, roll, pitch)
   -> risk model (Logistic Regression / pgmpy Bayesian network): P(SAFE), P(WARNING), P(DANGER)
   -> decision controller (minimum expected cost): CONTINUE / SLOW_DOWN / AVOID
   -> avoidance logic: velocity command
   -> velocity PID -> desired roll/pitch -> attitude PID -> torques      (self-stabilization)
   -> motor mixer -> 4 motor thrusts -> motor lag -> PyBullet physics (+ wind)
   -> drone state fed back
```

Physics runs at 240 Hz, the controller at 120 Hz.

---

### Step 1 — Physics of the drone (`physics_model.py`)

Four motors at body positions $(x_i,y_i)=(\pm d,\pm d)$ with thrusts $f_i$ and spin directions $s_i=\pm1$ produce one
thrust and three torques (the **motor mixer**):

$$
T=\sum_i f_i,\qquad \tau_x=\sum_i y_i f_i,\qquad \tau_y=-\sum_i x_i f_i,\qquad \tau_z=k_M\sum_i s_i f_i
$$

The mixer matrix is invertible, so any wanted $[T,\tau_x,\tau_y,\tau_z]$ becomes motor thrusts. Hover needs
$f_i=mg/4=1.226$ N. Each motor also has a first-order lag (time constant 0.03 s).

Motion follows Newton and Euler:

$$
m\dot{\mathbf v}=R\begin{bmatrix}0\\0\\T\end{bmatrix}-mg\,\hat{\mathbf z}-c_d\lVert\mathbf v\rVert\mathbf v+\mathbf F_{wind},
\qquad I\dot{\boldsymbol\omega}=\boldsymbol\tau-\boldsymbol\omega\times I\boldsymbol\omega
$$

For small angles this gives the key relation that lets a quadrotor move sideways by tilting:

$$
a_x\approx g\,\theta,\qquad a_y\approx -g\,\phi
$$

**Why the drone is nonlinear:** the rotation matrix contains $\sin$ and $\cos$ of the angles, the rotation equations
contain products of rates ($qr$, $pr$, $pq$), drag is quadratic in speed, and motors saturate. Near hover it is
almost linear, far from it is not (shown in Step 3).

**Validation:** the analytic model integrated with Runge–Kutta 4 matches PyBullet (RMSE about $10^{-3}$ or less). An
open-loop test shows that with constant thrust a small torque kick is never corrected, which is why feedback is needed.

---

### Step 2 — Self-stabilization with a mathematical PID (`self_stabilizing_pid_ml.py`)

For any controlled quantity with reference $r$ and measurement $y$:

$$
e=r-y,\qquad u=\underbrace{K_p\,e}_{P}+\underbrace{K_i\!\int e\,dt}_{I}+\underbrace{K_d\,\dot e}_{D}
$$

The integral is clamped (anti-windup) and the derivative is the analytic rate error or a filtered difference. The class
logs $e$, $\int e$, $\dot e$, $P$, $I$, $D$ at every step, so each term is plotted individually.
**P** reacts to the present error, **I** removes steady offset (for example from wind), **D** damps oscillation.

**Cascaded control:**

1. position or velocity PID $\rightarrow$ desired acceleration $a_x,a_y$
2. acceleration $\rightarrow$ desired tilt: $\theta_{des}=\arctan(a_x/g),\ \phi_{des}=-\arctan(a_y/g)$ (limit 0.5 rad)
3. altitude PID $\rightarrow$ thrust: $T=\dfrac{m(g+a_z)}{\cos\phi\cos\theta}$
4. attitude PID $\rightarrow$ torque: $\tau=I\cdot\alpha_{PID}$ (roll, pitch, yaw)
5. mixer inverse $\rightarrow$ four motor thrusts

| Loop | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|
| position (x, y) | 3.0 | 1.0 | 3.5 |
| altitude | 12 | 4 | 6 |
| attitude (roll, pitch) | 70 | 15 | 14 |

The fast inner (attitude) loop and slow outer (position) loop keep the cascade stable.

**Experiments**

* **Attitude disturbance:** roll and pitch torque kicks are rejected; $e$, $\int e$, $\dot e$, $P$, $I$, $D$ are plotted.
* **Circular trajectory with constant wind (0.6 N from t = 6 s):** the position loop is run as P, PI, PD and PID and
  the flown path is compared with the reference circle.

| Position loop | RMSE (m) | Final error (m) | Behaviour |
|---|---|---|---|
| P | 2.05 | 2.82 | undamped, unstable |
| PI | 3.96 | 5.16 | integral adds lag, unstable without D |
| PD | 0.36 | 0.46 | damped, but offset remains in wind |
| **PID** | **0.18** | **0.14** | damped and offset removed |

---

### Step 3 — Data-driven system identification (`self_stabilizing_pid_ml.py`)

Question: *given the controller input at one step, where will the drone be at the next step?* The state and input are

$$
\mathbf x=[\phi,\theta,p,q,r,v_x,v_y,v_z]^\top,\qquad \mathbf u=[T-mg,\ \tau_x,\ \tau_y,\ \tau_z]^\top
$$

The drone is flown with random targets and random motor dither, and triplets $(\mathbf x_k,\mathbf u_k,\mathbf x_{k+1})$
are recorded (90 s training, 40 s unseen test flight).

**Linear model (pseudo-inverse).** Assume $\mathbf x_{k+1}=A\mathbf x_k+B\mathbf u_k+\mathbf c$. Stacking all samples,
$X^{+}=\Theta Z$ with $Z=[X;\,U;\,\mathbf 1]$ and $\Theta=[A\ B\ \mathbf c]$, and the least-squares solution is

$$
\Theta=X^{+}\,\mathrm{pinv}(Z)
$$

**Nonlinear model (SINDy).** The derivative is written as a combination of candidate nonlinear functions:

$$
\dot{\mathbf x}=\Theta(\mathbf x,\mathbf u)\,\Xi
$$

The library contains the states and inputs plus $\sin\phi$, $\sin\theta\cos\phi$, $\cos\phi\cos\theta-1$, the products
$pq$, $pr$, $qr$, thrust-times-angle terms and quadratic drag $\lVert\mathbf v\rVert\mathbf v$. Coefficients $\Xi$ come
from sequentially thresholded least squares (pseudo-inverse, drop terms below 1 % of the target, refit), which leaves a
**sparse, interpretable** model. For example it recovers $\dot p\approx\tau_x/I_{xx}=133\,\tau_x$.

**Comparison on unseen data** (0.5 s open-loop prediction, recorded inputs, state never corrected):

| Model | roll $\phi$ (rad) | pitch $\theta$ (rad) | $v_x$ (m/s) |
|---|---|---|---|
| Linear $(A,B)$ | 0.0191 | 0.0175 | 0.1515 |
| **Nonlinear (SINDy)** | **0.0010** | **0.0005** | **0.0184** |
| Analytic physics | 0.0097 | 0.0102 | 0.0653 |

The drone is clearly nonlinear away from hover: the nonlinear identified model is about 10 times more accurate than the
linear one. (It also beats the analytic model, which assumes yaw = 0 and a simple integrator.)

---

### Step 4 — Intelligent obstacle avoidance (`obstacle_avoidance_pgmpy.py`)

**Features.** For an obstacle at $\mathbf p_o$ (radius $r_o$) and the drone at $\mathbf p$ (radius $r_d$):

$$
d=\lVert\mathbf p_o-\mathbf p\rVert-r_o-r_d,\qquad v_c=-(\mathbf v_o-\mathbf v)\cdot\mathbf n,\qquad \mathrm{TTC}=\frac{d}{v_c}\ \ (v_c>0.05)
$$

where $\mathbf n$ is the unit vector towards the obstacle and $v_c>0$ means approaching. Roll and pitch are added as
features because a strongly tilted drone brakes worse.

**Risk prediction.** 20 000 synthetic samples are labelled SAFE / WARNING / DANGER with a distance and TTC rule plus
5 % label noise (so accuracy measures how well the rule is learned, not real-world risk). Two models are trained:

* **Logistic Regression** on $[d,v_c,\mathrm{TTC},|\phi|,|\theta|]$.
* **pgmpy Bayesian network** on discretised features, with Distance $\rightarrow$ TTC $\leftarrow$ ClosingSpeed and
  Distance, TTC, Tilt $\rightarrow$ Risk. Its probability tables are learned from data and exact inference gives
  $P(\text{Risk}\mid\text{evidence})$.

| Model (held-out data) | Accuracy |
|---|---|
| Logistic Regression | 0.87 |
| pgmpy Bayesian network | 0.78 |

The network is less accurate (discretisation) but gives interpretable probabilities.

**Decision controller (minimum expected cost).**

$$
a^*=\arg\min_a\sum_r P(r)\,C[a][r],\qquad
C=\begin{bmatrix}0&3&10\\1&1&5\\3&2&1\end{bmatrix}
$$

(rows CONTINUE, SLOW_DOWN, AVOID; columns SAFE, WARNING, DANGER). Continuing in danger costs 10, avoiding when safe
only 3.

**Avoidance logic.** CONTINUE flies to the goal at 1.5 m/s; SLOW_DOWN cuts speed to 60 % and side-steps if the
obstacle is close and approaching; AVOID dodges sideways with a small retreat. The passing side is chosen once per
obstacle (to avoid chattering) by probing the free space on both sides. In the realistic world a small repulsion from
nearby obstacles and a low-pass filter on the velocity command are added.

**Result: obstacles moving at increasing speed** (30 random crossing scenarios per speed):

| Obstacle speed (m/s) | Collision rate, no avoidance | Collision rate, with avoidance |
|---|---|---|
| 0.5 | 100 % | 0 % |
| 1.0 | 70 % | 7 % |
| 2.0 | 67 % | 0 % |
| 3.0 | 53 % | 3 % |
| 4.0 | 53 % | 13 % |

Without avoidance the rate falls with speed partly because fast obstacles often cross before the drone arrives. The
3-obstacle zig-zag is passed below–above–below with no collision.

---

### Step 5 — Wind and the realistic world (`realistic_simulation.py`)

The wind is a mean wind plus Ornstein–Uhlenbeck turbulence plus random gusts with a "1−cos" shape, and the force uses
the air velocity **relative** to the drone:

$$
\mathbf w(t)=\mathbf w_{mean}\big(1+0.6\,g(t)\big)+\mathbf w_{turb}(t),\qquad
\mathbf F_{wind}=c_d\Big(\lVert\mathbf v\rVert\mathbf v-\lVert\mathbf v-\mathbf w\rVert(\mathbf v-\mathbf w)\Big)
$$

The world has a park path with 6 trees, 5 buildings with windows, a shed, a patrolling bird and a windsock. In the 3-D
window the drone shows a wind arrow and a label with wind speed, drone speed and the current decision, and the trees
sway with the wind.

---

### Step 6 — Integration and evaluation (`full_simulation.py`, `realistic_simulation.py`)

Every control step the system reads the state, computes obstacle features, predicts risk, decides, computes the
velocity command, runs the velocity and attitude PIDs, allocates motor thrusts and advances the physics. Metrics:
collision, goal reached, minimum clearance, time, maximum tilt, maximum altitude error.

| Test | Result |
|---|---|
| Zig-zag, 3 obstacles (PyBullet) | no collision, altitude error $\le$ 0.01 m |
| Crossing obstacles 0.5–3 m/s, with avoidance (6 runs each) | 0 % collisions at 0.5, 2, 3 m/s; 33 % at 1 m/s (small sample) |
| Realistic world, wind 0 / 2 / 4 m/s (3 runs each) | goal reached without collision, altitude within about 0.03 m |
| Realistic world, wind 6 m/s | collisions occurred; without avoidance every run collided |

---

## FILES AND HOW TO RUN

| File | Contents |
|---|---|
| `physics_model.py` | mixer, motor lag, Newton–Euler model, PyBullet drone, validation |
| `self_stabilizing_pid_ml.py` | PID, cascaded controller, experiments, system identification |
| `obstacle_avoidance_pgmpy.py` | features, Logistic Regression, pgmpy network, decision, avoidance, tests |
| `full_simulation.py` | complete system in PyBullet with metrics |
| `realistic_simulation.py` | realistic world with trees, buildings, wind |

```bash
pip install numpy pandas matplotlib scikit-learn pgmpy pybullet

python physics_model.py
python self_stabilizing_pid_ml.py
python obstacle_avoidance_pgmpy.py
python full_simulation.py          # 3-D PyBullet window; add --headless for plots and metrics only
python realistic_simulation.py     # 3-D window with trees, buildings and wind (--wind-speed 6 for stronger wind)
```

All figures are saved in the `plots/` folder.

---

## IMPLEMENTED, TESTED, AND STILL LIMITED

* **Implemented:** PID with separate P, I, D terms; physics with mixer and motor lag; linear and nonlinear system
  identification; Logistic Regression and Bayesian-network risk models; expected-cost decision; avoidance logic; full
  PyBullet integration; realistic world with wind.
* **Tested:** physics against the analytic model; disturbance rejection; P/PI/PD/PID comparison; identification on
  unseen data; classifier accuracy; obstacles at increasing speeds; wind sweep.
* **Limitations and next steps:** risk labels are rule-based and synthetic; the Bayesian network is less accurate than
  Logistic Regression; the planner is reactive (no global planning), so very fast obstacles and strong wind can still
  cause collisions; obstacles are 2-D circles at a fixed altitude; PyBullet sample sizes are small; the identified model
  is not yet used inside the controller (for example model-predictive control).

---

## REFERENCES

1. López-Sánchez, I., & Moreno-Valenzuela, J. (2023). PID Control of Quadrotor UAVs: A Survey. *Annual Reviews in Control*, 56, 100900.
2. López-Sánchez, I., & Moreno-Valenzuela, J. (2019). Position and Attitude Control of Multi-Rotor Aerial Vehicles: A Survey. *Annual Reviews in Control*.
3. Lotufo, M. A., et al. (2019). UAV Quadrotor Attitude Control: An ADRC-EMC Combined Approach. *Control Engineering Practice*, 84, 13–22.
4. Brunton, S. L., Proctor, J. L., & Kutz, J. N. (2016). Discovering governing equations from data by sparse identification of nonlinear dynamical systems. *PNAS*, 113(15), 3932–3937.
5. Ankan, A., & Panda, A. (2015). pgmpy: Probabilistic Graphical Models using Python. *Proceedings of the 14th Python in Science Conference*.
6. Coumans, E., & Bai, Y. PyBullet, a Python module for physics simulation for games, robotics and machine learning.

## LICENSE

MIT License.
