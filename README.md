<p align="center">
  <img width="441" height="114" alt="logo-branding-amrita-universiy-2024" src="https://github.com/user-attachments/assets/d939ec0b-574a-4d8e-8140-80275a9c9fa7" />
</p>

<div align="center"> <h2> AUTONOMOUS SELF-STABILIZING DRONE WITH OBSTACLE AVOIDANCE </h2> </div>

<p align="center">
  <img src="./results/amrita_logo.png" alt="Amrita Vishwa Vidyapeetham logo" width="200">
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
PyBullet**. A physics module defines how the drone flies (motor mixer, motor lag, drag, wind). A self-stabilization
module controls it with a mathematical cascaded PID (P, I and D terms logged separately) and learns its dynamics from
flight data (linear model with the pseudo-inverse, nonlinear model with SINDy). An obstacle-avoidance module computes
distance, closing speed and time-to-collision, predicts the risk SAFE / WARNING / DANGER with Logistic Regression and a
pgmpy Bayesian network, chooses CONTINUE / SLOW_DOWN / AVOID by minimum expected cost, and converts the action into a
velocity command. Two simulation files combine everything, including a realistic world with trees, buildings, a moving
bird and turbulent wind.

---

## METHODOLOGY

### 1. How the system is organised

The project has **four main codes** and one extension:

| Code | Question it answers |
|---|---|
| `physics_model.py` | *How does the drone fly?* (physics, motors, wind force, PyBullet drone) |
| `self_stabilizing_pid_ml.py` | *How do we keep it stable, and what kind of system is it?* (PID + system identification) |
| `obstacle_avoidance_pgmpy.py` | *When is it in danger and what should it do?* (features, ML risk, decision, avoidance) |
| `full_simulation.py` | *Does everything work together?* (integration and metrics in PyBullet) |
| `realistic_simulation.py` | *Does it still work with trees, buildings and wind?* (extension of the integration) |

### 2. Overall pipeline (runs every 1/120 s)

```
 ┌──────────────────────────── obstacle_avoidance_pgmpy.py ────────────────────────────┐
 │ obstacles + drone state                                                              │
 │    1. FEATURES: distance d, closing speed v_c, time-to-collision TTC, |roll|, |pitch|│
 │    2. RISK MODEL: Logistic Regression / pgmpy Bayesian network                       │
 │                   -> P(SAFE), P(WARNING), P(DANGER)                                  │
 │    3. DECISION: minimum expected cost -> CONTINUE / SLOW_DOWN / AVOID                │
 │    4. AVOIDANCE: action -> velocity command v_cmd                                    │
 └───────────────────────────────────────┬──────────────────────────────────────────────┘
                                         v
 ┌─────────────────────── self_stabilizing_pid_ml.py ───────────────────────────────────┐
 │    5. VELOCITY PID: v_cmd - v  -> desired acceleration -> desired roll/pitch         │
 │    6. ATTITUDE PID: roll/pitch error -> torques;  ALTITUDE PID -> total thrust       │
 └───────────────────────────────────────┬──────────────────────────────────────────────┘
                                         v
 ┌─────────────────────────── physics_model.py ─────────────────────────────────────────┐
 │    7. MOTOR MIXER: [T, tau_x, tau_y, tau_z] -> four motor thrusts                    │
 │    8. PHYSICS: motor lag + gravity + drag + wind + PyBullet rigid body               │
 │       -> new drone state (position, velocity, roll, pitch, rates)                    │
 └───────────────────────────────────────┬──────────────────────────────────────────────┘
                                         v
                       state fed back to step 1  (loop closed by full_simulation.py)
```

Physics runs at 240 Hz ($\Delta t_{sim}=1/240$ s) and the controller at 120 Hz ($\Delta t=1/120$ s, two physics steps
per control step).

### 3. Development steps (the order in which the project was built)

1. Build and validate the physics (`physics_model.py`).
2. Design the cascaded PID and test it with disturbances and wind (`self_stabilizing_pid_ml.py`, part A).
3. Learn the drone's dynamics from flight data (`self_stabilizing_pid_ml.py`, part B).
4. Build the risk models, decision and avoidance and test them on a point-mass drone (`obstacle_avoidance_pgmpy.py`).
5. Integrate stabilization and avoidance on the PyBullet drone and evaluate (`full_simulation.py`).
6. Add a realistic world and wind and evaluate again (`realistic_simulation.py`).

---

## CODE 1 — `physics_model.py` (the physics that keeps the drone flying)

**What it does:** defines the drone's mass, inertia and motors, converts the controller's thrust and torques into
motor thrusts, applies motor lag, drag and wind, and creates the PyBullet drone body. It also proves the physics is
correct by comparing PyBullet with the analytic equations.

**Pipeline of this file**
```
controller output [T, tau] -> mixer inverse -> motor commands -> motor lag -> actual thrusts
   -> wrench -> force and torque on the body (+ drag + wind) -> PyBullet step -> state
```

**Equations used**

*(a) Motor mixer.* Motors sit at $(x_i,y_i)=(\pm d,\pm d)$, $d=L/\sqrt2=0.1202$ m, with spin $s_i=\pm1$:

$$
T=\sum_{i=1}^{4}f_i,\qquad \tau_x=\sum_i y_i f_i,\qquad \tau_y=-\sum_i x_i f_i,\qquad \tau_z=k_M\sum_i s_i f_i
$$

$$
\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}=M\,\mathbf f,\qquad
\mathbf f=M^{-1}\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix},\qquad 0\le f_i\le f_{max}=3\text{ N}
$$

*(b) Hover thrust:* $f_i=\dfrac{mg}{4}=1.226$ N for $m=0.5$ kg.

*(c) Motor lag* (first order, applied every physics step):

$$
f_i\leftarrow f_i+\alpha\,(f_i^{cmd}-f_i),\qquad \alpha=\frac{\Delta t_{sim}}{\tau_m+\Delta t_{sim}},\quad\tau_m=0.03\text{ s}
$$

*(d) Rotation matrix* (body to world, $R=R_z(\psi)R_y(\theta)R_x(\phi)$) and translational dynamics:

$$
m\dot{\mathbf v}=R\begin{bmatrix}0\\0\\T\end{bmatrix}-\begin{bmatrix}0\\0\\mg\end{bmatrix}-c_d\lVert\mathbf v\rVert\,\mathbf v+\mathbf F_{wind}
$$

*(e) Small-angle relation* (why tilting makes the drone move):

$$
a_x\approx g\,\theta,\qquad a_y\approx-g\,\phi
$$

*(f) Euler-angle kinematics:*

$$
\dot\phi=p+(q\sin\phi+r\cos\phi)\tan\theta,\qquad \dot\theta=q\cos\phi-r\sin\phi,\qquad
\dot\psi=\frac{q\sin\phi+r\cos\phi}{\cos\theta}
$$

*(g) Euler's rotation equations with rotational drag:*

$$
\dot p=\frac{(I_{yy}-I_{zz})qr+\tau_x-c_r\lVert\omega\rVert p}{I_{xx}},\quad
\dot q=\frac{(I_{zz}-I_{xx})pr+\tau_y-c_r\lVert\omega\rVert q}{I_{yy}},\quad
\dot r=\frac{(I_{xx}-I_{yy})pq+\tau_z-c_r\lVert\omega\rVert r}{I_{zz}}
$$

*(h) Validation by Runge–Kutta 4* (analytic model driven by the same wrench as PyBullet):

$$
\mathbf x_{k+1}=\mathbf x_k+\frac{\Delta t}{6}\,(k_1+2k_2+2k_3+k_4)
$$

**Why the drone is nonlinear:** $\sin,\cos,\tan$ of the angles in (d) and (f), the products $qr$, $pr$, $pq$ in (g),
quadratic drag in (d), and motor saturation in (a). Near hover these are small; far from hover they dominate.

**Result:** analytic model and PyBullet agree (RMSE about $10^{-3}$ or less). With constant thrust and no controller,
a small torque kick is never corrected, which proves feedback is needed.

---

## CODE 2 — `self_stabilizing_pid_ml.py` (self-stabilization + data-driven part)

### Part A — Mathematical PID

**What it does:** keeps the drone level and on course by turning errors into motor thrusts, with P, I and D computed
and logged separately.

**Pipeline**
```
reference - measurement -> position/velocity PID -> desired acceleration -> desired roll/pitch
  -> attitude PID -> torques (+ altitude PID -> thrust) -> mixer inverse -> motor thrusts
```

**Equations used**

*(a) PID law* for any controlled quantity with reference $r$ and measurement $y$:

$$
e=r-y,\qquad u=\underbrace{K_p\,e}_{P}+\underbrace{K_i\!\int_0^t e\,d\tau}_{I}+\underbrace{K_d\,\frac{de}{dt}}_{D}
$$

*(b) Discrete form in the code* (integral with anti-windup clamp, derivative either analytic or filtered):

$$
I_k=I_{k-1}+e_k\Delta t,\quad |K_iI_k|\le5;\qquad
\dot e_k=\dot r_k-\dot y_k\ \ \text{or}\ \ d_k=d_{k-1}+0.15\Big(\frac{e_k-e_{k-1}}{\Delta t}-d_{k-1}\Big)
$$

*(c) Outer loop, position mode and velocity mode* (the second is used with obstacle avoidance):

$$
a_x=K_p(x_{ref}-x)+K_i\!\int\!(x_{ref}-x)dt+K_d(v_{x,ref}-v_x),\qquad
a_x=K_p(v_{x,cmd}-v_x)+K_i\!\int\!(v_{x,cmd}-v_x)dt+K_d\dot e
$$

(the same for $y$).

*(d) Acceleration to desired tilt* (rotated into the yaw frame, limited to $\pm0.5$ rad):

$$
a_{bx}=\cos\psi\,a_x+\sin\psi\,a_y,\quad a_{by}=-\sin\psi\,a_x+\cos\psi\,a_y,\quad
\theta_{des}=\arctan\frac{a_{bx}}{g},\quad\phi_{des}=-\arctan\frac{a_{by}}{g}
$$

*(e) Altitude PID to thrust* (tilt compensated):

$$
a_z=\mathrm{PID}_z(z_{ref}-z),\qquad T=\frac{m\,(g+a_z)}{\cos\phi\,\cos\theta}
$$

*(f) Attitude PID to torque* (PID output is an angular acceleration; derivative uses the measured rate, reference rate 0):

$$
\tau_x=I_{xx}\,\mathrm{PID}(\phi_{des}-\phi;\ \dot e=-p),\quad
\tau_y=I_{yy}\,\mathrm{PID}(\theta_{des}-\theta;\ \dot e=-q),\quad
\tau_z=I_{zz}\,\mathrm{PID}(\psi_{ref}-\psi;\ \dot e=-r)
$$

*(g) Motor allocation:* $\mathbf f=M^{-1}[T,\tau_x,\tau_y,\tau_z]^\top$, clipped to $[0,f_{max}]$.

*(h) Reference circle used in the wind experiment:*
$\mathbf r(t)=(R\cos\omega t,\ R\sin\omega t,\ 1)$, $R=1$ m, $\omega=2\pi/10$ rad/s, and tracking error
$\sqrt{\tfrac1N\sum\lVert\mathbf p-\mathbf r\rVert^2}$ (RMSE).

| Loop | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|
| position (x, y) | 3.0 | 1.0 | 3.5 |
| velocity (x, y) | 4.0 | 1.0 | 0.05 |
| altitude | 12 | 4 | 6 |
| attitude (roll, pitch) | 70 | 15 | 14 |
| yaw | 20 | 2 | 8 |

Attitude loop: $\omega_n=\sqrt{K_p}\approx8.4$ rad/s, damping $K_d/(2\sqrt{K_p})\approx0.84$; the outer loop (about
1.7 rad/s) is much slower, so the cascade is stable.

**Experiments and results**

1. *Attitude disturbance:* roll torque kick (0.05 N·m, 2.0–2.2 s) and pitch kick (5.0–5.2 s); the drone returns to level.
   Plots show $e,\ \int e,\ \dot e,\ P,\ I,\ D$ individually.
2. *Circle with constant wind (0.6 N from 6 s), position loop as P, PI, PD, PID:*

| Position loop | RMSE (m) | Final error (m) | Behaviour |
|---|---|---|---|
| P | 2.05 | 2.82 | no damping, unstable |
| PI | 3.96 | 5.16 | integral adds lag, unstable without D |
| PD | 0.36 | 0.46 | damped, but offset remains in wind |
| **PID** | **0.18** | **0.14** | damped and offset removed |

### Part B — Data-driven system identification (the ML part)

**What it does:** learns, from flight data, where the drone will be at the next time step for a given input, first as a
linear and then as a nonlinear system.

**Pipeline**
```
fly with random targets + motor noise -> record (x_k, u_k, x_{k+1}) -> fit linear model (pseudo-inverse)
   -> fit nonlinear model (SINDy) -> compare with analytic physics on unseen flight
```

**Equations used**

*(a) State and input:*

$$
\mathbf x=[\phi,\theta,p,q,r,v_x,v_y,v_z]^\top,\qquad \mathbf u=[T-mg,\ \tau_x,\ \tau_y,\ \tau_z]^\top
$$

Data: random target every 0.5 s, motor noise $\mathcal N(0,0.12^2)$ N, 90 s for training and a separate 40 s flight for testing.

*(b) Linear model with the pseudo-inverse:*

$$
\mathbf x_{k+1}=A\mathbf x_k+B\mathbf u_k+\mathbf c,\qquad
X^{+}=\Theta Z,\quad Z=\begin{bmatrix}X\\U\\\mathbf 1^\top\end{bmatrix},\quad \Theta=[A\ B\ \mathbf c],\qquad
\Theta=X^{+}\,Z^{\dagger}
$$

with $Z^{\dagger}=Z^\top(ZZ^\top)^{-1}$ for full row rank (`np.linalg.pinv`): the least-squares solution of an
over-determined system.

*(c) Nonlinear model (SINDy):*

$$
\dot{\mathbf x}\approx\frac{\mathbf x_{k+1}-\mathbf x_k}{\Delta t}=\Theta(\mathbf x,\mathbf u)\,\Xi,\qquad
\mathbf x_{k+1}=\mathbf x_k+\Delta t\,\Theta(\mathbf x_k,\mathbf u_k)\,\Xi
$$

The library $\Theta$ has 36 columns: $1$, the 8 states, the 4 inputs, and 23 nonlinear candidates:
$\sin\phi,\ \sin\theta,\ \sin\theta\cos\phi,\ \cos\phi\cos\theta-1,\ q\sin\phi\tan\theta,\ r\cos\phi\tan\theta,\
q(\cos\phi-1),\ r\sin\phi,\ pq,\ pr,\ qr,\ \Delta T\sin\theta\cos\phi,\ \Delta T\sin\phi,\ \Delta T(\cos\phi\cos\theta-1),\
\phi^2,\ \theta^2,\ \phi\theta,\ \lVert\mathbf v\rVert v_{x,y,z},\ \lVert\omega\rVert\,p,q,r$.

*(d) Sequentially thresholded least squares (STLSQ)* finds $\Xi$:

1. normalise the columns of $\Theta$;
2. $\Xi=\Theta_n^{\dagger}\,\dot X$;
3. set $\Xi_{jk}=0$ if $|\Xi_{jk}|<0.01\,\lVert\dot X_k\rVert$;
4. refit the remaining terms with the pseudo-inverse; repeat 10 times.

The result is a sparse, readable equation (for example $\dot p\approx\tau_x/I_{xx}=133\,\tau_x$).

*(e) Evaluation:* one-step and 0.5 s open-loop prediction (60 steps, recorded inputs, 150 windows):

$$
\mathrm{RMSE}_j=\sqrt{\frac1N\sum_k\big(\hat x_{j,k}-x_{j,k}\big)^2}
$$

| 0.5 s open-loop RMSE | roll $\phi$ (rad) | pitch $\theta$ (rad) | $v_x$ (m/s) |
|---|---|---|---|
| Linear $(A,B)$ | 0.0191 | 0.0175 | 0.1515 |
| **Nonlinear (SINDy)** | **0.0010** | **0.0005** | **0.0184** |
| Analytic physics | 0.0097 | 0.0102 | 0.0653 |

**Conclusion:** the drone is clearly nonlinear away from hover; the nonlinear identified model is about 10 times more
accurate than the linear one.

---

## CODE 3 — `obstacle_avoidance_pgmpy.py` (intelligent obstacle avoidance)

**What it does:** turns the drone and obstacle positions into a safe velocity command: measure danger, predict risk with
machine learning, choose the cheapest action, move away. It contains its own tests.

**Pipeline**
```
features -> synthetic training data (75 % / 25 % split) -> Logistic Regression + Bayesian network
   -> risk probabilities -> expected-cost decision -> avoidance velocity -> tests (zig-zag, speed sweep)
```

**Equations used**

*(a) Features* (drone radius $r_d=0.15$ m, obstacle radius $r_o$):

$$
d=\lVert\mathbf p_o-\mathbf p\rVert-r_o-r_d,\qquad \mathbf n=\frac{\mathbf p_o-\mathbf p}{\lVert\mathbf p_o-\mathbf p\rVert},\qquad
v_c=-(\mathbf v_o-\mathbf v)\cdot\mathbf n
$$

$$
\mathrm{TTC}=\begin{cases}d/v_c & v_c>0.05\ \text{m/s}\\ 20\ \text{s}&\text{otherwise}\end{cases}
$$

*(b) Training data* (generated in code, 20 000 samples; $k=1.4$ if $\max(|\phi|,|\theta|)>0.3$ rad else $1$):

$$
\text{DANGER: } d<0.6k\ \text{ or }(\mathrm{TTC}<1.0k\wedge d<3\text{ m});\qquad
\text{WARNING: } d<1.5k\ \text{ or }(\mathrm{TTC}<2.5k\wedge d<4\text{ m});\qquad\text{else SAFE}
$$

5 % of the labels are flipped to a neighbouring class (noise). The labels come from this rule, so accuracy measures how
well the rule is learned, not real-world risk.

*(c) Model A, Logistic Regression* on $\mathbf z=\text{standardise}([d,v_c,\mathrm{TTC},|\phi|,|\theta|])$:

$$
P(\text{risk}=c\mid\mathbf z)=\frac{\exp(\mathbf w_c^\top\mathbf z+b_c)}{\sum_{c'}\exp(\mathbf w_{c'}^\top\mathbf z+b_{c'})}
$$

(cross-entropy with L2 regularisation, $C=5$). Accuracy 0.87.

*(d) Model B, pgmpy Bayesian network.* Groups: Distance (Near $<0.8$, Mid $<2.0$, Far) m, ClosingSpeed (Low $<0.2$,
Med $<1.2$, High) m/s, TTC (Short $<1.2$, Medium $<3$, Long) s, Tilt (Level, Tilted $\ge0.3$ rad). Edges:
Distance $\rightarrow$ TTC, ClosingSpeed $\rightarrow$ TTC, Distance, TTC, Tilt $\rightarrow$ Risk.

$$
P(D,S,T,L,R)=P(D)\,P(S)\,P(L)\,P(T\mid D,S)\,P(R\mid D,T,L),\qquad
\hat\theta_{ijk}=\frac{N_{ijk}+\alpha/(r_iq_i)}{N_{ij}+\alpha/q_i}\ \ (\alpha=5)
$$

Exact inference (variable elimination) is pre-computed for all $3\times3\times3\times2=54$ evidence combinations.
Accuracy 0.78 (lower because grouping loses detail, but the probabilities are interpretable).

*(e) Decision controller (minimum expected cost):*

$$
a^*=\arg\min_a\sum_r P(r)\,C[a][r],\qquad
C=\begin{bmatrix}0&3&10\\1&1&5\\3&2&1\end{bmatrix}
$$

(rows CONTINUE, SLOW_DOWN, AVOID; columns SAFE, WARNING, DANGER). Example with $P=[0.1,0.3,0.6]$: expected costs
6.9, 3.4, 1.5, so AVOID is chosen.

*(f) Avoidance logic.* Preferred velocity, with $v_{cruise}=1.5$ m/s:

$$
\mathbf v_{pref}=v_{cruise}\frac{\mathbf g-\mathbf p}{\lVert\mathbf g-\mathbf p\rVert}\min\big(1,\lVert\mathbf g-\mathbf p\rVert\big)
$$

With heading $\hat{\mathbf h}$, its left perpendicular $\mathbf s$ and passing side $\sigma=\pm1$:

$$
\mathbf v_{cmd}=\begin{cases}
\mathbf v_{pref}&\text{CONTINUE}\\
0.6\,\mathbf v_{pref}+0.35\,\sigma\,\mathbf s&\text{SLOW\_DOWN (side-step only if }d<1.8\text{ m and approaching)}\\
1.8\,(0.9\,\sigma\,\mathbf s-0.3\,\mathbf n)+0.4\,\mathbf v_{pref}&\text{AVOID}
\end{cases}
$$

limited to 2 m/s. The side $\sigma$ is chosen once per obstacle and remembered, by scoring both sides:

$$
\text{probe}_\sigma=\mathbf p+1.8\,\sigma\mathbf s+2.0\,\hat{\mathbf h},\qquad
\text{score}_\sigma=\min(\text{clearance}_\sigma,2.5)-0.3\,(\text{off-line distance})+0.5\,[\sigma=\sigma_{geom}]
$$

*(g) Optional repulsion (used in the realistic world):*

$$
\mathbf v_{rep}=0.35\Big(\frac1{d_s}-\frac1{1.6}\Big)\frac{\mathbf p-\mathbf p_o}{\lVert\mathbf p-\mathbf p_o\rVert}\quad(d_s<1.6\text{ m},\ d_s\ge0.15\text{ m})
$$

*(h) Test model (point-mass drone):* $\dot{\mathbf v}=(\mathbf v_{cmd}-\mathbf v)/\tau$, $\tau=0.35$ s, acceleration limit 4 m/s²,
tilt $\approx\arctan(a/g)$.

**Result: obstacle crossing the path at increasing speed (30 random scenarios per speed)**

| Obstacle speed (m/s) | Collisions, no avoidance | Collisions, with avoidance |
|---|---|---|
| 0.5 | 100 % | 0 % |
| 1.0 | 70 % | 7 % |
| 2.0 | 67 % | 0 % |
| 3.0 | 53 % | 3 % |
| 4.0 | 53 % | 13 % |

Without avoidance the rate falls with speed partly because fast obstacles often cross before the drone arrives.
The 3-obstacle zig-zag is passed below–above–below with no collision.

---

## CODE 4 — `full_simulation.py` and `realistic_simulation.py` (integration)

**What they do:** connect Code 3 (what to do), Code 2 (how to stabilise and move) and Code 1 (the physics) in one
closed loop on the PyBullet drone, and evaluate the whole system.

**Pipeline of one control step**
```
read state -> obstacle features -> risk probabilities -> decision -> v_cmd -> (smoothing)
  -> velocity PID -> attitude PID -> mixer -> motor lag -> wind + physics -> log metrics -> next step
```

**Equations used (new compared with Codes 1–3)**

*(a) Collision and clearance metric:* $c=\lVert\mathbf p_o(t)-\mathbf p\rVert-r_o-r_d$; collision if $c<0$; minimum
clearance is $\min_t c$.

*(b) Command smoothing in the realistic world* (removes chattering when the decision switches):

$$
\mathbf v_f\leftarrow\mathbf v_f+(\mathbf v_{cmd}-\mathbf v_f)\frac{\Delta t}{0.5}
$$

*(c) Wind model* (`realistic_simulation.py`): mean wind, Ornstein–Uhlenbeck turbulence and "1−cos" gusts:

$$
\mathbf w(t)=\mathbf w_{mean}\big(1+0.6\,g(t)\big)+\mathbf w_{turb}(t),\qquad
g(t)=\tfrac12\Big(1-\cos\frac{2\pi(t-t_0)}{T_g}\Big),\ T_g\in[2,4]\text{ s}
$$

$$
\mathbf w_{turb,k+1}=\mathbf w_{turb,k}-\frac{\mathbf w_{turb,k}}{\tau}\Delta t+\sigma\sqrt{\frac{2\Delta t}{\tau}}\,\boldsymbol\xi_k,\quad \tau=1.5\text{ s},\ \sigma=0.7\text{ m/s},\ \boldsymbol\xi_k\sim\mathcal N(0,I)
$$

*(d) Wind force from the relative air velocity* (added to the built-in drag so that total drag is
$-c_d\lVert\mathbf v-\mathbf w\rVert(\mathbf v-\mathbf w)$):

$$
\mathbf F_{wind}=c_d\Big(\lVert\mathbf v\rVert\,\mathbf v-\lVert\mathbf v-\mathbf w\rVert(\mathbf v-\mathbf w)\Big),\qquad c_d=0.05\ \text{N/(m/s)}^2
$$

A 6 m/s wind therefore pushes with about 1.8 N, 37 % of the drone's weight.

**What each file adds**

* `full_simulation.py` (flight altitude 1.0 m): 3-D window with the zig-zag and then a crossing obstacle; `--headless`
  runs a speed sweep with and without avoidance and saves tables and plots.
* `realistic_simulation.py` (flight altitude 1.5 m): park path with 6 trees, 5 buildings with windows, a shed, a
  patrolling bird, a windsock and wind; buildings are covered with circles (radius $1.15\min(h_x,h_y)$) so the 2-D
  avoidance can use them, trees use the canopy radius $0.7\,s$. In the 3-D window the drone shows a wind arrow, wind
  speed, drone speed and the current decision, and the trees sway.

**Metrics:** collision, goal reached, minimum clearance, flight time, maximum and RMS tilt, maximum altitude error.

| Test | Result |
|---|---|
| Zig-zag, 3 obstacles (PyBullet) | no collision, altitude error $\le0.01$ m |
| Crossing obstacles 0.5–3 m/s, with avoidance (6 runs each) | 0 % collisions at 0.5, 2, 3 m/s; 33 % at 1 m/s (small sample) |
| Realistic world, wind 0 / 2 / 4 m/s (3 runs each) | goal reached without collision, altitude within about 0.03 m |
| Realistic world, wind 6 m/s | collisions occurred; without avoidance every run collided |

---

## IMPLEMENTED, TESTED, AND STILL LIMITED

* **Implemented:** physics with mixer, motor lag, drag and wind; PID with separately logged P, I, D; linear and nonlinear
  system identification; Logistic Regression and Bayesian-network risk models; expected-cost decision; avoidance logic;
  full PyBullet integration; realistic world with wind.
* **Tested:** physics against the analytic model; disturbance rejection; P/PI/PD/PID comparison; identification on
  unseen data; classifier accuracy; obstacles at increasing speeds; wind sweep.
* **Limitations and next steps:** risk labels are rule-based and synthetic (next: label from simulated collisions); the
  Bayesian network is less accurate than Logistic Regression; the planner is reactive (no global planning), so very fast
  obstacles and strong wind can still cause collisions; obstacles are 2-D circles at a fixed altitude; PyBullet sample
  sizes are small; the identified model is not yet used inside the controller (next: model-predictive control).

---

## FILES AND HOW TO RUN

```bash
pip install numpy pandas matplotlib scikit-learn pgmpy pybullet

python physics_model.py              # validation + open-loop test
python self_stabilizing_pid_ml.py    # PID experiments + system identification
python obstacle_avoidance_pgmpy.py   # risk models, zig-zag, obstacle speed sweep
python full_simulation.py            # 3-D PyBullet window (add --headless for plots and metrics only)
python realistic_simulation.py       # 3-D window with trees, buildings and wind (--wind-speed 6 for stronger wind)
```

Options: `--headless`, `--no-show`, `--wind-speed`, `--wind-dir`, `--model lr|bn`, `--speed`, `--eval`, `--snapshot`,
`--opengl2`. All figures are saved in `plots/`.

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
