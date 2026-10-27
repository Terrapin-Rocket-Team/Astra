# Extended Kalman Filter 

# Proposal: Physics-Based Propagation Model for the Flight Kalman Filter

Oct 19, 2026 · @Jack

## 1. Summary

We propose replacing the filter's accelerometer-driven, constant-acceleration propagation with a physics-based model driven by thrust, drag, gravity, and the airbrake as a control input. Thrust comes from our own static fire data and drag from a Mach-dependent drag curve. The filter estimates one drag correction factor, $c$, that absorbs the difference between predicted and actual vehicle drag. The accelerometer moves from driving propagation, where its noise passes straight into the estimates, to measuring the non-gravitational forces, which makes $c$ observable early in coast.

The main payoff is better apogee prediction during coast. That is the quantity the airbrake controller acts on, so filter fidelity there translates directly into control performance.

Scope: navigation filter, all powered and coast phases from ignition through apogee.

## 2. Motivation: Limitations of the Current Model

The current filter drives propagation directly with accelerometer readings, holding each reading constant between updates. No physics model sits behind it, so accelerometer noise and vibration go straight into the velocity and position estimates, and those errors integrate over the flight. The constant-acceleration assumption is also weakest exactly where we need accuracy:

- **Drag scales with $v^2$ and $\rho(h)$.** During coast, deceleration changes continuously. Holding it constant makes the prediction lag reality, so the filter leans on measurement corrections.
- **Transonic $C_D$ changes.** Our vehicle passes through Mach 1, where $C_D$ can change sharply. A constant-acceleration model cannot anticipate this.
- **Airbrake actuation looks like a disturbance.** When the brake deploys, the filter sees an unexplained jump in deceleration and must re-converge through process noise. It cannot tell brake effect from model error.
- **Apogee extrapolation.** Predicting apogee means integrating the dynamics forward many seconds. A constant-acceleration prediction is not physically meaningful over that horizon; a drag model is.

## 3. Proposed Model

### 3.1 State Vector

| State | Symbol | Notes |
| --- | --- | --- |
| Position (NED or local ENU) | $\mathbf{p}$ | 3 states |
| Velocity | $\mathbf{v}$ | 3 states |
| Attitude | $q$ | Quaternion, propagated with gyro rates; use a three-component attitude error for the covariance |
| Accelerometer bias | $\mathbf{b}_a$ | Optional / prospective |
| Drag correction factor | $c$ | Scalar, nominal 1.0, random walk |

The attitude and velocity states we are already adding let drag act along the actual air-relative velocity vector instead of assuming vertical flight.

### 3.2 Inputs

| Input | Source |
| --- | --- |
| Thrust $T(t)$ | Static fire thrust curve of our motor, indexed by time since ignition detection (Get from static fire testing) |
| Mass $m(t)$ | Dry mass + propellant mass depleted in proportion to cumulative impulse from static fire |
| Airbrake deployment $\delta$ | Measured airbrake position (commanded position with an actuator lag model if feedback is unavailable). Depends on intra-rocket communication between the flight computer and the airbrake (see 3.2.1) |
| Body rates $\boldsymbol{\omega}$ | Gyro, for attitude propagation |
| Launch-day atmosphere | Pad pressure and temperature, anchoring the ISA model |

**Why gyro and magnetometer stay inputs:** A rotational dynamics model (moments of inertia, aerodynamic moments) is out of scope for this proposal, so there is no physics prediction of attitude to measure against. The gyro and magnetometer can therefore keep driving attitude as they do now. The accelerometer is deliberately not an input: it becomes a measurement against the translational physics model (see 3.5).

#### 3.2.1 Dependency: Intra-Rocket Communication with the Airbrake

Using airbrake deflection as a control input depends on intra-rocket communication between the flight computer running the filter and the airbrake. The filter needs the deflection value at its update rate, with known or timestamped latency, so the drag term reflects the brake's actual position at each propagation step. If the link drops, lags, or reports a stale position, the filter applies the wrong drag area and pushes the error into $c$ and the velocity estimate.

Requirements this places on the avionics:

- A defined message carrying airbrake deflection (measured position preferred) from the airbrake to the flight computer
- Update rate at or above the filter rate, with timestamps or bounded latency
- A fallback when the link is lost: hold the last known position and inflate velocity process noise, or revert to commanded position plus the actuator lag model

### 3.3 Translational Dynamics

Air-relative velocity and Mach number, with wind $\mathbf{w}$ (zero, or a launch-day estimate):

$$
\mathbf{v}_r = \mathbf{v} - \mathbf{w}, \qquad M = \frac{\|\mathbf{v}_r\|}{a(h)}
$$

Equations of motion:

$$
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{v}} &= \mathbf{g} + \frac{T(t)}{m(t)}\,\hat{\mathbf{x}}_b - \frac{\rho(h)}{2\,m(t)}\,D\,\|\mathbf{v}_r\|\,\mathbf{v}_r \\
\dot{c} &= w_c
\end{aligned}
$$

$\hat{\mathbf{x}}_b$ is the unit vector along the rocket's body axis, expressed in the navigation frame. Rotate the body-frame unit vector through the body-to-navigation rotation $R(q)$:

$$
\hat{\mathbf{x}}_b = R(q)\mathbf{e}_x,
\qquad \mathbf{e}_x = \begin{bmatrix}1 & 0 & 0\end{bmatrix}^{\top}.
$$

This gives thrust a direction. The motor pushes along the airframe, and the airframe isn't necessarily pointed straight up or along the velocity vector. Use the same navigation frame for $\mathbf{p}$, $\mathbf{v}$, $\mathbf{w}$, and $\mathbf{g}$. Altitude $h$ increases upward, so it increases with the vertical position in ENU and decreases with it in NED.

The total drag area is:

$$
D = c \cdot C_{D,\text{body}}(M)\,A_{\text{ref}} + C_{D,\text{ab}}(M,\delta)\,A_{\text{ab}}(\delta)
$$

- $C_{D,\text{body}}(M)$: lookup table from RASAero/OpenRocket for the airbrake-retracted vehicle.
- $C_{D,\text{ab}} A_{\text{ab}}$: lookup table from our airbrake wind tunnel testing, vs. deployment (and Mach where data exists).
- $\rho(h)$, $a(h)$: ISA, offset to launch-day pad pressure and temperature.

**Why a correction factor instead of a raw ballistic coefficient:** $C_D$ varies strongly with Mach on our vehicle, especially through transonic. A single ballistic coefficient state would spend coast chasing that variation instead of converging. Scaling a known drag curve with $c$ keeps the physics in the table and leaves the filter to estimate only true model error (surface finish, build differences, sim error). $c$ effectively scales $1/\beta_{\text{body}}$, and drag is linear in it, which keeps the Jacobian well-conditioned.

**Why the airbrake is a separate input:** If the brake were folded into the same coefficient, every actuation would look like a sudden change in $c$. Treating deployment as a known input lets the filter anticipate the added drag and keeps $c$ steady through actuation.

### 3.4 Flight Phases

The same free-flight equations cover boost and coast; only the inputs change. Phase transitions use existing event detection (ignition, burnout). On the pad, launch support constrains position and velocity; hold them fixed until release rather than integrating the free-flight equations.

| Phase | Thrust | Mass | Airbrake | $c$ process noise |
| --- | --- | --- | --- | --- |
| Pad | 0 | $m_0$ | Retracted | 0 (frozen) |
| Boost | Static fire curve | Depleting | Retracted | Low or frozen (see 3.6) |
| Coast | 0 | $m_{\text{dry}}$ | Active, $\delta$ input | Nominal |
| Apogee onward | Out of scope |  |  |  |

### 3.5 Measurement Models

**Accelerometer (specific force, body frame).** An accelerometer measures non-gravitational forces only:

$$
\mathbf{f}_{\text{meas}} = R(q)^{\top} \left[ \frac{T(t)}{m(t)}\,\hat{\mathbf{x}}_b - \frac{\rho}{2m}\,D\,\|\mathbf{v}_r\|\,\mathbf{v}_r \right] + \mathbf{b}_a + \mathbf{n}_a
$$

During coast this is a direct measurement of drag, so it gives direct information about $c$. This is the key structural change: the accelerometer stops driving propagation and starts correcting the physics model.

The bias term $\mathbf{b}_a$ is optional and prospective, matching its status in 3.1. If the bias state is not carried, drop $\mathbf{b}_a$ from this equation and absorb any residual bias into the accelerometer measurement noise $\mathbf{n}_a$.

**Barometer:**

$$
h_{\text{meas}} = h + n_h
$$

Static port pressure errors through transonic flight are not modeled, so for now the filter ignores barometer measurements in the transonic window (Mach bounds TBD). The physics model and accelerometer carry the altitude and velocity estimates through that window, and baro updates resume once the vehicle is back in the subsonic range.

**GPS** (if used): unchanged from the current filter.

### 3.6 Thrust Uncertainty

Static fire data defines the nominal thrust curve, but flight motors vary (grain temperature, casting variation). During boost:

- Set velocity process noise from the motor-to-motor spread seen across our static fires.
- Keep $c$ frozen or very low noise. Thrust uncertainty dominates drag in this phase, so a free $c$ would absorb thrust error instead of drag error.
- Optional later extension: a thrust scale factor state, if static fire repeatability shows it is worth it.

## 4. EKF Implementation

The model is nonlinear, so the filter becomes an EKF (a UKF is an option if hand-derived Jacobians become a burden).

**State propagation:** RK4 (or sub-stepped Euler) on the full nonlinear equations at the filter rate. (Claude suggestion, haven't looked into)

**Covariance propagation:** Use the dynamics Jacobian $A = \partial \dot{x}/\partial x$, consistent with the notation in the hybrid EKF notes below. For a sufficiently small interval, the first-order transition is $\Phi_k \approx I + A(t_k)\Delta t$. The drag-related blocks below hold wind fixed and ignore the Mach dependence of both drag coefficients during differentiation; the nonlinear mean propagation still uses the Mach-dependent tables. These blocks apply when $\|\mathbf{v}_r\| > 0$:

$$
\begin{aligned}
\mathbf{a}_d &= -\frac{\rho D}{2m}\|\mathbf{v}_r\|\,\mathbf{v}_r \\
\frac{\partial \mathbf{a}_d}{\partial \mathbf{v}} &= -\frac{\rho D}{2m}\left(\|\mathbf{v}_r\|\,I + \frac{\mathbf{v}_r\mathbf{v}_r^{\top}}{\|\mathbf{v}_r\|}\right) \\
\frac{\partial \mathbf{a}_d}{\partial h} &= -\frac{D}{2m}\frac{d\rho}{dh}\|\mathbf{v}_r\|\,\mathbf{v}_r \\
\frac{\partial \mathbf{a}_d}{\partial c} &= -\frac{\rho}{2m}\,C_{D,\text{body}}(M)\,A_{\text{ref}}\,\|\mathbf{v}_r\|\,\mathbf{v}_r
\end{aligned}
$$

At zero air-relative speed, set $\mathbf{a}_d$ and the displayed drag Jacobian blocks to zero using their continuous limits, rather than dividing by $\|\mathbf{v}_r\| = 0$.

Attitude blocks must use the same three-component attitude-error coordinates as the covariance, including the thrust term's dependence on attitude through $\hat{\mathbf{x}}_b$ during boost.

**Second pass:** include $\partial C_D/\partial M$ from the table slopes, including the altitude dependence of Mach through $a(h)$, if transonic performance shows the frozen-coefficient Jacobian is insufficient.

**Apogee prediction:** forward-integrate the same dynamics from the current estimate (with the planned brake schedule) to zero vertical velocity. The controller can use this directly (if it has access, we could run this same filter on their computer too).

## 5. Observability and Tuning

- $c$ is observable when drag is significant, meaning high velocity in early coast. That is exactly when apogee prediction matters most. Near apogee $c$ becomes unobservable and holds its value, which is harmless.
- Initialize $c = 1.0$ with a prior standard deviation reflecting confidence in the drag curve (starting point: 10–20%).
- $c$ and airbrake-model error are only separable while the brake is moving. We estimate one factor for now and treat brake-table error as measurement/process noise.
- Process noise on $c$ stays small, since vehicle drag characteristics should not change during flight.

## 6. Data Required

| Item | Source | Status |
| --- | --- | --- |
| Body $C_D$ vs. Mach | RASAero / OpenRocket | TBD |
| Airbrake $C_D A$ vs. deployment | Team wind tunnel testing | Exists; confirm Mach range covered |
| Thrust curve(s) | Static fire data | Exists; pull and format |
| Propellant mass, dry mass, $A_{\text{ref}}$ | Vehicle mass budget | TBD |
| Airbrake position feedback | Avionics | Confirm availability |
| Airbrake ↔ flight computer communication link | Avionics | Define message, rate, latency, and dropout behavior |

## 7. Validation Plan

1. **Simulation with truth.** Generate trajectories from OpenRocket/RASAero, add sensor noise matching our hardware, and perturb the true drag (e.g. $C_D \times 1.15$) and thrust (static fire spread).
   1. We could also try to make our own sim (GNC sub team plug)
2. **Head-to-head.** Run the current constant-acceleration filter and the proposed EKF on identical data. Metrics:
   - Apogee prediction error vs. time-to-apogee
   - Velocity and altitude RMSE by phase
   - Convergence time of $c$
   - Filter consistency (NEES / NIS)
3. **Airbrake scenarios.** Repeat with scripted brake deployments to confirm $c$ stays steady through actuation.
4. **Flight log replay.** Run both filters on previous flight data and compare against recorded apogee.
5. **Hardware-in-the-loop.** Confirm the EKF meets loop timing on the flight computer.

## 8. Risks

| Risk | Mitigation |
| --- | --- |
| Drag table wrong in transonic | $c$ absorbs scale error; baro ignored through transonic; validate in sim with perturbed tables |
| Airbrake data outside tested Mach range | Inflate uncertainty or restrict deployment envelope (see Section 6) |
| Flight motor differs from static fire | Velocity process noise from static fire spread; $c$ frozen during boost |
| Loss or delay of airbrake deflection data over the intra-rocket link | Timestamped messages; hold last value and inflate process noise on dropout; fall back to commanded position with lag model |
| Wind / angle of attack | Air-relative velocity; small-AoA assumption stated; normal force can be added later |
| Computational load | Low state dimension; table lookups are cheap; fall back to fixed-gain or reduced-order if needed |
| Jacobian errors | Unit test Jacobians against finite differences |

## 9. Implementation Plan

1. Build lookup tables (body $C_D$, airbrake $C_D A$, thrust, mass) and the ISA model.
2. Implement nonlinear propagation and verify against OpenRocket with zero noise.
3. Implement EKF covariance propagation and measurement models; unit test Jacobians against finite differences.
4. Run the simulation comparison (Section 7, steps 1–3) and tune.
5. Replay flight logs.
6. Port to flight software and run HIL.

## 10. References

- Team airbrake wind tunnel test report — TBD
- Team static fire data — TBD

# Hybrid EKF — Study Notes and Pseudocode

## 1. LTI vs. LTV Systems

A **linear time-invariant (LTI)** system has a constant dynamics matrix $A$, while a **linear time-varying (LTV)** system has a matrix $A(t)$ that can change with time. Both describe linear dynamics, but an LTV model can describe changing behavior along a trajectory. For an LTI system, the state transition matrix (STM) is $\Phi(t,t_0)=e^{A(t-t_0)}$. For a general LTV system, we instead solve a matrix differential equation to find the STM.

$$
\dot{x}=A(t)x+B(t)u
$$

## 2. State Transition Matrix (STM)

The **STM** maps the state of a homogeneous linear system, or a small state error in a linearized system, from one time to another. For an LTV system, its differential equation is $\dot\Phi(t,t_k)=A(t)\Phi(t,t_k)$, with initial condition $\Phi(t_k,t_k)=I$. We integrate this equation over the prediction interval $[t_k,t_{k+1}]$ to obtain a new $\Phi_k$ for that interval. The STM is **not the derivative of $A$**: its derivative is $A(t)\Phi$, and for small timesteps we can approximate $\Phi_k\approx I+A(t_k)\Delta t$.

$$
\boxed{
\begin{aligned}
\dot\Phi(t,t_k) &= A(t)\Phi(t,t_k), \\
\Phi(t_k,t_k) &= I, \\
\Phi_k &= \Phi(t_{k+1},t_k).
\end{aligned}
}
$$

## 3. Nonlinear Dynamics, the Jacobian, and the Nominal Trajectory

The state vector $x$ contains quantities such as position and velocity, whereas $f$ is the vector of **physics equations for their time derivatives**: $\dot{x}=f(x,u,w,t)$. We compute the dynamics Jacobian $A=\partial f/\partial x$ by differentiating each component of $f$ with respect to each state. The Jacobian is evaluated along a **nominal or estimated trajectory**, so its numerical entries can change as the rocket moves. That time-varying Jacobian is what defines the LTV error-dynamics model used by the EKF.

$$
\boxed{A(t)=\left.\frac{\partial f}{\partial x}\right|_{\hat x(t),\,u(t),\,w=0}}
$$

## 4. Nominal Trajectory: Linearized KF vs. EKF

A traditional **linearized Kalman filter** can linearize a nonlinear model around a separately known nominal trajectory. The **EKF** instead uses its own current corrected state estimate as the starting point for the next predicted trajectory. It propagates that estimate using nonlinear physics, evaluates Jacobians along the prediction, and corrects the result when measurements arrive. Ordinary Kalman filters also reuse previous estimates; the special feature of the EKF is **relinearizing around its evolving estimate**.

$$
\hat x_k^+ \xrightarrow{\text{nonlinear prediction}} \hat x_{k+1}^- \xrightarrow{\text{measurement correction}} \hat x_{k+1}^+
$$

## 5. The Three Covariance Matrices: $P$, $Q$, and $R$

**$P$** is the covariance of state-estimation errors, **$Q$** describes process disturbances and uncertainty in the dynamics, and **$R$** describes measurement noise from sensors. A covariance matrix has variances on its diagonal and covariances between different error components off the diagonal. For uncorrelated sensor measurement errors, $R$ is diagonal; correlated errors give nonzero off-diagonal values, which can be estimated from synchronized measurement-error data. In continuous-time dynamics, the process noise is described by a continuous-time covariance **intensity** $Q_c$, which must be accumulated over an interval to obtain discrete process covariance $Q_{d,k}$.

$Q_c(t)$ is treated as a known input to covariance propagation. It may be constant or scheduled by flight phase and is chosen from modeling assumptions, test data, and tuning rather than calculated from a Jacobian. When integrating $\dot{P}=AP+PA^{\top}+LQ_cL^{\top}$ directly, process uncertainty accumulates within $P$, so no STM or separate discrete process covariance $Q_{d,k}$ is required.

## 6. How the STM Relates to $P$, $Q$, and $R$

In the EKF, the STM is **primarily used for propagating $P$** and, when discretizing continuous process noise, for calculating $Q_{d,k}$. The state covariance propagates as $P^-_{k+1}=\Phi_kP_k^+\Phi_k^T+Q_{d,k}$, where $Q_{d,k}$ represents disturbances accumulated over that interval. The same STM also maps small state deviations, but **we do not propagate the sensor noise covariance $R$ using the STM**. Alternatively, we can integrate $\dot P=AP+PA^T+LQ_cL^T$ directly and avoid explicitly calculating $\Phi$ and $Q_{d,k}$.

$$
\boxed{P^-_{k+1}=\Phi_kP_k^+\Phi_k^T+Q_{d,k}}
$$

$$
Q_{d,k}=\int_{t_k}^{t_{k+1}}\Phi(t_{k+1},\tau)L(\tau)Q_c(\tau)L(\tau)^T\Phi(t_{k+1},\tau)^T\,d\tau
$$

## 7. The Four System and Noise Jacobians: $A$, $L$, $C$, $M$

**$A=\partial f/\partial x$** tells us how the states affect the dynamics, and **$L=\partial f/\partial w$** tells us how process noise enters those dynamics. **$C=\partial h/\partial x$** describes how states affect predicted sensor readings (often called $H$), while **$M=\partial h/\partial v$** describes how measurement noise enters those readings. These roles also exist in linear Kalman filters; when process and measurement noise are directly additive with matching dimensions, **$L=I$ and $M=I$**, but that must be verified from the model rather than assumed.

$$
\dot x=f(x,u,w,t),\qquad y_k=h_k(x_k,v_k)
$$

## 8. Why We Use a Hybrid EKF

A **hybrid (continuous-discrete) EKF** is appropriate when a rocket's dynamics evolve continuously but IMU, GPS, or barometer readings arrive at discrete times. Between measurements, we integrate the nonlinear state estimate and propagate $P$ using either its differential equation or the STM-based method. At a measurement timestamp, we evaluate $C$ and $M$, use the measurement noise covariance $R_k$, and perform a discrete Kalman correction. The measurement is **not** its expected value multiplied by $\Delta t$; instead, we compare the actual reading with the predicted reading to form a residual.

$$
\boxed{
\begin{aligned}
\dot{\hat x} &= f(\hat x,u,0,t), \\
\dot P &= AP+PA^T+LQ_cL^T.
\end{aligned}
}
$$

$$
\boxed{r_k=y_k-h_k(\hat x_k^-,0)}
$$

## 9. Discrete Measurement Correction

The measurement Jacobian $C_k$ and effective measurement covariance $M_kR_kM_k^T$ determine how strongly the EKF should trust a new reading. We calculate the innovation covariance $S_k$, then the Kalman gain $K_k$, and use that gain to correct the predicted state. The covariance is also corrected because the new measurement generally reduces uncertainty in some directions. Afterward, the corrected state and covariance become the initial conditions for the next prediction interval.

$$
S_k=C_kP_k^-C_k^T+M_kR_kM_k^T
$$

$$
\boxed{
\begin{aligned}
K_k &= P_k^-C_k^TS_k^{-1}, \\
\hat x_k^+ &= \hat x_k^-+K_kr_k.
\end{aligned}
}
$$

---

## Hybrid EKF Pseudocode — STM-Based Prediction

The equations below use Euclidean state coordinates and assume that process noise and measurement noise are independent, with measurement noise uncorrelated with the predicted state error. Here $w$ and $v$ denote process and measurement noise; they are distinct from the wind $\mathbf{w}$ and velocity $\mathbf{v}$ in the proposal.

For the proposal's quaternion attitude, use a three-component attitude error in the covariance and derive $A$, $L$, and $C$ in those error coordinates. Inject its correction by quaternion multiplication, normalize the quaternion, then reset the attitude error and transform the covariance with the reset Jacobian. The additive state update below applies to Euclidean components; see [Solà, Sections 6.2–6.3](https://arxiv.org/pdf/1711.02508#page=64) for quaternion error injection and reset.

### Initialization

1. **Define the state vector $x$ and the continuous nonlinear dynamics:**

    $$
    \dot{x} = f(x,u,w,t).
    $$

    Here $f$ is the vector of derivative equations, **NOT** the state vector $x$.

2. **Define a measurement model for each sensor:**

    $$
    y = h_{\text{sensor}}(x,v,t).
    $$

3. **Derive or implement the Jacobians:**

    $$
    \begin{aligned}
    A &= \frac{\partial f}{\partial x} && \text{dynamics Jacobian}, \\
    L &= \frac{\partial f}{\partial w} && \text{process-noise mapping}, \\
    C &= \frac{\partial h}{\partial x} && \text{measurement Jacobian (also called } H\text{)}, \\
    M &= \frac{\partial h}{\partial v} && \text{measurement-noise mapping}.
    \end{aligned}
    $$

    Evaluate each Jacobian at the applicable estimated state and noise mean. For directly additive noise with matching dimensions, $L$ or $M$ may equal $I$.

4. **Define noise models:**

    $$
    \begin{aligned}
    Q_c(t) &\quad \text{continuous-time process noise intensity}, \\
    R_{\text{sensor}} &\quad \text{discrete measurement noise covariance for each sensor}.
    \end{aligned}
    $$

5. **Initialize:**

    $$
    \begin{aligned}
    \hat{x}^{+} &\quad \text{initial corrected state estimate}, \\
    P^{+} &\quad \text{initial state-error covariance}, \\
    t_{\text{prev}} &\quad \text{initial time}.
    \end{aligned}
    $$

For each new sensor measurement $(y,\text{sensor\_id},t_{\text{next}})$:

### Prediction: Propagate from $t_{\text{prev}}$ to $t_{\text{next}}$

The noise means $\bar{w}$ and $\bar{v}$ below correspond to `w_mean` and `v_mean` in the pseudocode.

6. **Initialize the interval STM and accumulated process covariance:**

    $$
    \hat{x}(t_{\text{prev}}) = \hat{x}^{+},
    \qquad \Phi(t_{\text{prev}}) = I,
    \qquad Q_d(t_{\text{prev}}) = 0.
    $$

7. **Numerically integrate the coupled equations to $t_{\text{next}}$:**

    $$
    \begin{aligned}
    \dot{\hat{x}} &= f(\hat{x},u,\bar{w},t), \\
    A(t) &= \left.\frac{\partial f}{\partial x}\right|_{\hat{x}(t),\bar{w}}, \\
    L(t) &= \left.\frac{\partial f}{\partial w}\right|_{\hat{x}(t),\bar{w}}, \\
    \dot{\Phi} &= A(t)\Phi, \\
    \dot{Q}_d &= A(t)Q_d + Q_d A(t)^{\top} + L(t)Q_c(t)L(t)^{\top}.
    \end{aligned}
    $$

    Evaluate $A(t)$ and $L(t)$ along the predicted $\hat{x}(t)$. The $Q_d$ equation accumulates the same process uncertainty represented by the interval integral involving the STM at intermediate times.

8. **Obtain the predicted state and covariance at $t_{\text{next}}$:**

    $$
    \begin{aligned}
    \hat{x}^{-} &= \hat{x}(t_{\text{next}}), \\
    P^{-} &= \Phi P^{+}\Phi^{\top} + Q_d.
    \end{aligned}
    $$

### Measurement Update: Apply the New Discrete Sensor Reading

9. **Predict the sensor measurement:**

    $$
    \hat{y} = h_{\text{sensor}}(\hat{x}^{-},\bar{v},t_{\text{next}}).
    $$

10. **Evaluate the measurement Jacobians and choose that sensor's $R$:**

    $$
    \begin{aligned}
    C &= \left.\frac{\partial h_{\text{sensor}}}{\partial x}\right|_{\hat{x}^{-},\bar{v},t_{\text{next}}}, \\
    M &= \left.\frac{\partial h_{\text{sensor}}}{\partial v}\right|_{\hat{x}^{-},\bar{v},t_{\text{next}}}, \\
    R &= R_{\text{sensor}}.
    \end{aligned}
    $$

11. **Compute the innovation (measurement residual):**

    $$
    r = y - \hat{y}.
    $$

12. **Compute innovation covariance:**

    $$
    S = C P^{-} C^{\top} + M R M^{\top}.
    $$

13. **Calculate Kalman gain (in code, solve $S$ rather than explicitly invert):**

    $$
    K = P^{-} C^{\top} S^{-1}.
    $$

14. **Correct state:**

    $$
    \hat{x}^{+} = \hat{x}^{-} + K r.
    $$

15. **Correct covariance using the numerically robust Joseph form:**

    $$
    \begin{aligned}
    J &= I - K C, \\
    P^{+} &= J P^{-} J^{\top} + K\left(M R M^{\top}\right)K^{\top}.
    \end{aligned}
    $$

16. **Set $t_{\text{prev}} = t_{\text{next}}$ and repeat for the next sensor measurement.**

## Alternative Pseudocode — Direct Covariance Integration

Instead of prediction steps 6–8, integrate the state and covariance together, starting from $\hat{x}(t_{\text{prev}}) = \hat{x}^{+}$ and $P(t_{\text{prev}}) = P^{+}$:

$$
\begin{aligned}
\dot{\hat{x}} &= f(\hat{x},u,\bar{w},t), \\
A(t) &= \left.\frac{\partial f}{\partial x}\right|_{\hat{x}(t),\bar{w}}, \\
L(t) &= \left.\frac{\partial f}{\partial w}\right|_{\hat{x}(t),\bar{w}}, \\
\dot{P} &= A(t)P + P A(t)^{\top} + L(t)Q_c(t)L(t)^{\top}.
\end{aligned}
$$

Evaluate $A(t)$ and $L(t)$ along the predicted state trajectory.

At $t_{\text{next}}$:

$$
\hat{x}^{-} = \hat{x}(t_{\text{next}}),
\qquad P^{-} = P(t_{\text{next}}).
$$

Then use the SAME discrete measurement update (steps 9–16).
Do not apply both covariance prediction methods over the same interval.
