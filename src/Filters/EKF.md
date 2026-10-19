# Extended Kalman Filter 

# Proposal: Physics-Based Propagation Model for the Flight Kalman Filter

Oct 19, 2026 · @Jack

## 1. Summary

We propose replacing the filter's accelerometer-driven, constant-acceleration propagation with a physics-based model driven by thrust, drag, gravity, and the airbrake as a control input. Thrust comes from our own static fire data and drag from a Mach-dependent drag curve. The filter estimates one drag correction factor, c, that absorbs the difference between predicted and actual vehicle drag. The accelerometer moves from driving propagation, where its noise passes straight into the estimates, to measuring the non-gravitational forces, which makes c observable early in coast.

The main payoff is better apogee prediction during coast. That is the quantity the airbrake controller acts on, so filter fidelity there translates directly into control performance.

Scope: navigation filter, all powered and coast phases from ignition through apogee.

## 2. Motivation: Limitations of the Current Model

The current filter drives propagation directly with accelerometer readings, holding each reading constant between updates. No physics model sits behind it, so accelerometer noise and vibration go straight into the velocity and position estimates, and those errors integrate over the flight. The constant-acceleration assumption is also weakest exactly where we need accuracy:

- **Drag scales with v² and ρ(h).** During coast, deceleration changes continuously. Holding it constant makes the prediction lag reality, so the filter leans on measurement corrections.
- **Transonic C\_D changes.** Our vehicle passes through Mach 1, where C\_D can change sharply. A constant-acceleration model cannot anticipate this.
- **Airbrake actuation looks like a disturbance.** When the brake deploys, the filter sees an unexplained jump in deceleration and must re-converge through process noise. It cannot tell brake effect from model error.
- **Apogee extrapolation.** Predicting apogee means integrating the dynamics forward many seconds. A constant-acceleration prediction is not physically meaningful over that horizon; a drag model is.

## 3. Proposed Model

### 3.1 State Vector

| State | Symbol | Notes |
| --- | --- | --- |
| Position (NED or local ENU) | p | 3 states |
| Velocity | v | 3 states |
| Attitude | q | Quaternion, propagated with gyro rates (error-state handling as in current filter) |
| Accelerometer bias | b\_a | Optional / prospective |
| Drag correction factor | c | Scalar, nominal 1.0, random walk |

The attitude and velocity states we are already adding let drag act along the actual air-relative velocity vector instead of assuming vertical flight.

### 3.2 Inputs

| Input | Source |
| --- | --- |
| Thrust T(t) | Static fire thrust curve of our motor, indexed by time since ignition detection (Get from static fire testing) |
| Mass m(t) | Dry mass + propellant mass depleted in proportion to cumulative impulse from static fire |
| Airbrake deployment δ | Measured airbrake position (commanded position with an actuator lag model if feedback is unavailable). Depends on intra-rocket communication between the flight computer and the airbrake (see 3.2.1) |
| Body rates ω | Gyro, for attitude propagation |
| Launch-day atmosphere | Pad pressure and temperature, anchoring the ISA model |

**Why gyro and magnetometer stay inputs:** A rotational dynamics model (moments of inertia, aerodynamic moments) is out of scope for this proposal, so there is no physics prediction of attitude to measure against. The gyro and magnetometer can therefore keep driving attitude as they do now. The accelerometer is deliberately not an input: it becomes a measurement against the translational physics model (see 3.5).

#### 3.2.1 Dependency: Intra-Rocket Communication with the Airbrake

Using airbrake deflection as a control input depends on intra-rocket communication between the flight computer running the filter and the airbrake. The filter needs the deflection value at its update rate, with known or timestamped latency, so the drag term reflects the brake's actual position at each propagation step. If the link drops, lags, or reports a stale position, the filter applies the wrong drag area and pushes the error into c and the velocity estimate.

Requirements this places on the avionics:

- A defined message carrying airbrake deflection (measured position preferred) from the airbrake to the flight computer
- Update rate at or above the filter rate, with timestamps or bounded latency
- A fallback when the link is lost: hold the last known position and inflate velocity process noise, or revert to commanded position plus the actuator lag model

### 3.3 Translational Dynamics

Air-relative velocity and Mach number, with wind w (zero, or a launch-day estimate):

```math
\mathbf{v}_r = \mathbf{v} - \mathbf{w}, \qquad M = \frac{\|\mathbf{v}_r\|}{a(h)}
```

Equations of motion:

```math
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{v}} &= \mathbf{g} + \frac{T(t)}{m(t)}\,\hat{\mathbf{x}}_b - \frac{\rho(h)}{2\,m(t)}\,D\,\|\mathbf{v}_r\|\,\mathbf{v}_r \\
\dot{c} &= w_c
\end{aligned}
```

x̂\_b is the unit vector along the rocket's body axis, expressed in the navigation frame. You get it by rotating the body-frame unit vector \[1, 0, 0\] through the current attitude estimate: x̂\_b = R(q)·e\_x. This gives thrust a direction. The motor pushes along the airframe, and the airframe isn't necessarily pointed straight up or along the velocity vector.

The total drag area is:

```math
D = c \cdot C_{D,\text{body}}(M)\,A_{\text{ref}} + C_{D,\text{ab}}(M,\delta)\,A_{\text{ab}}(\delta)
```

- C\_D,body(M): lookup table from RASAero/OpenRocket for the airbrake-retracted vehicle.
- C\_D,ab · A\_ab: lookup table from our airbrake wind tunnel testing, vs. deployment (and Mach where data exists).
- ρ(h), a(h): ISA, offset to launch-day pad pressure and temperature.

**Why a correction factor instead of a raw ballistic coefficient:** C\_D varies strongly with Mach on our vehicle, especially through transonic. A single ballistic coefficient state would spend coast chasing that variation instead of converging. Scaling a known drag curve with c keeps the physics in the table and leaves the filter to estimate only true model error (surface finish, build differences, sim error). c effectively scales 1/β\_body, and drag is linear in it, which keeps the Jacobian well-conditioned.

**Why the airbrake is a separate input:** If the brake were folded into the same coefficient, every actuation would look like a sudden change in c. Treating deployment as a known input lets the filter anticipate the added drag and keeps c steady through actuation.

### 3.4 Flight Phases

The same equations cover every phase; only the inputs change. Phase transitions use existing event detection (ignition, burnout).

| Phase | Thrust | Mass | Airbrake | c process noise |
| --- | --- | --- | --- | --- |
| Pad | 0 | m₀ | Retracted | 0 (frozen) |
| Boost | Static fire curve | Depleting | Retracted | Low or frozen (see 3.6) |
| Coast | 0 | m\_dry | Active, δ input | Nominal |
| Apogee onward | Out of scope |  |  |  |

### 3.5 Measurement Models

**Accelerometer (specific force, body frame).** An accelerometer measures non-gravitational forces only:

```math
\mathbf{f}_{\text{meas}} = R(\mathbf{q})^T \left[ \frac{T(t)}{m(t)}\,\hat{\mathbf{x}}_b - \frac{\rho}{2m}\,D\,\|\mathbf{v}_r\|\,\mathbf{v}_r \right] + \mathbf{b}_a + \mathbf{n}_a
```

During coast this is a direct measurement of drag, so it gives direct information about c. This is the key structural change: the accelerometer stops driving propagation and starts correcting the physics model.

The bias term b\_a is optional and prospective, matching its status in 3.1. If the bias state is not carried, drop b\_a from this equation and absorb any residual bias into the accelerometer measurement noise n\_a.

**Barometer:**

```math
h_{\text{meas}} = h + n_h
```

Static port pressure errors through transonic flight are not modeled, so for now the filter ignores barometer measurements in the transonic window (Mach bounds TBD). The physics model and accelerometer carry the altitude and velocity estimates through that window, and baro updates resume once the vehicle is back in the subsonic range.

**GPS** (if used): unchanged from the current filter.

### 3.6 Thrust Uncertainty

Static fire data defines the nominal thrust curve, but flight motors vary (grain temperature, casting variation). During boost:

- Set velocity process noise from the motor-to-motor spread seen across our static fires.
- Keep c frozen or very low noise. Thrust uncertainty dominates drag in this phase, so a free c would absorb thrust error instead of drag error.
- Optional later extension: a thrust scale factor state, if static fire repeatability shows it is worth it.

## 4. EKF Implementation

The model is nonlinear, so the filter becomes an EKF (a UKF is an option if hand-derived Jacobians become a burden).

**State propagation:** RK4 (or sub-stepped Euler) on the full nonlinear equations at the filter rate. (Claude suggestion, haven't looked into)

**Covariance propagation:** Jacobian F = ∂ẋ/∂x, discretized as Φ ≈ I + FΔt. Drag-related blocks, with the drag acceleration defined below and ∂C\_D/∂M ignored on the first pass (haven't had time to look this part over):

```math
\begin{aligned}
\mathbf{a}_d &= -\frac{\rho D}{2m}\|\mathbf{v}_r\|\,\mathbf{v}_r \\
\frac{\partial \mathbf{a}_d}{\partial \mathbf{v}} &= -\frac{\rho D}{2m}\left(\|\mathbf{v}_r\|\,I + \frac{\mathbf{v}_r\mathbf{v}_r^T}{\|\mathbf{v}_r\|}\right) \\
\frac{\partial \mathbf{a}_d}{\partial h} &= -\frac{D}{2m}\frac{d\rho}{dh}\|\mathbf{v}_r\|\,\mathbf{v}_r \\
\frac{\partial \mathbf{a}_d}{\partial c} &= -\frac{\rho}{2m}\,C_{D,\text{body}}(M)\,A_{\text{ref}}\,\|\mathbf{v}_r\|\,\mathbf{v}_r
\end{aligned}
```

Attitude blocks are unchanged from the current attitude filter, plus the thrust term's dependence on attitude through x̂\_b during boost.

**Second pass:** include ∂C\_D/∂M from the table slope if transonic performance shows the first-order Jacobian is insufficient.

**Apogee prediction:** forward-integrate the same dynamics from the current estimate (with the planned brake schedule) to zero vertical velocity. The controller can use this directly (if it has access, we could run this same filter on their computer too).

## 5. Observability and Tuning

- c is observable when drag is significant, meaning high velocity in early coast. That is exactly when apogee prediction matters most. Near apogee c becomes unobservable and holds its value, which is harmless.
- Initialize c = 1.0 with a prior standard deviation reflecting confidence in the drag curve (starting point: 10–20%).
- c and airbrake-model error are only separable while the brake is moving. We estimate one factor for now and treat brake-table error as measurement/process noise.
- Process noise on c stays small, since vehicle drag characteristics should not change during flight.

## 6. Data Required

| Item | Source | Status |
| --- | --- | --- |
| Body C\_D vs. Mach | RASAero / OpenRocket | TBD |
| Airbrake C\_D·A vs. deployment | Team wind tunnel testing | Exists; confirm Mach range covered |
| Thrust curve(s) | Static fire data | Exists; pull and format |
| Propellant mass, dry mass, A\_ref | Vehicle mass budget | TBD |
| Airbrake position feedback | Avionics | Confirm availability |
| Airbrake ↔ flight computer communication link | Avionics | Define message, rate, latency, and dropout behavior |

## 7. Validation Plan

1. **Simulation with truth.** Generate trajectories from OpenRocket/RASAero, add sensor noise matching our hardware, and perturb the true drag (e.g. C\_D × 1.15) and thrust (static fire spread).
   1. We could also try to make our own sim (GNC sub team plug)
2. **Head-to-head.** Run the current constant-acceleration filter and the proposed EKF on identical data. Metrics:
   - Apogee prediction error vs. time-to-apogee
   - Velocity and altitude RMSE by phase
   - Convergence time of c
   - Filter consistency (NEES / NIS)
3. **Airbrake scenarios.** Repeat with scripted brake deployments to confirm c stays steady through actuation.
4. **Flight log replay.** Run both filters on previous flight data and compare against recorded apogee.
5. **Hardware-in-the-loop.** Confirm the EKF meets loop timing on the flight computer.

## 8. Risks

| Risk | Mitigation |
| --- | --- |
| Drag table wrong in transonic | c absorbs scale error; baro ignored through transonic; validate in sim with perturbed tables |
| Airbrake data outside tested Mach range | Inflate uncertainty or restrict deployment envelope (see Section 6) |
| Flight motor differs from static fire | Velocity process noise from static fire spread; c frozen during boost |
| Loss or delay of airbrake deflection data over the intra-rocket link | Timestamped messages; hold last value and inflate process noise on dropout; fall back to commanded position with lag model |
| Wind / angle of attack | Air-relative velocity; small-AoA assumption stated; normal force can be added later |
| Computational load | Low state dimension; table lookups are cheap; fall back to fixed-gain or reduced-order if needed |
| Jacobian errors | Unit test Jacobians against finite differences |

## 9. Implementation Plan

1. Build lookup tables (body C\_D, airbrake C\_D·A, thrust, mass) and the ISA model.
2. Implement nonlinear propagation and verify against OpenRocket with zero noise.
3. Implement EKF covariance propagation and measurement models; unit test Jacobians against finite differences.
4. Run the simulation comparison (Section 7, steps 1–3) and tune.
5. Replay flight logs.
6. Port to flight software and run HIL.

## 10. References

- Team airbrake wind tunnel test report — TBD
- Team static fire data — TBD
