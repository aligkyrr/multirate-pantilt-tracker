# Multi-Rate Pan-Tilt Target Tracking System

**A real-time, multi-rate control simulation of a two-axis pan-tilt tracking platform.**
PyQt5-based UI, OpenGL 3D visualization, closed-loop PID control, and Kalman-filter-based target state estimation.

---

## Overview

This project is a control and simulation system that models a two-axis (azimuth / elevation) pan-tilt platform tracking moving targets in real time. The goal is not a visual demo — it's an accurate software-level model of the problems real-time tracking systems actually face: synchronizing subsystems running at different frequencies, closed-loop control stability, chatter-free mode/lock transitions, target state estimation under measurement noise, and target loss / re-acquisition behavior.

The system is built on three independent fixed-timestep loops and is fully parametrized through `config.py`; no control or timing constant is hardcoded anywhere in the codebase.

---

## Control System — Technical Details

### Multi-Rate Loop Architecture

| Loop | Frequency | Responsibility |
|---|---|---|
| Target Update | 60 Hz | Target physics (OU-type smooth random walk, boundary bounce) |
| Control Loop (PID) | 120 Hz | Angular error computation, PID output generation |
| Pan-Tilt Update | 60 Hz | Angle/velocity/acceleration integration, actuator physics |

Running the control loop at twice the frequency of the other two is a deliberate design choice: sampling the controller more often than the actuation and target physics update produces a more stable, lower-latency PID output.

Timing is driven by an accumulator model advancing in fixed `dt` steps (`DT_TARGET`, `DT_CONTROL`, `DT_PANTILT`). If wall-clock time spikes abnormally — due to window dragging, a breakpoint, or a GC pause — a `MAX_DT = 0.1s` ceiling and `MAX_CATCHUP_STEPS = 10` prevent a **spiral-of-death** (the loop locking up while trying to catch up). This is a standard safeguard in real-time embedded systems, and its presence signals that the simulation was written with real-time system discipline rather than as an ad-hoc demo.

### PID Controller

```
PID_KP = 0.34   PID_KI = 0.015   PID_KD = 0.060
PID_INTEGRAL_LIMIT = 12.0     (anti-windup)
PID_OUTPUT_LIMIT   = 2.0 deg/tick   (fine-mode velocity ceiling)
PID_DERIVATIVE_FILTER_ALPHA = 0.25
```

- **Anti-windup:** the integral term is clamped at `PID_INTEGRAL_LIMIT` so sustained large errors can't drive integral saturation and destabilize the controller.
- **Derivative filtering:** small noise from the target's random-walk motion can get amplified by the D term and show up as jitter in motor output. The derivative term is therefore low-pass filtered (`α = 0.25`).
- **Target lead / prediction:** rather than aiming at the target's instantaneous position, the controller aims at a position projected `LEAD_TIME_SEC = 0.35s` ahead, computed from the target's Kalman-filtered velocity estimate (see below) — a standard technique for reducing phase lag against moving targets.

### Coarse/Fine Mode Switching and Lock-On Logic

The system switches between two control modes:

- **COARSE:** fast approach at a fixed ceiling speed (`COARSE_MAX_SPEED_DEG_PER_TICK = 8.0`) while the angular error is large
- **FINE:** PID takes over for precision approach once the error drops below `TRACK_ANGLE_THRESHOLD_DEG = 2.5°`
- **LOCKED:** lock state is entered once the error drops below `LOCK_ANGLE_THRESHOLD_DEG = 1.2°`

Naively switching modes on a single threshold causes chatter whenever the error oscillates near that threshold, flipping modes every tick. This project solves it with two layered mechanisms:

1. **Hysteresis (Schmitt-trigger logic):** the exit threshold is kept higher than the entry threshold — `COARSE_REENTRY_THRESHOLD_DEG = 4.0°`, `LOCK_EXIT_THRESHOLD_DEG = 2.16°`
2. **Debounce (consecutive-tick confirmation):** hysteresis alone isn't enough, since a continuously moving target can still oscillate around the threshold. A state change (mode or lock) only becomes permanent once the new state holds for `MODE_SWITCH_CONFIRM_TICKS = 6` (~96 ms) / `LOCK_CONFIRM_TICKS = 6` consecutive ticks.

Using both mechanisms together mirrors the approach real radar/tracking systems use to maintain track stability.

### Acceleration Limiting

If the PID/coarse output were applied to the angle directly, velocity could jump instantaneously from one tick to the next — something a real servo motor physically cannot do. To account for this, the rate of change of velocity is separately capped (`MAX_ACCEL_AZ/EL_DEFAULT_DEG_S2 = 150.0°/s²`), configurable independently per axis and adjustable at runtime from the UI. This is a post-processing step applied **after** the coarse/fine velocity computation and **immediately before** integrating it into the angle.

### Target State Estimation — Constant-Velocity Kalman Filter

Each `Target` carries its own 2D Kalman filter (`core/kalman.py`, `CVKalmanFilter2D`) with state `[px, py, vx, vy]` and a constant-velocity process model. The filter is deliberately dependency-free (pure Python, no NumPy) — the 4×4 covariance propagation and 2×2 innovation-covariance inversion are written out explicitly, so the module has zero external requirements.

```
predict:  x_k = F x_{k-1}                  (constant-velocity model)
          P_k = F P_{k-1} F^T + Q
update:   y   = z - H x_k                   (innovation)
          S   = H P_k H^T + R
          K   = P_k H^T S^{-1}
          x_k = x_k + K y
          P_k = (I - K H) P_k
```

`Target.step()` feeds the filter with the ground-truth position each tick as the "measurement," and `predicted_position(lead_time)` extrapolates from the filter's state estimate rather than from raw instantaneous velocity. Process noise (`q_vel`) is scaled per target profile from `TARGET_NOISE_SCALE` / `TARGET_ACCEL_DAMPING`, so an `aggressive` target's filter trusts new velocity information faster (less lag, more responsive to maneuvers) while a `slow` target's filter smooths more aggressively.

This is designed as the estimation layer the system will sit on top of once the planned YOLO-based vision input replaces the simulated ground-truth target position — see [Results](#results) for why that matters.

---

## Multi-Target and Route System

### Multi-Target

- Starts with `INITIAL_TARGET_COUNT = 4`, adjustable at runtime between `MIN_TARGETS = 1` and `MAX_TARGETS = 12`
- Three target profiles (`normal`, `aggressive`, `slow`), each with its own speed envelope and OU-type acceleration noise magnitude — used to simulate distinct target/threat behaviors
- **Automatic target selection:** `nearest` (closest to the platform) or `center` (smallest azimuth) strategy
- **Flip-flop prevention:** to stop auto mode from switching targets every tick when two targets' scores are close, a new candidate must beat the current one by at least a **margin** (`0.6 m` / `4.0°`) and at least `AUTO_SWITCH_COOLDOWN_SEC = 1.2s` must have passed since the last switch. In a synthetic test, this cut the switch count from 300 (every tick, unprotected) to 2 over 300 ticks

### Waypoint / Route System

- Movement along user-defined waypoint sequences (`TRACKING_MODE_ROUTE`)
- Three end-of-route behaviors: `loop` (return to start), `stop` (halt at last point), `pingpong` (reverse direction)
- Route speed adjustable at runtime in the `0.2–6.0 m/s` range

---

## Architecture

The project uses a layered architecture with a strict one-way dependency (UI → Core) that fully separates control logic from the interface. The `core` layer has no dependency on `ui`, which allows the control/simulation logic to be unit-tested independently of the interface.

```
pantilt_tracker/
├── core/
│   ├── kalman.py       # CVKalmanFilter2D — constant-velocity Kalman filter (dependency-free)
│   ├── target.py        # Target physics, target-side state estimation, TargetManager
│   └── ...               # Control loop, PID, route logic, state machine
├── ui/                 # PyQt5 interface layer — see module breakdown below
├── visualization/       # OpenGL-based 3D render pipeline
├── models/              # STL 3D model assets (servo, camera, brackets)
├── config.py            # All system/control parameters (single source of truth)
└── main.py              # Application entry point
```

### `ui/` Module Breakdown

The interface layer is itself split into sub-packages by responsibility; no file carries more than one concern:

```
ui/
├── components/          # Shared/reusable UI components
│
├── control_panel/        # Control panel (left panel)
│   ├── api.py             # The panel's outward-facing interface to core
│   ├── interactions.py    # User interaction logic (button/slider callbacks)
│   ├── panel.py           # The panel widget itself / layout setup
│   └── sections.py        # Sub-sections within the panel (PID, route, target, etc.)
│
├── main_window/           # Main window and application loop
│   ├── handlers.py         # Event/signal handlers
│   ├── layout.py           # Main window layout
│   ├── loop.py             # Wires the multi-rate simulation loop into the UI thread (QTimer)
│   ├── selection.py        # Target/waypoint selection logic
│   └── window.py           # QMainWindow definition
│
└── radar_widget/          # 2D radar/map visualization widget
    ├── coordinates.py      # World <-> screen coordinate transforms
    ├── drawing_grid.py     # Grid and map bounds rendering
    ├── drawing_hud.py      # HUD layer (angle, status, telemetry overlay)
    ├── drawing_route.py    # Waypoint/route rendering
    ├── drawing_targets.py  # Target markers, lock ring, aim-line rendering
    ├── interaction.py      # Mouse click/drag interaction for targets and waypoints
    ├── style.py             # Applies color/style constants to the widget
    └── widget.py             # Root QWidget class, paint-event orchestration
```

Highlights of this breakdown:

- **Drawing, coordinate transforms, and interaction live in separate files inside `radar_widget`** — instead of one "god widget," each concern has its own file, so a change to drawing style (`style.py`) can be made without touching interaction logic (`interaction.py`).
- **In `control_panel`, UI interaction logic (`interactions.py`) is separated from the outward-facing API (`api.py`)** — how the panel talks to `core` is managed from a single point, isolating the blast radius of any interface change on the `core` side to one file.
- **`main_window/loop.py`** is the layer that wires the multi-rate simulation loop into Qt's event loop (`QTimer`, `TICK_MS = 4`) — the bridge between simulation timing and the UI thread is isolated here.
- **`core/kalman.py` is isolated from `core/target.py`** — the filter is a general-purpose, target-agnostic module with no knowledge of `config.py` or target profiles; `target.py` owns the domain-specific tuning (how process noise maps to target type).

---

## Visualization

- **2D Radar (`radar_widget`):** targets, active-target highlighting, lock ring, aim-line, route/waypoint rendering, and a real-time HUD overlay
- **3D Scene (OpenGL):** STL-model-based pan-tilt mechanism (servo brackets, camera assembly), real-time rotation about the pan/tilt pivots, laser/aim-line simulation
- **Telemetry plots:** angular error and velocity, over a `WINDOW = 100`-sample rolling window with a `TRAIL = 50`-sample trace

---

## Results

Two benchmarks were run to validate the target-lead prediction pipeline, comparing naive extrapolation (`position + velocity * LEAD_TIME_SEC`) against the CV Kalman filter's `predicted_position()`, measured as RMSE against each target's actual future position, `LEAD_TIME_SEC = 0.35s` ahead, over 3000 ticks at `DT_TARGET`.

**1. Noiseless ground-truth input** (current state of the simulation — `Target.step()` feeds the filter its own exact position):

| Target profile | Naive RMSE | Kalman RMSE |
|---|---|---|
| normal | 0.175 m | 0.201 m |
| aggressive | 0.445 m | 0.489 m |
| slow | 0.050 m | 0.063 m |

Naive extrapolation is marginally *better* here — expected, since the filter is smoothing a velocity signal that already has zero measurement noise to remove, so the smoothing itself becomes a small lag cost. This confirms the filter isn't buying anything when the input is already ground truth.

**2. Simulated noisy position measurement** (`σ = 0.15 m`, modeling the accuracy of a vision-based detector like the planned YOLO input, where velocity is *not* directly observable and must be estimated from successive noisy position readings):

| Target profile | Finite-difference RMSE | Kalman RMSE | Improvement |
|---|---|---|---|
| normal | 6.465 m | 0.356 m | 94.5% |
| aggressive | 6.443 m | 0.787 m | 87.8% |
| slow | 6.442 m | 0.181 m | 97.2% |

The gap is stark because differentiating two noisy position samples over one tick (`Δt = 1/60s`) amplifies measurement noise by `1/Δt`, turning a 15 cm position error into a multi-m/s velocity error. This is precisely the regime the planned vision-based input will operate in, and it's the reason the Kalman filter is built into the estimation layer now rather than retrofitted later.

---

## Setup

```bash
git clone https://github.com/aligkyrr/multirate-pantilt-tracker.git
cd multirate-pantilt-tracker
pip install -r requirements.txt
python main.py
```

---

## Engineering Scope

The project includes concrete, fully parametrized implementations of:

- Multi-rate real-time system design (independent-Hz sub-loops, fixed-dt integration, spiral-of-death protection)
- Closed-loop PID control design (anti-windup, derivative filtering, target lead/prediction)
- State-machine stability via hysteresis + debounce (chatter prevention)
- Target state estimation via a dependency-free constant-velocity Kalman filter, with per-profile noise tuning
- Actuator simulation modeling physical constraints (velocity/acceleration limits)
- Automatic target selection with flip-flop prevention in a multi-target environment
- Layered, single-responsibility modular software architecture (particularly the sub-packages under `ui/`)
- Real-time 2D/3D data visualization

---

## Planned Improvements

- Real-time target detection via YOLO (vision-based input) — will feed the existing `CVKalmanFilter2D` estimation layer with real, noisy measurements instead of simulated ground truth
- Hardware integration (servo motor driver / Raspberry Pi deployment)
- Network-based remote control interface

---

## Developer

**Ali İhsan GÖKYER**
Electrical-Electronics Engineering Student

---