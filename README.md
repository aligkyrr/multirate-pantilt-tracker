# Multi-Rate Pan-Tilt Target Tracking System

**A real-time, multi-rate control simulation of a two-axis pan-tilt tracking platform.**
PyQt5-based UI, OpenGL 3D visualization, closed-loop PID control, and Kalman-filter-based target state estimation.

---

## Overview

This project is a control and simulation system modeling a two-axis (azimuth / elevation) pan-tilt platform that tracks moving targets in real time. The goal is not a visual demo — it's to build a software-level model that correctly reflects the actual problems real-time tracking systems face: synchronizing subsystems running at different frequencies, closed-loop control stability, chatter-free mode/lock transitions, target state estimation under measurement noise, and target-loss / re-lock behavior.

The system is built on three independent fixed-timestep loops and is fully parameterized through `config.py`; no control or timing constant is hardcoded anywhere in the codebase.

---

## Control System — Technical Details

### Multi-Rate Loop Architecture

| Loop | Frequency | Responsibility |
|---|---|---|
| Target Update | 60 Hz | Target physics (OU-type smooth random walk, boundary bounce) |
| Control Loop (PID) | 120 Hz | Angular error computation, PID output generation |
| Pan-Tilt Update | 60 Hz | Angle/velocity/acceleration integration, actuator physics |

Running the control loop at twice the frequency of the other two loops is a deliberate design decision: sampling the controller more often than actuation and target-physics updates produces a more stable, lower-latency PID output.

Timing is executed with a fixed-`dt` accumulator model (`DT_TARGET`, `DT_CONTROL`, `DT_PANTILT`). If wall-clock time spikes abnormally (window dragging, a breakpoint, a GC pause), a `MAX_DT = 0.1s` ceiling and a `MAX_CATCHUP_STEPS = 10` limit prevent a **spiral-of-death** (the loop locking up while trying to catch up). This is a standard safeguard in real-time embedded systems, and its presence signals that the simulation was written with real-time-system discipline rather than as a casual demo.

### PID Controller

```
PID_KP = 0.34   PID_KI = 0.015   PID_KD = 0.060
PID_INTEGRAL_LIMIT = 12.0     (anti-windup)
PID_OUTPUT_LIMIT   = 2.0 deg/tick   (fine-mode speed ceiling)
PID_DERIVATIVE_FILTER_ALPHA = 0.25
```

- **Anti-windup:** the integral term is clamped at `PID_INTEGRAL_LIMIT`, so sustained large errors can't drive the integral into saturation and destabilize the controller.
- **Derivative filtering:** small noise from the target's random-walk motion can be amplified by the D term and show up as jitter in the motor output. The derivative term is therefore run through a low-pass filter (`α = 0.25`).
- **Target lead/prediction:** the controller doesn't aim at the target's current position — it aims at a position projected `LEAD_TIME_SEC = 0.35s` ahead, using the Kalman-filtered velocity estimate (explained below) — a standard technique for reducing phase lag against moving targets.

### Coarse/Fine Mode Switching and Lock-On Logic

The system switches between two control modes:

- **COARSE:** fast approach at a fixed speed ceiling (`COARSE_MAX_SPEED_DEG_PER_TICK = 8.0`) while angular error is large
- **FINE:** engages once error drops below `TRACK_ANGLE_THRESHOLD_DEG = 2.5°`, for precise PID-driven approach
- **LOCKED:** enters lock state once error drops below `LOCK_ANGLE_THRESHOLD_DEG = 1.2°`

Naively switching modes on a single threshold causes the mode to flip every tick (chatter) whenever the error oscillates near that threshold. This project solves it with a two-layer mechanism:

1. **Hysteresis (Schmitt-trigger logic):** the exit threshold is kept higher than the entry threshold — `COARSE_REENTRY_THRESHOLD_DEG = 4.0°`, `LOCK_EXIT_THRESHOLD_DEG = 2.16°`
2. **Debounce (consecutive-tick confirmation):** hysteresis alone isn't enough, since a continuously moving target can still oscillate around the threshold. A state change (mode or lock) only becomes permanent once the new state persists for `MODE_SWITCH_CONFIRM_TICKS = 6` (~96 ms) / `LOCK_CONFIRM_TICKS = 6` consecutive ticks.

Using both mechanisms together mirrors the approach real radar/tracking systems use to maintain track stability.

### Acceleration Limiting

If the PID/coarse output were applied directly to the angle, velocity could jump instantaneously from one tick to the next — something a real servo motor physically cannot do. To account for this, the rate of change of velocity is separately clamped (`MAX_ACCEL_AZ/EL_DEFAULT_DEG_S2 = 150.0°/s²`), independently configurable per axis and adjustable at runtime from the UI. This is a post-processing step applied **after** the coarse/fine velocity computation and **immediately before** integrating into the angle.

### Hardware Realism Layer

On top of the control logic (the "ideal" angle/velocity/acceleration integration), an independent layer (`core/pantilt_hardware.py`) models the physical/electromechanical constraints and imperfections of a real pan-tilt device. The layer has no dependency on `simulator.py` or `target.py` — a clean, one-directional boundary. It can be toggled at runtime via `HARDWARE_REALISM_ENABLED_DEFAULT`; when disabled, the system behaves identically to the original ideal simulation that existed before this layer was added (backward compatible).

The modeled effects, independently configurable per axis (AZ/EL):

- **Speed envelope:** commanded speeds below `AZ/EL_MIN_SPEED_DEG_S` can't produce motion due to motor stiction (static friction) and the axis stalls; `MAX_SPEED_DEG_S` is the upper speed ceiling.
- **Angle limit:** with `AZ_LIMIT_ENABLED` on, the azimuth axis hard-stops at `AZ_LIMIT_MIN/MAX_DEG = ±185°` to prevent cable wrap (it hits the limit switch and stays there, no bounce). Elevation has no unlimited option to begin with, since it's mechanically bounded.
- **Acceleration limit:** `AZ/EL_MAX_ACCEL_DEG_S2` — since the hardware layer enforces its own acceleration limit, the tracker's own acceleration limiter is disabled while this layer is active (otherwise acceleration would be clamped twice).
- **Communication command-rate limit:** `COMM_MAX_COMMAND_RATE_HZ = 50 Hz` — even though the control loop generates commands at 120 Hz, the device won't accept commands faster than this rate; the commands in between are dropped, and the last accepted command keeps executing. A typical constraint on real serial/CAN/RS485-communicating servo drivers.
- **Velocity ripple:** small random fluctuation produced by the device's own inner controller (a cheap PID or bang-bang driver); modeled with OU-type smooth-random-walk noise (`AZ/EL_VELOCITY_RIPPLE_MAX_DEG_S`).
- **Angular resolution:** encoder/step resolution (`AZ/EL_ANGULAR_RESOLUTION_DEG`) — the position the device reports is rounded to this step (quantization).
- **Position accuracy and repeatability:** accuracy is a fixed per-session calibration bias; repeatability is a random error (backlash, gear play, etc.) re-drawn on every reading.
- **Settling time:** once the commanded velocity drops to zero, the axis is considered "settled" once it enters the `SETTLING_BAND_DEG` band and stays there continuously for `AZ/EL_SETTLING_TIME_SEC`; this affects how cautious the lock-on logic needs to be.

Two distinct notions of position are kept separate: `true_position_deg` is the simulation's internal "true" physical position, while `read_position_deg()` is the position the device reports **externally** (with resolution + accuracy + repeatability error added) — the UI/telemetry/control loop always reads the latter.

**Limit-aware error resolution:** when computing the azimuth error, the naive "shortest path" (`±180°`) logic isn't sufficient — on an axis with a hard angle limit, if the target passes "behind" the pan-tilt (around the 180° point), the shortest path can fall outside the limit, and the axis slams into the stop and gets stuck. Instead, `PanTiltTracker._resolve_az_error()` picks, among the target angle's `±360°` equivalents, the one that stays within the `AZ_LIMIT_ENABLED` bound and is closest to the current position; when needed, this means taking the longer-but-actually-reachable path instead of the shortest one, in the opposite direction.

The parameters can be adjusted at runtime, grouped by category (Azimuth / Elevation / Communication / Settling), through a PyQt5 panel auto-generated from `HARDWARE_MENU_SCHEMA` (`ui/control_panel/hardware_panel.py`, `HardwareProfilePanel`); the panel also includes a telemetry label showing live device status (settled/moving) and the command drop rate.

### Target State Estimation — Constant-Velocity Kalman Filter

Every `Target` carries its own 2D Kalman filter (`core/kalman.py`, `CVKalmanFilter2D`) — state vector `[px, py, vx, vy]` with a constant-velocity process model. The filter is deliberately dependency-free (pure Python, no NumPy) — the 4×4 covariance propagation and the 2×2 innovation-covariance inverse are written out explicitly by hand, so the module has zero external dependencies.

```
predict:  x_k = F x_{k-1}                  (constant-velocity model)
          P_k = F P_{k-1} F^T + Q
update:   y   = z - H x_k                   (innovation)
          S   = H P_k H^T + R
          K   = P_k H^T S^{-1}
          x_k = x_k + K y
          P_k = (I - K H) P_k
```

`Target.step()` feeds the filter with the ground-truth position as a "measurement" on every tick, and `predicted_position(lead_time)` extrapolates from the filter's state estimate rather than the raw instantaneous velocity. Process noise (`q_vel`) is scaled from `TARGET_NOISE_SCALE` / `TARGET_ACCEL_DAMPING` according to the target profile; so an `aggressive` target's filter trusts new velocity information faster (less lag, more responsive to maneuvers), while a `slow` target's filter smooths more aggressively.

This is designed as the estimation layer the system will sit on once the planned YOLO-based image input replaces the simulated ground-truth target position — see the [Results](#results) section for why this matters.

---

## Multi-Target and Route System

### Multi-Target

- Starts with `INITIAL_TARGET_COUNT = 4`, adjustable at runtime between `MIN_TARGETS = 1` and `MAX_TARGETS = 12`
- Three target profiles (`normal`, `aggressive`, `slow`), each with its own speed envelope and OU-type acceleration noise magnitude — used to simulate different target/threat behaviors
- **Automatic target selection:** `nearest` (closest to the platform) or `center` (smallest azimuth) strategy
- **Flip-flop prevention:** to stop auto mode from switching targets every tick when two targets' scores are close, a new candidate must beat the current active target by at least a **margin** (`0.6 m` / `4.0°`), and at least `AUTO_SWITCH_COOLDOWN_SEC = 1.2s` must have passed since the last switch. In a synthetic test, this reduced the switch count over 300 ticks from 300 (every tick, unprotected) to 2

### Waypoint / Route System

- Movement along user-defined waypoint sequences (`TRACKING_MODE_ROUTE`)
- Three behaviors at route end: `loop` (return to start), `stop` (halt at the last point), `pingpong` (reverse direction)
- Route speed adjustable at runtime in the `0.2–6.0 m/s` range

---

## Architecture

The project uses a layered architecture that fully separates control logic from the UI (a one-directional UI → Core dependency). The `core` layer has zero dependency on `ui`, which allows control/simulation logic to be unit-tested independently of the UI.

```
pantilt_tracker/
├── core/
│   ├── kalman.py         # CVKalmanFilter2D — constant-velocity Kalman filter (dependency-free)
│   ├── target.py          # Target physics, target-side state estimation, TargetManager
│   ├── pantilt_hardware.py # Hardware realism layer (speed/accel/angle limits, ripple, resolution, accuracy/repeatability, settling) — depends on config, independent of simulator.py/target.py
│   └── ...               # Control loop, PID, route logic, state machine
├── ui/                  # PyQt5 UI layer — see the module breakdown below
├── visualization/        # OpenGL-based 3D render pipeline
├── models/               # STL 3D model assets (servos, camera, brackets)
├── config.py             # All system/control parameters (single source of truth)
└── main.py               # Application entry point
```

### `ui/` Module Breakdown

The UI layer is split into sub-packages by responsibility; no file carries more than one responsibility:

```
ui/
├── components/          # Shared/reusable UI components
│
├── control_panel/        # Control panel (left panel)
│   ├── api.py             # The panel's outward-facing interface to core
│   ├── interactions.py    # User interaction logic (button/slider callbacks)
│   ├── panel.py           # The panel widget itself / layout setup
│   ├── sections.py        # Sub-sections within the panel (PID, route, target, etc.)
│   └── hardware_panel.py  # Hardware realism parameters panel (auto-generated from HARDWARE_MENU_SCHEMA)
│
├── main_window/           # Main window and application loop
│   ├── handlers.py         # Event/signal handlers
│   ├── layout.py           # Main window layout
│   ├── loop.py             # Binds the multi-rate simulation loop to the UI thread (QTimer)
│   ├── selection.py        # Target/waypoint selection logic
│   └── window.py           # QMainWindow definition
│
└── radar_widget/          # 2D radar/map visualization widget
    ├── coordinates.py      # World <-> screen coordinate transforms
    ├── drawing_grid.py     # Grid and map-boundary drawing
    ├── drawing_hud.py      # HUD layer (angle, status, telemetry overlay)
    ├── drawing_route.py    # Waypoint/route drawing
    ├── drawing_targets.py  # Target markers, lock ring, aim-line drawing
    ├── interaction.py      # Mouse click/drag interaction for targets and waypoints
    ├── style.py             # Applies color/style constants to the widget
    └── widget.py             # Root QWidget class, paint-event orchestration
```

Highlights of this breakdown:

- **Drawing, coordinate transforms, and interaction each live in separate files within `radar_widget`** — instead of a single "god widget," every responsibility has its own file, so a drawing-style change (`style.py`) can be made without touching interaction logic (`interaction.py`).
- **Within `control_panel`, UI interaction logic (`interactions.py`) is separated from the outward-facing API (`api.py`)** — how the panel talks to `core` is managed from a single place, so the blast radius of any interface change on the `core` side is isolated to one file.
- **`main_window/loop.py`** is the layer that binds the multi-rate simulation loop to Qt's event loop (`QTimer`, `TICK_MS = 4`) — the bridge between simulation timing and the UI thread is isolated here.
- **`core/kalman.py` is decoupled from `core/target.py`** — the filter is a general-purpose module with no knowledge of the target and has no awareness of `config.py` or target profiles; `target.py` owns the domain-specific calibration (how process noise maps to target type) itself.

---

## Visualization

- **2D Radar (`radar_widget`):** targets, active-target highlighting, lock ring, aim line, route/waypoint drawing, and a real-time HUD overlay
- **3D Scene (OpenGL):** STL-model-based pan-tilt mechanism (servo brackets, camera mount), real-time rotation around pan/tilt pivots, laser/aim-line simulation
- **Telemetry graphs:** angular error and velocity, over a `WINDOW = 100`-sample rolling window with a `TRAIL = 50`-sample trail

---

## Results

Two benchmarks were run to validate the target-lead-prediction pipeline: naive extrapolation (`position + velocity * LEAD_TIME_SEC`) was compared against the CV Kalman filter's `predicted_position()` output, measured as RMSE against each target's true position `LEAD_TIME_SEC = 0.35s` later, over 3000 ticks at the `DT_TARGET` step.

**1. Noise-free ground-truth input** (the simulation's current state — `Target.step()` feeds the filter its own perfect position):

| Target profile | Naive RMSE | Kalman RMSE |
|---|---|---|
| normal | 0.175 m | 0.201 m |
| aggressive | 0.445 m | 0.489 m |
| slow | 0.050 m | 0.063 m |

Naive extrapolation is slightly *better* here — an expected result, since the filter is smoothing a velocity signal whose measurement noise is already zero; that smoothing itself costs a small amount of lag. This confirms that the filter provides no gain when the input is already ground truth.

**2. Simulated noisy position measurement** (`σ = 0.15 m`, modeling the accuracy of a planned YOLO-like image-based detector — in this scenario velocity isn't directly observable and must be estimated from consecutive noisy position readings):

| Target profile | Finite-difference RMSE | Kalman RMSE | Improvement |
|---|---|---|---|
| normal | 6.465 m | 0.356 m | 94.5% |
| aggressive | 6.443 m | 0.787 m | 87.8% |
| slow | 6.442 m | 0.181 m | 97.2% |

The gap is this large because differentiating two noisy position samples over a single tick (`Δt = 1/60s`) amplifies measurement noise by a factor of `1/Δt` — a 15 cm position error turns into a very large velocity error. This is exactly the regime the planned image-based input will operate in, and it's why the Kalman filter was built into the estimation layer from the start rather than bolted on later.

**3. Limit-aware path resolution** — with the hard-angle-limited (`AZ_LIMIT_ENABLED=True`, `±185°`) hardware realism layer enabled, a scenario where the target passes directly behind the platform (around 180°) was simulated end-to-end using `core/pantilt_hardware.PanTiltDeviceSimulator`: the pan-tilt is at `+170°` and the target suddenly jumps to `-170°` (since `des_az` is always computed via `atan2` within `±180°`, this is a genuine situation that arises any time the target actually passes behind the platform). Keeping the coarse-mode velocity computation and the real `PanTiltDeviceSimulator` physics (acceleration, speed ceiling, hard stop) exactly as they are, the naive `±180°`-wrap error computation was compared against the fixed `_resolve_az_error()` (which picks the nearest reachable candidate, within the limit, among the target's `±360°` equivalents), over up to 2000 ticks at the `DT_PANTILT` step:

| | Before fix (naive ±180° wrap) | After fix (`_resolve_az_error`) |
|---|---|---|
| Locks onto target | ❌ never (over 2000 ticks) | ✅ 351 ticks (~5.85 s) |
| Final azimuth | 184.97° (stuck at the hard limit) | -167.41° |
| Final true angular error | 5.03° (permanent, fixed) | 2.59° (within the lock-threshold band, oscillating) |
| Smallest error observed | 5.01° (never escapes the limit) | 0.20° |

Before the fix, the axis slams into the limit switch and gets stuck there because the shortest (~20°) path to the target falls outside the `185°` bound — and it **never reaches the target again**; the error stays permanently fixed around ~5°. After the fix, the system picks the target among its `±360°` equivalents that stays within the limit (`-170°`, i.e. the "long but reachable" `-340°` path from the current position), and effectively achieves full lock onto the target.

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

The project contains concrete, fully parameterized implementations of:

- Multi-rate real-time system design (independent-Hz sub-loops, fixed-dt integration, spiral-of-death protection)
- Closed-loop PID control design (anti-windup, derivative filtering, target lead prediction)
- State-machine stability via hysteresis + debounce (chatter prevention)
- Dependency-free constant-velocity Kalman filter for target state estimation, with per-profile noise calibration
- Actuator simulation modeling physical constraints (speed/acceleration limits)
- An independent hardware realism layer: speed envelope, hard angle limit, communication command dropping, velocity ripple, encoder resolution, accuracy/repeatability, settling time — paired with a limit-aware angular path resolution that prevents getting stuck at the limit on a hard-angle-limited axis when the target passes behind the platform
- Flip-flop-resistant automatic target selection in a multi-target environment
- Layered, single-responsibility modular software architecture (especially the sub-packages under `ui/`)
- Real-time 2D/3D data visualization

---

## Planned Improvements

- Real-time target detection with YOLO (image-based input) — will feed the existing `CVKalmanFilter2D` estimation layer with real, noisy measurements instead of the simulated ground truth
- Hardware integration (servo motor driver / Raspberry Pi deployment)
- Network-based remote control interface

---

## Developer

**Ali İhsan GÖKYER**
Electrical-Electronics Engineering Student

---