# Software, Navigation and Control Subsystem — PDR Content

**NIOT Student Autonomous Underwater Vehicle Competition (SAVe)**
Sections 8, 9 and 10 of the Preliminary Design Report

> Editing notes appear in blockquotes and should be removed before
> submission. Figure placeholders are marked **[FIGURE n]** with a
> description of what to draw and a suggested caption.

---

## 8. Software & Simulation Architecture

### 8.1 Overview

The autonomy stack runs on ROS 2 Humble and comprises approximately
4,100 lines across nine modules. It has been developed against a full
simulation of the competition arena in Gazebo Sim 8, where **four of the
six mission tasks currently execute end to end in a single autonomous
run.**

The stack is organised so that perception, estimation, control and
mission logic are separable — each can be tested in isolation before
integration, and each has been.

---

### 8.2 Architectural Details

#### The governing constraint

The vehicle has no absolute horizontal position reference. There is no
DVL, no acoustic positioning, and a magnetometer is unreliable indoors
near steel pool structure and the vehicle's own thruster magnets.

**Every horizontal decision must therefore be made from what the cameras
see at that instant.**

This produces the architecture's defining property: *the vehicle
inspects rather than remembers.* It does not plan in world coordinates.
It centres over whatever target is beneath it, reads it, and acts —
because "return to where the X bin was" is not an instruction a vehicle
without position can execute.

This is a deliberate design choice, not a limitation accepted
reluctantly. A stack built around a position estimate would degrade
silently as that estimate drifted; a stack built around present
observation either sees its target or reports that it does not.

#### Module structure

| Module | Lines | Responsibility |
|---|---:|---|
| `line_buoy.py` | 1581 | Mission sequencer — 17-state finite state machine |
| `buoy.py` | 573 | Multi-colour buoy detection, ranking, visual servoing |
| `bluerov2_native_bridge.py` | 550 | Simulator ↔ ROS 2 bridge and sensor models |
| `line_follow.py` | 448 | Path detection, cross-track control, fork selection |
| `marker.py` | 396 | Bin detection, symbol classification, station keeping |
| `thruster_mixer.py` | 197 | Body-frame commands → 8 thruster demands |
| `octagon.py` | 196 | Surfacing-ring detection and containment test |
| `depth_filter.py` | 145 | Pressure + inertial fusion |
| `depth_control.py` | 45 | Depth PID |
| **Total** | **4131** | |

#### Explaining the module structure

The line counts reveal the architecture. **`line_buoy.py` at 1581 lines
is the largest module by a wide margin** — it is the mission sequencer,
and it contains no perception or control mathematics at all. It
arbitrates: deciding which behaviour owns the vehicle at any instant and
when to hand over.

The three perception modules (`buoy`, `line_follow`, `marker`) are
comparable in size and structurally similar — each detects its targets,
computes pixel errors, and returns body-frame commands. None of them
knows the mission exists.

The two control modules are strikingly small (`depth_control.py` is 45
lines) because the control problem is genuinely simple once the
estimation problem is solved. `depth_filter.py` is three times the size
of the controller it feeds, which reflects where the difficulty actually
lies.

#### Layered architecture

> **[FIGURE 8.1]** Redraw as a four-layer block diagram with arrows
> between layers labelled as shown.
> *Caption: "Four-layer software architecture. Only one behaviour
> commands the horizontal axes at any instant; the depth loop runs
> beneath all of them."*

```
┌──────────────────────────────────────────────────────┐
│  MISSION SEQUENCER          line_buoy.py             │
│  17-state FSM; arbitrates which behaviour is active  │
└───────┬──────────────────────────────────▲───────────┘
        │ activate behaviour               │ target acquired /
        │                                  │ task complete / lost
┌───────▼──────────────────────────────────┴───────────┐
│  BEHAVIOUR LAYER                                     │
│  line_follow.py │ buoy.py │ marker.py │ octagon.py   │
│  perception + control law, one per mission task      │
└───────┬──────────────────────────────────▲───────────┘
        │ surge, sway, yaw                 │ camera frames
        │ depth setpoint                   │
┌───────▼──────────────────────────────────┴───────────┐
│  ESTIMATION & CONTROL                                │
│  depth_filter.py  (z, ż)   depth_control.py  (PID)   │
└───────┬──────────────────────────────────▲───────────┘
        │ heave command                    │ pressure, IMU
┌───────▼──────────────────────────────────┴───────────┐
│  ACTUATION       thruster_mixer.py                   │
│  body-frame wrench → 8 individual thrust demands     │
└──────────────────────────────────────────────────────┘
```

#### Separation of axes — the structural decision

One decision underpins the whole design:

> **Exactly one behaviour commands the horizontal axes at any instant,
> but the depth loop runs continuously beneath every state.**

The reason is physical. The vehicle is positively buoyant by
approximately 1.4 N — it floats to the surface the moment nothing holds
it down. Depth can therefore never be handed between behaviours the way
steering can; there is no safe moment when nobody owns it.

Behaviours influence depth by *moving the setpoint*, never by commanding
thrust directly:

```
pixel error → commanded rate (m/s) → depth setpoint → PID → thrust
  outer loop        integrator        inner loop
```

Keeping the buoyancy-trim integrator in exactly one place prevents two
controllers accumulating against each other on the same axis. Had each
behaviour commanded heave directly, every handover would produce a
transient as one integrator's accumulated trim was discarded.

---

### 8.3 Perception Layer

The stack uses classical computer vision — colour segmentation and
contour analysis — rather than a learned detector. The targets are of
known size, shape and colour; a classical pipeline is deterministic,
requires no training data, runs comfortably on embedded hardware, and
can be justified analytically. Each threshold below is derived from
measurement rather than tuned by trial.

#### Colour thresholds

| Target | HSV lower | HSV upper |
|---|---|---|
| Orange path | (2, 80, 50) | (22, 255, 255) |
| Bin (navy) | (100, 120, 60) | (125, 255, 190) |
| Bin symbol (white) | (0, 0, 88) | (180, 60, 255) |
| Octagon ring (blue) | (104, 100, 55) | (128, 255, 175) |

#### Explaining the thresholds — measured, not modelled

These bounds come from captured frames, **not** from the materials'
declared colours. The distinction is not academic; it was learned by
failure.

| Surface | Predicted from material | Measured in frame |
|---|---|---|
| White bin symbol | (106, 47, 226) | **(0, 0, 97)** |
| Navy bin | (111, 188, 137) | (112, 131, 68) |
| Grey floor | (108, 70, 168) | (120, 6, 80) |

Rendered scenes are 2–2.5× darker than an alpha blend of the material
colour predicts, because the scene is lit by a single source through a
metre of translucent water. A threshold set from the material model
required V ≥ 165 for white; real white paint renders at V = 97. The
white mask contained *zero pixels*, the bin symbol was invisible to the
classifier, and the vehicle hovered over the target indefinitely without
ever committing.

The correction — measuring a real frame before setting the threshold —
is the methodology now applied to every colour band in the stack.

#### Shape and topology gates

Colour alone is insufficient because several arena objects share
materials. Two geometric cues resolve them.

**Circularity** — the ratio of contour area to the area of its minimum
enclosing circle. This is scale-free, so it works at any range:

| Object | Circularity |
|---|---|
| Buoy (sphere) | 0.97 |
| Bin symbol "O" (ring) | 0.97–0.99 |
| Torpedo target (flat square) | 0.38 |
| Bin symbol "X" (cross) | 0.32–0.33 |

A single threshold at 0.72 separates spheres from the square torpedo
plate, which shares the buoys' exact red material. A second threshold at
0.70 separates the ring from the cross.

**Contour topology** — a ring encloses an inner contour; a cross does
not. This is an independent confirmation of the O/X classification, so a
single ambiguous frame cannot flip the decision. It is also the cue the
heart cut-out will use, since a hole in a plate is topologically the
same problem.

**Elevation gating** — the octagon floats sit at the water surface,
above the vehicle, and therefore always project into the upper region of
the frame. Rejecting detections above a fraction of frame height removes
all eight floats without reference to colour or range.

> The elevation threshold was initially set assuming the vehicle sits at
> its nominal cruise depth. In practice the dive overshoots, placing the
> vehicle below nominal and the target *above* the optical axis — where
> the gate rejected it. The threshold now accommodates the full range of
> depths the vehicle actually visits, which is a more defensible basis
> than a single nominal geometry.

#### Detection gates summary

| Parameter | Buoy | Bin |
|---|---|---|
| Minimum circularity | 0.72 | — |
| Maximum aspect ratio | 1.35 | 1.45 |
| Minimum extent | — | 0.55 |
| Elevation gate | 0.22 of frame height | — |
| Classification split | — | 0.70 circularity |

---

### 8.4 Mission Sequencer

A 17-state finite state machine driven by a fixed **20 Hz** timer,
operating on the most recent frame from each camera rather than inside
image callbacks. This decouples the control period from camera jitter
and gives every derivative and integrator a constant timestep.

> **[FIGURE 8.2]** Redraw as a proper state diagram. Solid arrows for
> nominal transitions, dashed for recovery paths.
> *Caption: "Mission finite state machine. Every failure path is
> bounded: lost targets trigger search states, searches time out, and
> attempt counters abandon a task rather than trapping the run."*

```
        INIT
          │ sensors ready
        DIVE
          │ depth held
    ┌─►LINE_FOLLOW◄──────────────────────────┐
    │     │                                  │
    │     ├─ buoy in range ─► BUOY_APPROACH  │
    │     │                      │    ▲      │
    │     │                      │    └─ BUOY_SEARCH
    │     │                      ▼           │
    │     │                  BUOY_TOUCH      │
    │     │                      ▼           │
    │     │                 BUOY_BACKOFF ────┤
    │     │                                  │
    │     ├─ fork detected ─► route selected │
    │     │                                  │
    │     ├─ bin in view ─► BIN_APPROACH     │
    │     │                   │      │       │
    │     │          BIN_REJECT      ▼       │
    │     │              │      BIN_DESCEND  │
    │     │              │           ▼       │
    │     │              │       BIN_HOLD◄─┐ │
    │     │              │           ▼     │ │
    │     │              │        BIN_DROP─┘ │
    │     │              │           ▼       │
    │     │              └────►  BIN_DONE ───┤
    │     │                                  │
    │     └─ path ends ─► OCTAGON_ARRIVE     │
    │                          ▼             │
    │                    OCTAGON_HOLD        │
    │                          ▼             │
    │                   OCTAGON_ASCEND       │
    │                          ▼             │
    └───────────────────►  SURFACED
```

#### Explaining the state machine

**Every mission task follows the same four-phase shape:** approach the
target under visual servoing, verify it is the right target, execute the
action, then withdraw and rejoin the path. Recognising this pattern is
what allowed the buoy, bin and octagon behaviours to share the same
control machinery.

**Recovery states are first-class, not exception handling.**
`BUOY_SEARCH` and `BIN_REJECT` are ordinary states with their own
control laws and timeouts. A vehicle that loses its target does not
fail — it enters a bounded search, and if that search times out, an
attempt counter abandons the task rather than retrying forever.

**Every path has a bound.** This is deliberate: an autonomous vehicle
with an unbounded retry loop will spend the entire mission window on one
task. Attempt counters mean a task that cannot be completed is
abandoned in favour of the ones that can.

#### Route selection at the fork

The path splits into two symmetric branches. Naive centroid-based line
following averages the two legs and steers into the gap between them,
then follows whichever branch happens to remain in frame — an arbitrary
choice.

The path is instead sampled as **separate contiguous clusters** in a
horizontal band. Two or more clusters means a fork, at which point the
preferred side is selected deliberately.

> One subtlety: at the split itself the two legs still touch, merging
> into a single wide cluster. Fork detection therefore uses a longer
> lookahead than steering does — the legs only resolve as distinct some
> distance up the image.

---

### 8.5 Simulation Environment

| Item | Selection |
|---|---|
| Simulator | Gazebo Sim 8 (Harmonic) |
| Physics | `dartsim`, 1 ms fixed step |
| Middleware | ROS 2 Humble |
| Arena | 25 × 20 m tank, 2.5 m water column, full course |
| Vision | OpenCV 4 |

#### Sensor realism — a deliberate choice

The simulator provides perfect, instantaneous ground truth. **Tuning a
controller against that produces gains that cannot transfer to
hardware.** The sensor chain is therefore modelled explicitly:

| Effect | Value | Applied |
|---|---|---|
| Transport latency | 30 ms | First |
| Sensor noise | 0.3 mbar RMS | Second |
| Quantisation | 0.2 mbar | Third |

The ordering is physical, not arbitrary: an ADC digitises an
already-noisy analogue signal, so noise precedes quantisation. Reversing
them would eliminate the dithering that real noise provides for free.

IMU noise and bias are modelled at MTi-630R grade.

#### Restricted ground truth access

The entire stack consumes exactly **two scalars** of ground truth —
depth and depth rate — and both are replaced by the modelled sensor
chain before reaching any controller. No module uses ground-truth
horizontal position or heading.

This is what keeps the simulation-to-hardware boundary narrow and
testable: only one measurement chain must be replicated on the real
vehicle, and the Bar30 is precisely the sensor that provides it.

---

### 8.6 Digital Twin Vehicle Profile

The simulated vehicle mirrors the physical one.

**Eight-thruster actuation.** Four horizontal, vectored at 45° about the
centre of gravity, providing coupled surge, sway and yaw. Four vertical
at the frame corners, driven identically for heave.

Corner placement of the vertical cluster makes the zero-pitch property
*structural* rather than tuned: with four units symmetric fore and aft
about the centre of gravity, equal drive produces exactly zero pitch
moment regardless of magnitude.

**Virtual sensors** — camera pair matching the real optics, IMU with
realistic noise and bias, and a Bar30 model with the latency,
quantisation and noise described above.

---

## 9. Navigation Stack

> **This section describes what the stack implements.** An earlier
> revision specified a 12-state EKF with waypoint following; that
> architecture assumes an absolute horizontal position estimate which
> the sensor suite cannot provide.

### 9.1 Sensor Fusion and Data Interfaces

| Topic | QoS | Rationale |
|---|---|---|
| Camera streams | Best effort, depth 1 | Latency matters more than delivery; a stale frame is worse than a dropped one |
| `/bluerov2/depth` | Best effort, depth 1 | High rate; estimator tolerates occasional loss |
| `/bluerov2/imu/data` | Best effort, depth 1 | As above |
| `/cmd_surge`, `/cmd_sway` | Reliable, depth 1 | Every command must reach the mixer |
| `/cmd_yaw`, `/cmd_heave` | Reliable, depth 1 | As above |
| `/mission/state` | Reliable, depth 1 | Monitoring must not miss transitions |

#### Explaining the QoS choices

The split is between **data that becomes worthless when stale** and
**data that must not be lost**.

Camera frames and sensor readings are the first kind. A frame that
arrives 200 ms late is not merely less useful — it is actively harmful,
because the control loop would act on a scene that no longer exists.
Best-effort with a queue depth of 1 means the newest frame overwrites
any predecessor, so the loop always operates on the freshest data.

Actuation commands are the second kind. A dropped thrust command means
the mixer holds its previous value, and the vehicle continues on a stale
demand. Reliable delivery with depth 1 gives both guarantees: the
command arrives, and it is the most recent one.

---

### 9.2 Vertical State Estimation

This is the most substantive analytical result in the stack.

#### The problem

Depth control requires vertical *velocity* for its damping term, but the
pressure sensor measures only position. Differencing consecutive samples
amplifies noise severely. For 2 mm resolution at 20 Hz:

```
σ_v = σ_z · √2 / Δt = 0.002 × 1.414 / 0.05 = 0.057 m/s
```

Passed through the derivative gain, this produces thrust chatter
**larger than the 1.4 N of buoyancy the loop is trimming.** The scaling
is perverse: sampling *faster* makes the naive derivative worse, because
real motion between samples shrinks while noise stays constant.

#### Why filtering alone fails

Low-pass filtering the derivative trades noise against phase lag. No
setting is simultaneously quiet and responsive — the filter that removes
the chatter also removes the loop's ability to react.

#### The solution

Fuse pressure with inertial acceleration in a two-state Kalman filter
over `[z, ż]`. Each sensor covers exactly what the other cannot:

| | Pressure (Bar30) | IMU accelerometer |
|---|---|---|
| Update rate | 20 Hz | 1 kHz |
| Noise | jittery | smooth |
| Long-term | never drifts | drifts (bias integrates) |

The prediction step carries the fast dynamics from the accelerometer;
the slow, absolute, drift-free pressure measurement corrects the
accelerometer's bias.

The measurement update also corrects **velocity**, not just position,
through the position–velocity correlation term in the covariance. This
is how a clean rate estimate is extracted from a position-only sensor.

#### Measured results

| Configuration | Thrust chatter | Estimator error |
|---|---:|---:|
| Raw backward difference | **3.67 N** | 1.1 cm |
| Constant-velocity filter | 0.56 N | 1.0 cm |
| **Pressure + IMU fused** | **0.08 N** | **0.4 cm** |

#### Explaining the results table

Read the middle column. **3.67 N of chatter against 1.4 N of buoyancy
trim** means the derivative term was fighting harder than the force it
was meant to correct — the vertical thrusters would buzz continuously,
wasting energy and adding acoustic noise.

The middle row shows that a constant-velocity filter alone recovers most
of the benefit (0.56 N), which is worth noting: it is a much simpler
implementation. But it costs phase lag, and the third row shows fusion
achieves a further **7× improvement** on top of that.

The right column shows estimator error improving by more than half. The
two improvements are related but distinct: less chatter means less
wasted actuation, while lower error means the vehicle holds the depth it
was asked to.

**A factor of 46 reduction in chatter, with no change to controller
gains.** This is the strongest argument for the fusion approach — it
required no retuning of the controller it feeds.

> **[FIGURE 9.1]** Plot commanded heave against time for the three
> configurations. The naive trace should visibly oscillate; the fused
> trace should be nearly flat.
> *Caption: "Vertical state estimation. Naive differencing produces
> thrust chatter exceeding the buoyancy trim; inertial fusion reduces it
> by a factor of 46."*

---

### 9.3 Horizontal Velocity — Visual Odometry

With no DVL, horizontal velocity is measured from how fast a tracked
feature slides across the downward camera, scaled by altitude:

```
metres_per_pixel = altitude / focal_length
```

Altitude derives from the pressure sensor and known target height.

**Why this matters:** a released marker inherits the vehicle's
horizontal velocity. Commanding zero velocity is not the same as being
stopped, because the hull coasts. Without a direct measurement, the
vehicle cannot know whether it has actually come to rest.

> **Implementation note.** The estimator publishes no value until it has
> accumulated sufficient motion history. A filter initialised at zero
> reports "stopped" on its first sample regardless of true speed — a
> fault observed during development, where markers were released at a
> reported 0 cm/s while ground truth showed 42–57 cm/s. The markers
> still landed in the bin, but by margin rather than by the gate
> working. The corrected estimator now withholds output until converged.

---

### 9.4 Range Estimation

Range to a target of known dimension follows the pinhole model:

```
Z = R_real · f / R_pixels
```

For the L-bar, range is better obtained from the bar's angular
*depression* below the optical axis than from its apparent thickness,
because the bar's height is known and its diameter is small:

| Range | Error per pixel, via thickness | via depression |
|---|---|---|
| 1.5 m | ±0.07 m | ±0.01 m |
| 3.0 m | **±0.30 m** | **±0.03 m** |

#### Explaining the table

Both methods use the same camera and the same pixel measurement
precision. The difference is in what is being measured.

The bar is 0.06 m in diameter — at 3 m it spans only about 11 pixels, so
a single pixel of error is nearly 10 % of the measurement. The
depression angle, by contrast, is measured against the bar's known
height above the floor, which produces a much larger pixel displacement
and correspondingly better conditioning.

**An order of magnitude better, from the same sensor.** The lesson
generalises: when several geometric measurements are available, prefer
the one with the largest pixel signal.

#### Camera calibration

**Intrinsics are read from a `CameraInfo` topic rather than hard-coded.**
This is necessary because a flat viewport scales effective focal length
by approximately 1.33; using a dry calibration underwater makes every
range read about 25 % short:

| True range | Reported with dry calibration |
|---|---|
| 0.50 m | 0.38 m |
| 1.00 m | 0.75 m |
| 2.00 m | 1.50 m |

A vehicle told to stop at 0.35 m would halt at 0.47 m and never reach
its target. Because intrinsics arrive over a topic, an underwater
calibration substitutes without any code change.

---

### 9.5 Task Sequencing in Place of Waypoints

The stack carries no waypoint list and no world-frame trajectory
generator, because it has no absolute horizontal position to plan
against. Sequencing is by visual recognition:

| Transition | Trigger |
|---|---|
| Dive complete | Depth held within tolerance for a settle period |
| Buoy engagement | Apparent radius exceeds threshold for N consecutive frames |
| Route selection | Two path branches resolved as separate clusters |
| Bin engagement | Navy region above area threshold, passing squareness gate |
| Symbol classified | Circularity and topology agree |
| Marker release | Centred within tolerance **and** visual ground speed below threshold |
| Course end | Path lost persistently after the marker task |
| Surfaced | Pressure-derived depth below threshold |

#### Explaining the sequencing approach

Each trigger is a **present observation**, not a position. This makes
the sequencing robust to accumulated drift, which is the failure mode a
waypoint-based approach would suffer without absolute positioning.

Two triggers deserve comment. **Marker release requires two independent
conditions** — centred *and* stopped — because either alone is
insufficient: a centred but moving vehicle releases a marker that drifts
out of the bin. **Course end uses persistent path loss** rather than a
coordinate, because both branches of the course terminate at an octagon
centre by construction; running out of path *is* arrival.

---

## 10. Control System Design & Tuning

### 10.1 Cascaded Control Architecture

The vehicle regulates four degrees of freedom: **surge, sway, heave and
yaw.** Roll and pitch are not actively controlled and do not need to be:

- **Roll** is passively stable because the centre of buoyancy sits above
  the centre of gravity, producing a righting moment.
- **Pitch** is passively stable for the same reason, and additionally
  the four vertical thrusters are symmetric fore and aft about the
  centre of gravity, so heave produces zero pitch moment by
  construction.

Every loop implements the standard discrete PID form:

```
e_k         = setpoint_k − measured_k
integral_k  = integral_{k−1} + e_k · Δt
derivative_k = −(measured_k − measured_{k−1}) / Δt
u_k         = Kp·e_k + Ki·integral_k − Kd·derivative_k
```

#### Three implementation properties

**Anti-windup.** Integral accumulation is clamped. Without this, a
saturated actuator allows the integral to grow without bound, producing
severe overshoot when the constraint releases.

**Derivative on measurement.** The derivative term acts on the measured
value, not the tracking error. This avoids "derivative kick" — a step
change in setpoint would otherwise produce an impulse in the derivative
term and a spike in actuator command.

**Rate damping from a measured rate.** The vehicle's drag is purely
quadratic with no linear damping term, so the derivative term must use
an actual measured rate rather than a differentiated setpoint error.
This is why the vertical estimator of Section 9.2 exists.

---

### 10.2 Eight-Thruster Allocation Matrix

The control loops output a desired body-frame wrench:

```
τ = [Fx, Fy, Fz, Mz]ᵀ
```

where Fx is surge, Fy is sway, Fz is heave, and Mz is yaw moment. The
vehicle has 4 controllable degrees of freedom and 8 thrusters, so the
wrench is distributed by:

```
T = B⁺ · τ           T = [T1, T2, T3, T4, T5, T6, T7, T8]ᵀ
```

where **B is the 4×8 configuration matrix** derived from each thruster's
mounting position and thrust unit vector, and B⁺ its Moore–Penrose
pseudo-inverse.

The horizontal contributions, derived from mounting geometry:

| Thruster | Surge | Sway | Yaw |
|---|---:|---:|---:|
| T1 front-right | −0.707 | −0.707 | −0.164 |
| T2 front-left | −0.707 | +0.707 | +0.164 |
| T3 rear-right | +0.707 | −0.707 | +0.164 |
| T4 rear-left | +0.707 | +0.707 | −0.164 |

T5–T8 are vertical, contributing to heave only, and driven identically.

#### Explaining the allocation table

The **±0.707 entries are cos 45°** — the horizontal thrusters are
vectored at 45°, so each contributes equally to surge and sway. This is
what makes the vehicle holonomic in the horizontal plane: it can
translate sideways without turning, which the buoy and bin behaviours
both depend on.

The **±0.164 yaw entries are much smaller** because yaw authority comes
from the moment arm between the thruster and the centre of gravity,
which is short relative to the thrust magnitude. Yaw is therefore the
weakest axis — a consideration when tuning, since the yaw loop saturates
before surge or sway.

**Note the sign pattern.** Driving all four horizontal thrusters with
the same sign produces zero net force: the front pair opposes the rear
pair. Motion comes from the *differences* between them. This is a useful
sanity check during bring-up — a uniform command should leave the
vehicle stationary.

#### Saturation handling

**When any thruster would saturate, the entire command vector is scaled
down rather than clipping the individual channel.**

This matters more than it appears. Clipping one thruster changes the
*direction* of the resultant force, silently converting a commanded
turn into a turn plus unintended drift. Uniform scaling preserves the
direction and reduces only the magnitude — the vehicle does what was
asked, just less forcefully.

---

### 10.3 Controller Loop Parameters

| Loop | Kp | Ki | Kd | Purpose |
|---|---:|---:|---:|---|
| Depth (heave) | 10.0 | 2.0 | 8.0 | Depth hold, dives, ascents |
| Path cross-track (sway) | 16.0 | — | 4.0 | Lateral offset from the path |
| Path heading (yaw) | 9.0 | — | 2.0 | Nose alignment with the path |
| Buoy servo (yaw) | 9.0 | 0.6 | 1.6 | Pixel-error alignment |
| Buoy servo (sway) | 3.5 | 0.0 | 0.8 | Lateral trim during approach |
| Buoy servo (vertical rate) | 0.45 | 0.05 | 0.06 | Commanded climb/descend rate |
| Bin station-keeping (surge) | 6.0 | — | 1.6 | Fore/aft position over bin |
| Bin station-keeping (sway) | 6.0 | — | 1.6 | Lateral position over bin |

#### Explaining the parameter table

Read the table by loop *purpose*, not by number.

**The depth loop has the only significant integral term (Ki = 2.0).**
This is not arbitrary: the vehicle is positively buoyant, so holding
depth requires a constant downward thrust. That steady-state offset is
exactly what an integrator provides. In simulation it converges to the
precise trim value the buoyancy requires. No other axis has a persistent
disturbance, so no other axis needs integral action.

**The path cross-track gain (16.0) is the highest in the system**
because the vehicle is fully actuated and its sway authority equals its
surge authority. This permits an unusual decomposition: cross-track
error is corrected by *translating sideways* rather than by turning. A
conventional differential-drive robot must fix a lateral offset with
yaw, which produces the familiar weaving oscillation. Decoupling the two
removes it entirely.

**The buoy servo's vertical gain is very small (0.45)** because its
output is not a thrust — it is a commanded *rate* in metres per second,
which the depth loop then converts to thrust. The units differ from
every other row in the table, which is why the number looks
inconsistent.

**Bin station-keeping uses matched surge and sway gains (both 6.0)**
because the task is symmetric: holding position over a bin has no
preferred direction. Note also that this loop contains no yaw term at
all — with a downward camera, both image axes map onto translation,
making it a station-keeping problem rather than a pointing one.

---

### 10.4 Tuning and Validation Methodology

Tuning follows a strict bottom-up sequence:

**1. Perception in isolation.** Detection and classification are
validated against captured imagery before any control loop is closed,
comparing computed errors against known ground truth.

**2. Individual loop tuning.** Each control loop is tuned separately
against the simulated vehicle dynamics, using step responses.

**3. Integration.** Tuned loops are chained inside the mission sequencer
only after passing individual validation. Loops sharing a physical axis
use a unified gain profile so vehicle dynamics remain predictable across
task transitions.

**4. Full-mission runs.** End-to-end autonomous runs with automated
pass/fail scoring on each mission task.

#### Automated verification

The full mission is scored automatically against seven independent
checks — dive, buoy touch, route selection, marker release, surfacing,
octagon containment by ground truth, and octagon containment as detected
by the vehicle itself.

The last two are deliberately separate. One asks *did the vehicle end up
in the right place*; the other asks *did the vehicle know it*. A run
where these disagree indicates a perception fault even though the
outcome was correct — a distinction that would be invisible if the two
were collapsed into a single check.

---

## 11. Results and Development Status

### 11.1 Full-Mission Result

The complete sequence executes end to end in simulation:

```
DIVE → LINE_FOLLOW → BUOY_APPROACH → BUOY_TOUCH → BUOY_BACKOFF
     → FORK (route selected) → BIN_APPROACH → BIN_DESCEND
     → BIN_HOLD → BIN_DROP ⇄ BIN_HOLD → BIN_DROP → BIN_DONE
     → OCTAGON_ARRIVE → OCTAGON_HOLD → OCTAGON_ASCEND → SURFACED
```

Representative run:

| Event | Result |
|---|---|
| Buoy touched | Committed at 0.89 m range |
| Route selected | 2 legs detected, correct branch taken |
| Marker 1 released | 5.6 cm from bin centre, 6.2 cm/s ground speed |
| Marker 2 released | 4.0 cm from bin centre, 5.2 cm/s ground speed |
| Surfaced | 0.12 m depth |
| Octagon containment | 2.99 m from centre; 35/36 frames confirmed |

#### Explaining the results

**Marker placement of 4–6 cm** should be read against the bin's 32 cm
half-width — roughly a 5× margin. The ground-speed figures matter
equally: releasing at 5 cm/s means the marker falls almost vertically,
whereas an earlier uncorrected release at 42–57 cm/s relied on margin
rather than accuracy.

**Octagon containment at 2.99 m from centre** sits inside the 3.26 m
apothem. The 35/36 frame confirmation is the vehicle's own detector
agreeing with ground truth — the independent check described in
Section 10.4.

### 11.2 Component Validation

| Test | Conditions | Result |
|---|---|---|
| Buoy detection | 3 water-tint levels | Centre exact; radius within 0.8 px |
| Bin symbol classification | 3 altitudes × 3 tints | 18/18 correct; position within 2 mm |
| Depth acquisition | vs. true vertical dynamics | Settles ≈ 2 s, ≈ 1 cm overshoot |
| Vertical estimator | modelled sensor chain | Chatter 3.67 N → 0.08 N |
| Octagon containment | inside vs. 4 m outside | 5.8× separation in discriminating metric |

### 11.3 Development Status

| Mission task | Status |
|---|---|
| 1 — Dive and path following | **Demonstrated in simulation** |
| 2 — Buoy touch | **Demonstrated in simulation** |
| 3 — L-bar crossing | Designed; currently cleared with 0.5 m margin |
| 4 — Marker dropping | **Demonstrated in simulation** |
| 5 — Torpedo through cut-out | Designed; arena target built |
| 6 — Surfacing in octagon | **Demonstrated in simulation** |

Four of six mission tasks are implemented and demonstrated end to end.
The remaining two are designed in detail, and the simulated arena has
been prepared so each is physically realisable. The architecture is
structured so each requires one perception module and a small number of
FSM states, reusing the existing depth cascade and path follower
unchanged.

---

## 12. Known Limitations

Recorded explicitly, as these set the agenda for hardware trials.

| Limitation | Consequence | Mitigation |
|---|---|---|
| Flat viewport refraction not modelled | Monocular range reads ~25 % short behind a flat port | Intrinsics read from `CameraInfo`; underwater calibration substitutes without code change. Dome port specified |
| No optical attenuation model | Real HSV values shift more than modelled, especially red at range | Controlled vehicle lighting; thresholds re-derived from pool imagery |
| Rolling shutter not modelled | Angle estimates acquire turn-rate-dependent bias (0.2–2.1°) | Global-shutter forward camera specified |
| No leak/under-voltage abort state | Hardware monitoring exists; software does not act on it | Fault state pre-empting all others, scheduled before CDR |
| Mesh collision unsupported in physics engine | Torpedo cut-out collision is a rectangular approximation | Physics gives pass/block; precise scoring requires geometric check |

---

## 13. Forward Work to CDR

1. Implement L-bar crossing and torpedo firing behaviours.
2. Enable multi-buoy touching — the machinery exists; currently
   configured for a single buoy.
3. Add the fault state: leak and under-voltage detection pre-empting all
   mission states and driving the vehicle to the surface.
4. Underwater camera calibration in the final housing.
5. Re-derive HSV thresholds from pool imagery under vehicle lighting.
6. Replace the simulated pressure model with the real driver and re-tune
   the estimator against measured noise.
