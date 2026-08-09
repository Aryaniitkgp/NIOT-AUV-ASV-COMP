# PDR Sections 6–12 — Revised

> Replacement text for Sections 6, 7, 8, 9 and revision notes for 10, 11,
> 12. Written to match the eight-thruster vehicle and the autonomy stack
> that actually runs. Editing notes in blockquotes; delete before
> submission.

---

# 6. Electronics System

## 6.1 Overview

The electronics architecture separates high-bandwidth perception from
real-time actuation, and keeps high-current propulsion paths electrically
isolated from sensitive logic. This manages computational load and
prevents thruster-generated noise from reaching the processors.

## 6.2 Components List

| Component | Model | Function |
|---|---|---|
| Main Computer | Raspberry Pi 5 (8 GB) | ROS 2 perception, vertical state estimation, mission state machine, control, logging |
| Flight Controller | Blue Robotics Navigator | Sensor aggregation and hardware PWM generation |
| IMU | XSENS MTi-630R | 6-DOF inertial data for vertical state estimation |
| Forward Camera | Global-shutter colour camera (IMX296/AR0234 class) | Buoy, obstacle and torpedo-target detection; visual servoing |
| Downward Camera | Raspberry Pi Camera Module 3 Wide | Orange path tracking, bin detection, visual ground-speed estimation |
| ESC | **8×** Blue Robotics Basic ESC | Battery-to-thruster power interface |
| Thrusters | **8×** Blue Robotics T200 | 4 horizontal (vectored 45°), 4 vertical (corners) |
| Battery | 2× 4S LiPo | Main power supply |
| Pressure Sensor | Bar30 (MS5837) | Depth measurement |
| Underwater Lights | 2× Blue Robotics Lumen R2 | Controlled illumination for colour segmentation |
| Marker Dropper Servo | DSServo DS3225 | Rotary release, one marker per actuation |
| Torpedo Firing Servo | DSServo DS3235 | Latch release for spring-loaded launcher |
| Power Monitors | 2× INA219 | Per-pack voltage and current |

> **Changes from the previous revision, with reasons:**
>
> **Optical flow sensor removed.** The PMW3901 is a fixed-focus sensor
> designed for aerial use over textured ground in good light. Its focus
> band does not cover the vehicle's full operating altitude, and a pool
> floor provides neither the texture density nor the illumination it
> requires. Critically, it degrades *silently* — it returns a velocity
> number with no confidence indication, so a failed lock is
> indistinguishable from a genuine zero. Horizontal velocity is instead
> derived from the downward camera (Section 9.2), which is already
> carried, provides an inspectable image, and has autofocus. This also
> removes one hull penetration.
>
> **Forward camera changed** from Intel RealSense D435i to a
> global-shutter colour camera. The RealSense derives depth from an
> 850 nm structured-light projector; water absorbs 850 nm at roughly
> 4 m⁻¹, so over the out-and-back path the pattern is >98 % attenuated
> by 0.5 m and the depth stream is unusable. Range is obtained
> geometrically instead (Section 9.3). Global shutter is preferred
> because the vehicle's angle measurements — path heading, bar
> orientation — acquire a turn-rate-dependent bias under rolling shutter.
>
> **Lights added.** The entire perception stack is HSV colour
> segmentation. Ambient pool illumination varies with time of day and
> surface state, and water attenuates red first. Controlled lighting is
> what makes colour thresholds repeatable between runs.
>
> **Compute upgraded** to Pi 5, and booting from NVMe rather than
> microSD. SD-card corruption after hard power cycles is a common
> failure mode in vehicles that are killed mid-write.

## 6.3 Component Placement

| Component | Position |
|---|---|
| Main Computer | Electronics enclosure |
| Flight Controller | Electronics enclosure |
| IMU | Electronics enclosure |
| Pressure sensor | Electronics enclosure — rear end cap |
| Forward camera | Electronics enclosure — front, behind dome port |
| Downward camera | Electronics enclosure — bottom exterior |
| ESC ×8 | Electronics enclosure, isolated tray |
| Lights ×2 | Outer frame rails, angled outboard |
| Battery ×2 | Battery enclosure |

> **Dome port on the forward camera.** A flat viewport refracts light
> at the water–air boundary, scaling effective focal length by ~1.33 and
> narrowing the field of view by about 25 %. Two consequences: monocular
> range estimates read ~25 % short unless recalibrated underwater, and
> the already-narrow 60° field of view shrinks further — which matters
> because several targets (the L-bar corner, the octagon ring) leave the
> frame at close range. A dome port admits light perpendicular to its
> surface at every point, eliminating the refraction. The camera's
> entrance pupil must sit at the dome's centre of curvature; mounted
> off-centre a dome behaves as a lens and is worse than a flat port.
>
> **Lights mounted wide and angled outboard**, not beside the lens.
> Lights adjacent to the optical axis illuminate suspended particles
> directly in front of the camera and produce backscatter — the same
> effect as headlights in fog.

## 6.4 Connection Analysis

Four divisions, addressing the three dominant failure modes in marine
robotics: electrical noise, processing bottlenecks, and control latency.

### 1. Power distribution and electrical isolation

Two 4S LiPo packs feed a Power Distribution Board through XT60
connectors. Delivery splits into two paths:

**Propulsion rail** — raw 4S direct to the eight ESCs, which commutate
it into 3-phase drive for the thrusters.

**Logic rail** — raw 4S into a 5 V/6 A BEC producing a regulated rail
for the Navigator, which shares it with the Pi. This isolates the
processors from the voltage transients and EMI the thrusters generate.

### 2. High-level processing and perception (Raspberry Pi 5)

The Pi runs the ROS 2 graph: both camera pipelines, the vertical state
estimator, the mission state machine, and all control loops. Keeping
this on the Pi leaves the flight controller free for real-time I/O.

- **Forward camera** — USB 3.0, colour frames for buoy, obstacle and
  target detection.
- **Downward camera** — 15-pin CSI ribbon, direct to the Pi's image
  signal processor, for path tracking and bin work.
- **IMU** — USB 2.0 serial, high-rate inertial data.

### 3. Low-level sensor aggregation (Navigator)

The Navigator stacks beneath the Pi on the 40-pin GPIO header and acts
as sensor aggregator and hardware interface:

- **Bar30** — I²C, absolute pressure, converted to depth.
- **INA219 ×2** — I²C, per-pack voltage and current.

These are passed up the GPIO header into the Pi for state estimation.

### 4. Closed-loop actuation and payload control

The Pi's control loops publish body-frame commands. A thruster mixer
node converts these into eight individual thrust demands, which cross
the GPIO connection to the Navigator:

- **PWM channels 1–8** → the eight ESCs.
- **PWM channel 9** → marker dropper servo.
- **PWM channel 10** → torpedo launcher servo *(release, not
  "reclamation" — the mechanism fires the torpedo, per Section 4.4)*.

## 6.5 Connection Diagram Rationale

- **USB 3.0, forward camera** — bandwidth for uncompressed colour frames
  without added latency.
- **CSI ribbon, downward camera** — bypasses USB, giving zero-copy frame
  access for the highest-rate vision loop in the system.
- **USB 2.0 serial, IMU** — isolated path for high-rate inertial data
  feeding the vertical estimator's prediction step.
- **GPIO stacking, Pi → Navigator** — eliminates external signal wiring,
  minimising EMI exposure and latency.
- **I²C, Bar30 and INA219** — low-overhead two-wire bus, appropriate for
  these low-rate telemetry streams.
- **PWM 1–10** — hardware-timed pulses, isolating actuator timing from
  operating-system scheduling.
- **XT60 → PDB → BEC** — dedicated regulator forming an electrical
  barrier against thruster back-EMF and brownout.

---

# 7. Power System

## 7.1 Power Requirements

| Component | Source | V | I (A) | Limit | P (W) |
|---|---|---:|---:|---|---:|
| Thrusters ×8 | Raw battery | 16 | 6.25 each | 25 A each | 800 |
| Marker dropper | AUX 5 V (UBEC) | 5 | 2 | 3 A | 10 |
| Torpedo launcher | AUX 6 V (UBEC) | 6 | 3 | 5 A | 18 |
| Raspberry Pi 5 | Pi power plane | 5 | 4 | 5 A | 20 |
| Navigator | 5 V/6 A BEC | 5 | 0.5 | 6 A | 2.5 |
| Forward camera | Pi USB 3.0 | 5 | 0.7 | 700 mA | 3.5 |
| Downward camera | Pi CSI | 3.3 | 0.25 | 350 mA | 0.82 |
| Lights ×2 | Raw battery | 16 | 1.5 each | 3 A each | 48 |

> Pi 5 draws more than the Pi 4B it replaces (20 W vs 15 W), and the two
> lights add 48 W. Both are accounted for above.

## 7.2 Main Battery Selection

A 4S LiPo configuration (14.8 V nominal, 16.8 V charged) is used
because:

- **Thruster voltage match** — 14.8 V sits on the T200 efficiency curve,
  delivering full thrust without exceeding thermal limits.
- **Reduced current and heat** — higher rail voltage lowers the current
  needed for equivalent power, reducing resistive heating inside a
  sealed hull and avoiding ESC thermal throttling.
- **Regulator headroom** — stays above the 5 V BEC dropout even at low
  state of charge (~13.2 V), so thruster sag cannot brown out the Pi.
- **Energy density** — high C-rating chemistry supplies burst current for
  multi-axis manoeuvres while keeping mass within the buoyancy budget.

**Dual-pack architecture** isolates high-current thruster draw from
logic, suppressing EMI and voltage transients. **Mission endurance** at
a typical 35 A continuous multi-axis draw gives roughly 20 minutes from
12000 mAh, sufficient for the full course. **Field maintenance** is by
poolside pack swap in the isolated lower pod, without opening the
electronics chamber.

## 7.3 Battery Specifications

| Parameter | Specification |
|---|---|
| Chemistry | Lithium Polymer |
| Configuration | 4S |
| Nominal voltage | 14.8 V |
| Capacity | 12000 mAh total |
| C-rating | High-C (15–20 C) thruster pack; low-C logic pack |
| Mass | 5 kg |
| Endurance | ~20 min |

## 7.4 Power Distribution

Two independent chains:

```
LOGIC PACK                    THRUSTER PACK
    │                              │
  FUSE                           FUSE
    │                              │
Current sensor (INA219)      Current sensor (INA219)
    │                              │
5 V regulator              HARDWARE KILL SWITCH
    │                              │
    ├── Raspberry Pi 5         ESC bank (×8)
    └── Navigator                  │
                              Thrusters (×8)
```

> Note the kill switch sits on the **thruster** chain only. Cutting
> propulsion leaves the Pi powered and logging, which preserves the
> record of whatever caused the abort.

## 7.5 Battery Management

**Charging** — balance charger, per-cell monitoring, ≤1 C, inside a
fire-containment bag, never unattended. Charged to full only shortly
before a run.

**In-mission monitoring** — per-pack voltage and current via INA219 on
I²C. Either pack going under-voltage triggers the failsafe; losing
either is mission-ending regardless of the other.

**Storage** — packs idle beyond 48 hours are brought to ~3.8 V/cell.
Visual inspection before every session; damaged cells retired, not
charged.

**Failure response** — a swollen or damaged cell is isolated into a
fire-safe container and disposed of per LiPo guidelines. The hardware
kill switch removes propulsion power on any regulator or pack anomaly.

---

# 8. Software & Simulation Architecture

## 8.1 Overview

The autonomy stack is built on ROS 2 Humble and comprises approximately
4,100 lines across nine modules. It has been developed and validated
against a full simulation of the competition arena, in which four of the
six mission tasks currently execute end to end.

## 8.2 Architectural Details

### Governing constraint

The vehicle has no absolute horizontal position reference. There is no
DVL, and a magnetometer is unreliable indoors near steel structure and
the vehicle's own thruster magnets. Every horizontal decision must
therefore be made from what the cameras see at that instant.

This produces the architecture's defining property: **the vehicle
inspects rather than remembers.** It does not plan in world coordinates.
It centres over whatever target is beneath it, reads it, and acts —
because "return to where the X bin was" is not an instruction it can
execute.

### Module structure

| Module | Lines | Responsibility |
|---|---:|---|
| `line_buoy.py` | 1581 | Mission sequencer — 17-state finite state machine |
| `buoy.py` | 573 | Multi-colour buoy detection, ranking, visual servoing |
| `bluerov2_native_bridge.py` | 550 | Simulator ↔ ROS 2 bridge and sensor models |
| `line_follow.py` | 448 | Path detection, cross-track control, fork selection |
| `marker.py` | 396 | Bin detection, symbol classification, station keeping |
| `thruster_mixer.py` | 197 | Body-frame commands → 8 thrusters |
| `octagon.py` | 196 | Surfacing-ring detection and containment test |
| `depth_filter.py` | 145 | Pressure + inertial fusion |
| `depth_control.py` | 45 | Depth PID |

### Separation of axes

One structural decision underpins the design:

> **Exactly one behaviour commands the horizontal axes at any instant,
> but the depth loop runs continuously beneath every state.**

The reason is physical. The vehicle is positively buoyant by
approximately 1.4 N — it floats to the surface the moment nothing holds
it down. Depth can therefore never be handed between behaviours the way
steering can. Behaviours influence depth by *moving the setpoint*, never
by commanding thrust:

```
pixel error → commanded rate (m/s) → depth setpoint → PID → thrust
  outer loop        integrator        inner loop
```

Keeping the buoyancy-trim integrator in exactly one place prevents two
controllers fighting for the same axis.

### Perception layer

Classical computer vision — colour segmentation and contour analysis —
rather than a learned detector. Targets are of known size, shape and
colour; a classical pipeline is deterministic, needs no training data,
runs on embedded hardware, and can be justified analytically.

Two techniques recur:

**Minimum enclosing circle for spheres.** The buoys are spheres, so a
minimum enclosing circle fits better than a bounding box: it yields
centre and radius in one operation, and contour-area to circle-area
gives a scale-free *circularity* measure. This separates a sphere
(≈0.97) from the flat square torpedo target (≈0.38) at any range.

**Contour topology.** A ring has an inner contour; a cross does not.
This distinguishes the O and X bins, and is the same cue the heart
cut-out will use.

**Thresholds are measured, not modelled.** HSV bounds come from captured
frames, not from material colours. This matters: rendered scenes are
2–2.5× darker than an alpha blend of the material predicts, because the
scene is lit by one source through a metre of water.

| Surface | Predicted from material | Measured |
|---|---|---|
| White bin symbol | (106, 47, 226) | **(0, 0, 97)** |
| Navy bin | (111, 188, 137) | (112, 131, 68) |
| Grey floor | (108, 70, 168) | (120, 6, 80) |

### Mission sequencer

A 17-state finite state machine on a fixed 20 Hz timer, operating on the
most recent frame from each camera rather than inside image callbacks.
This decouples the control period from camera jitter and gives every
derivative and integrator a constant timestep.

```
INIT → DIVE → LINE_FOLLOW ⇄ BUOY_APPROACH → BUOY_TOUCH → BUOY_BACKOFF
                    │                ↕
                    │           BUOY_SEARCH
                    ├──→ BIN_APPROACH → BIN_DESCEND → BIN_HOLD
                    │         │              ↑           ↓
                    │    BIN_REJECT      BIN_DONE ← BIN_DROP
                    └──→ OCTAGON_ARRIVE → OCTAGON_HOLD
                                        → OCTAGON_ASCEND → SURFACED
```

Every failure path is bounded: lost targets trigger search states,
search states time out, and attempt counters abandon a task rather than
trapping the run in a retry loop.

## 8.3 Simulation Environment

| Item | Selection |
|---|---|
| Simulator | Gazebo Sim 8 (Harmonic), `dartsim`, 1 ms step |
| Middleware | ROS 2 Humble |
| Arena | 25 × 20 m tank, 2.5 m water column, full course |

**Sensor realism was added deliberately.** The simulator provides
perfect, instantaneous ground truth, which flatters any controller tuned
against it. The pressure sensor is therefore modelled with 30 ms
latency, 0.3 mbar RMS noise and 0.2 mbar quantisation, applied in
physical order — delay, then noise, then quantisation, because an ADC
digitises an already-noisy analogue signal. IMU noise and bias are
modelled at MTi-630R grade.

The stack consumes exactly **two scalars** of ground truth — depth and
depth rate — and both are replaced by the modelled sensor chain. No
module uses ground-truth horizontal position or heading. This keeps the
simulation-to-hardware boundary narrow and testable.

## 8.4 Digital Twin Vehicle Profile

The simulated vehicle mirrors the physical one:

**Actuator layout — eight thrusters.** Four horizontal, vectored at 45°
about the centre of gravity, providing coupled surge, sway and yaw.
Four vertical at the frame corners, driven identically for heave.

Corner placement of the vertical cluster is what makes the zero-pitch
property structural rather than tuned: with the four units symmetric
fore and aft about the centre of gravity, equal drive produces exactly
zero pitch moment.

**Virtual sensors** — camera pair matching the real optics, IMU with
realistic noise and bias, and a Bar30 model with the latency,
quantisation and noise characteristics described above.

---

# 9. Navigation Stack

> **This section replaces the previous EKF-and-waypoints description.**
> That architecture assumed an absolute horizontal position estimate
> which the sensor suite cannot provide. What follows is what the stack
> actually implements.

## 9.1 Sensor Fusion and Data Interfaces

| Topic | QoS | Rationale |
|---|---|---|
| Camera streams | Best effort, depth 1 | Latency matters more than delivery; a stale frame is worse than a dropped one |
| `/bluerov2/depth`, `/bluerov2/imu/data` | Best effort, depth 1 | High rate; the estimator tolerates occasional loss |
| `/cmd_surge`, `/cmd_sway`, `/cmd_yaw`, `/cmd_heave` | Reliable, depth 1 | Every command must reach the mixer; depth 1 prevents acting on stale demands |
| `/mission/state` | Reliable, depth 1 | Monitoring and logging must not miss transitions |

## 9.2 Horizontal Velocity — Visual Odometry

With no DVL, horizontal velocity is measured from how fast a tracked
feature slides across the downward camera, scaled by altitude:

```
metres_per_pixel = altitude / focal_length
```

Altitude comes from the Bar30 and known target height. This is the
measurement that gates marker release: a released marker inherits the
vehicle's horizontal velocity, and commanding zero is not the same as
being stopped, because the hull coasts.

> The estimator publishes no value until it has accumulated sufficient
> motion history. A filter initialised at zero reports "stopped" on its
> first sample regardless of true speed — a fault observed and corrected
> during development, where markers were released at a reported 0 cm/s
> while actually travelling at 42–57 cm/s.

## 9.3 Vertical State Estimation

Depth control requires vertical *velocity*, but the pressure sensor
measures only position. Differencing consecutive samples amplifies noise
severely — for 2 mm resolution at 20 Hz:

```
σ_v = σ_z·√2/Δt = 0.002 × 1.414 / 0.05 = 0.057 m/s
```

Through the derivative gain this produces thrust chatter **larger than
the 1.4 N of buoyancy the loop is trimming**. The scaling is perverse:
sampling faster makes the naive derivative worse.

Low-pass filtering trades noise against phase lag, and no setting is
both quiet and responsive. The solution is to fuse pressure with
inertial acceleration in a two-state Kalman filter over `[z, ż]`. The
prediction step carries fast dynamics from the accelerometer; the slow,
absolute, drift-free pressure measurement corrects its bias:

| | Pressure (Bar30) | IMU accelerometer |
|---|---|---|
| Rate | 20 Hz | 1 kHz |
| Noise | jittery | smooth |
| Long term | never drifts | drifts (bias integrates) |

The measurement update also corrects **velocity**, through the
position–velocity correlation term in the covariance — which is how a
clean rate estimate is extracted from a position-only sensor.

**Measured:**

| Configuration | Thrust chatter | Estimator error |
|---|---:|---:|
| Raw backward difference | **3.67 N** | 1.1 cm |
| Constant-velocity filter | 0.56 N | 1.0 cm |
| **Pressure + IMU fused** | **0.08 N** | **0.4 cm** |

A factor of **46** reduction in chatter, with no change to controller
gains.

## 9.4 Range Estimation

Range to a target of known size follows the pinhole model:

```
Z = R_real · f / R_pixels
```

For the L-bar, ranging from angular *depression* below the optical axis
is preferred over apparent thickness, because the bar's known height
gives a far better-conditioned measurement:

| Range | Error per pixel, via thickness | via depression |
|---|---|---|
| 1.5 m | ±0.07 m | ±0.01 m |
| 3.0 m | ±0.30 m | ±0.03 m |

**Camera intrinsics are read from a `CameraInfo` topic, not hard-coded**,
so an underwater calibration can be substituted without code changes.
This is necessary because a flat viewport scales effective focal length
by ~1.33; using a dry calibration underwater makes every range read
about 25 % short.

## 9.5 Task Sequencing in Place of Waypoints

The stack carries no waypoint list and no world-frame trajectory
generator, because it has no absolute horizontal position to plan
against. Sequencing is by visual recognition:

| Transition | Trigger |
|---|---|
| Dive complete | Depth held within tolerance for a settle period |
| Buoy engagement | Apparent radius exceeds threshold for N consecutive frames |
| Route selection | Two path branches resolved as separate clusters; preferred side chosen |
| Bin engagement | Navy region exceeding area threshold, passing squareness gate |
| Marker release | Centred within tolerance **and** visual ground speed below threshold |
| Course end | Path lost persistently after the marker task, at the octagon centre |
| Surfaced | Pressure-derived depth below threshold |

---

# 10. Control System — Required Corrections

> **Section 10.2 and 10.4 still describe six thrusters and must be
> updated to match Sections 3, 5 and 6.**

## 10.2 — Allocation matrix

The heading "6-Thruster Allocation Matrix" and its contents are
inconsistent with the eight-thruster vehicle. A 4×6 matrix cannot
multiply against eight thruster forces.

Replace with:

```
τ = [Fx, Fy, Fz, Mz]ᵀ            T = [T1 … T8]ᵀ            T = B⁺τ
```

where **B is 4×8**. The horizontal contributions, derived from thruster
mounting geometry:

| Thruster | Surge | Sway | Yaw |
|---|---:|---:|---:|
| T1 front-right | −0.707 | −0.707 | −0.164 |
| T2 front-left | −0.707 | +0.707 | +0.164 |
| T3 rear-right | +0.707 | −0.707 | +0.164 |
| T4 rear-left | +0.707 | +0.707 | −0.164 |

T5–T8 are vertical, contributing to heave only and driven identically.

**On saturation the entire command vector is scaled down** rather than
clipping an individual channel — clipping changes the direction of the
resultant force, silently converting a commanded turn into a turn plus
unintended drift.

## 10.4 — Tuning table

The "Technical Purpose" column references a "vertical cluster (T5–T6)".
This should read **T5–T8**.

> The gain values in this table should be reconciled against the
> implemented controllers before submission — the depth loop in the
> current stack uses a different gain set, tuned against the modelled
> sensor chain rather than ideal ground truth.

---

# 11. Communication Systems — Required Corrections

**11.1** — the interface table needs three edits:

| Row | Issue |
|---|---|
| `Navigator – ESCs (6 T200)` | Should read **8 T200** |
| Optical flow UART link | Remove — sensor deleted (Section 6.2) |
| Forward camera | Update model to match the global-shutter selection |

**11.2 Kill Switch** — the text states the switch removes power from
"all six T200 ESCs". Should read **eight**. The described behaviour is
otherwise correct and consistent with Section 7.4: the switch acts on
the thruster pack only, leaving the Pi powered and logging.

---

# 12. Safety Analysis — Required Corrections

**12.1** — two counts to update:

- "cuts thruster-pack power to all 6 ESCs" → **8 ESCs**
- "guards on all 6 T200 thrusters" → **8 T200 thrusters**

**12.2 FMEA** — the *EKF divergence* row lists mitigation as
"Multi-sensor fusion (IMU+DVL+depth+visual landmarks)". The vehicle
carries no DVL, and there is no horizontal EKF. Suggested replacement:

| Failure mode | Effect | Severity | Mitigation |
|---|---|---|---|
| Loss of visual target | Task cannot complete | Medium | Bounded search states; per-task attempt limits; task abandoned rather than retried indefinitely |
| Depth estimate divergence | Incorrect depth hold | Medium | Pressure provides an absolute, drift-free reference; depth setpoint clamped to a safe operating band |

> **Gap worth recording:** the software has no leak or under-voltage
> abort state. Sections 11.2 and 12.1 both describe one. Implementing a
> fault state that pre-empts all others and drives the vehicle to the
> surface is scheduled before CDR.
