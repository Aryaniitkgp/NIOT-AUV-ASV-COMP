# Preliminary Design Report — Software Stack

**National Student Autonomous underwater Vehicle Competition (SAVe)**
Autonomy, Perception and Control Subsystem

> *Editing notes for the author are shown in blockquotes like this one and
> should be deleted before submission. Figure placeholders are marked
> **[FIGURE n]** with a description of what to insert and a suggested
> caption.*

---

## 1. Objective

The software stack is responsible for turning a passively stable, fully
actuated underwater vehicle into an autonomous one: it must perceive the
competition arena through two cameras, estimate the vehicle's vertical
state from inertial and pressure sensing, and sequence six distinct
mission tasks without any operator input or any external positioning
reference.

Specifically, the stack must:

1. Dive from the launch point to a working depth and hold it.
2. Detect and follow the orange path along the floor of the arena.
3. Detect a set of coloured buoys, select the nearest, and touch it.
4. Detect an "L" shaped PVC bar and pass over it.
5. Choose a route at the fork, identify a target bin by its painted
   symbol, and release markers into it.
6. Identify a heart-shaped cut-out and fire a torpedo through it.
7. Surface fully inside the octagonal boundary that ends the course.

The overriding design constraint is that **the vehicle has no absolute
horizontal position reference**. There is no DVL, no acoustic
positioning, and no usable magnetic heading indoors. Every horizontal
decision the vehicle makes must therefore be derived from what its
cameras can see at that instant. This single constraint shapes the entire
architecture and is returned to throughout this report.

> **[FIGURE 1]** Photograph or render of the complete simulated arena,
> viewed from above at an angle, with the orange path, flowers, L-bar,
> bins, cupid target and both octagons visible.
> *Caption: "The SAVe competition arena as reconstructed in Gazebo."*

---

## 2. Literature and Design Basis

The approach draws on three well-established bodies of work:

| Area | Basis | Application here |
|---|---|---|
| Image Based Visual Servoing (IBVS) | Chaumette & Hutchinson, *Visual Servo Control* | Buoy approach and bin centring are driven directly by pixel error, with no intermediate pose estimate |
| Pinhole camera geometry | Hartley & Zisserman, *Multiple View Geometry* | Monocular ranging from known target size; camera calibration |
| Cascaded control / state estimation | Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control* | Inner depth loop with outer visual loops; complementary fusion of pressure and inertial sensing |

The decision to use classical computer vision (colour segmentation and
contour analysis) rather than a learned detector is deliberate at this
stage. The targets are geometrically simple, of known size and known
colour, and a classical pipeline is deterministic, requires no training
data, runs comfortably on embedded hardware, and — critically for a
design review — can be reasoned about and justified analytically. Each
threshold in this report is derived from the arena's material properties
rather than tuned by trial and error.

---

## 3. System Architecture

### 3.1 Block diagram

> **[FIGURE 2]** Redraw the following as a proper block diagram.
> *Caption: "Software stack architecture. Perception and control are
> separated by the mission sequencer; only one behaviour commands the
> horizontal axes at any time, while the depth loop runs beneath all
> states."*

```
   ┌──────────────────────── GAZEBO SIMULATION ────────────────────────┐
   │  front camera   down camera    IMU    pressure    thruster cmds   │
   └────┬───────────────┬────────────┬────────┬─────────────▲──────────┘
        │               │            │        │             │
   ┌────▼───────────────▼────────────▼────────▼─────────────┴──────────┐
   │              bluerov2_native_bridge.py  (gz <-> ROS 2)            │
   │   images + CameraInfo | IMU | Bar30 sensor model | odometry       │
   └────┬───────────────┬────────────┬────────┬─────────────▲──────────┘
        │               │            │        │             │
        │               │      ┌─────▼────────▼─────┐       │
        │               │      │  depth_filter.py   │       │
        │               │      │  2-state Kalman    │       │
        │               │      │  (z, vz)           │       │
        │               │      └─────────┬──────────┘       │
        │               │                │                  │
   ┌────▼─────┐  ┌──────▼──────┐  ┌──────▼──────────┐       │
   │ buoy.py  │  │line_follow  │  │ depth_control.py│       │
   │ marker.py│  │    .py      │  │  PID -> heave   │       │
   │(percep-  │  │(perception  │  └──────┬──────────┘       │
   │ tion +   │  │ + control)  │         │                  │
   │ control) │  │             │         │                  │
   └────┬─────┘  └──────┬──────┘         │                  │
        │               │                │                  │
   ┌────▼───────────────▼────────────────▼──────────┐       │
   │        line_buoy.py — MISSION SEQUENCER        │       │
   │        13-state finite state machine           │       │
   │   arbitrates surge / sway / yaw / depth target │       │
   └────────────────────┬───────────────────────────┘       │
                        │ /cmd_surge /cmd_sway              │
                        │ /cmd_yaw   /cmd_heave             │
   ┌────────────────────▼───────────────────────────┐       │
   │            thruster_mixer.py                    ├───────┘
   │   body-frame commands -> 8 thruster forces      │
   └─────────────────────────────────────────────────┘
```

### 3.2 Module inventory

| Module | Lines | Responsibility |
|---|---:|---|
| `bluerov2_native_bridge.py` | 550 | Gazebo ↔ ROS 2 bridge; camera, IMU, simulated Bar30, odometry, marker spawning |
| `thruster_mixer.py` | 197 | Maps body-frame surge/sway/yaw/heave onto 8 thrusters |
| `line_follow.py` | 368 | Orange path detection and cross-track control |
| `buoy.py` | 502 | Multi-colour buoy detection, ranking, and visual servoing |
| `marker.py` | 329 | Bin detection, O/X classification, station-keeping, visual ground-speed |
| `depth_control.py` | 45 | Depth PID |
| `depth_filter.py` | 145 | Pressure + inertial fusion for vertical state |
| `line_buoy.py` | 1127 | Mission sequencer (finite state machine) |
| **Total** | **3263** | |

### 3.3 Design principle: separation of axes

The architecture rests on one structural decision:

> **Exactly one behaviour commands the horizontal axes at any instant,
> but the depth loop runs continuously underneath every state.**

The reason is physical. The vehicle displaces 13.14 kg of water against a
13.0 kg mass, leaving it **+1.36 N positively buoyant**. It floats to the
surface the moment nothing is actively holding it down. Depth can
therefore never be handed between behaviours the way steering can.

Behaviours influence depth by *moving the depth setpoint*, never by
commanding thrust directly. This makes the vertical axis a cascade:

```
   pixel error (e_y)  →  commanded rate (m/s)  →  depth setpoint  →  PID  →  thrust
       outer loop            integrator            inner loop
```

Keeping the buoyancy-trim integrator in exactly one place is what stops
two controllers fighting over the same axis.

---

## 4. Simulation Environment

### 4.1 Platform

| Item | Selection |
|---|---|
| Simulator | Gazebo Sim v8 (Harmonic), `dartsim` physics |
| Middleware | ROS 2 Humble |
| Vehicle model | BlueROV2 Heavy, 8-thruster configuration |
| Physics step | 1 ms, real-time factor 1.0 |
| Vision | OpenCV 4 |
| Arena | 25 × 20 m tank, 2.5 m water column |

### 4.2 Vehicle parameters

These are taken from the vehicle model and are used directly in the
control design, not estimated:

| Parameter | Value | Consequence for control |
|---|---|---|
| Mass | 13.0 kg | — |
| Displaced mass | 13.14 kg | +1.36 N buoyant → active depth hold mandatory |
| Centre of buoyancy | 49 mm above CoG | Passive roll/pitch stability; no attitude control needed |
| Surge drag (quadratic) | −33.73 N·s²/m² | Terminal speed ≈ 0.82 m/s at cruise thrust |
| Sway drag | −54.16 N·s²/m² | — |
| Heave drag | −73.23 N·s²/m² | No linear damping term; D-term must use measured rate |
| Vertical thrusters | 4, symmetric about CoG in x | Heave produces **zero pitch moment** by construction |
| Max thrust | 40 N per thruster | — |

### 4.3 Sensor suite

| Sensor | Specification | Used for |
|---|---|---|
| Front camera | 640 × 480, 60° HFOV, f = 554.4 px | Buoys, L-bar, torpedo target, octagon |
| Down camera | 640 × 480, 60° HFOV, pitched 90° | Path following, bins |
| IMU | 1 kHz, noise modelled at MTI-630R grade | Vertical acceleration for state estimation |
| Pressure (Bar30) | 20 Hz, 0.2 mbar resolution, 30 ms latency | Depth |

> **[FIGURE 3]** Screenshot of the vehicle in Gazebo with both camera
> frusta visible, or a two-panel image showing the front and down camera
> views side by side.
> *Caption: "Vehicle sensor configuration. The down camera (pitched 90°)
> follows the path; the front camera handles all forward-facing targets."*

### 4.4 Camera frame conventions

Getting these wrong is the most common source of sign errors in a visual
servoing system, so they are stated explicitly. Body frame is FLU
(x forward, y port, z up).

**Front camera** (mounted at (0.2, 0, 0.05), unrotated):

| Image direction | Body direction | Control consequence |
|---|---|---|
| right | starboard (−y) | target right of centre → negative yaw and sway |
| down | down (−z) | target below centre → descend |

**Down camera** (mounted at (0, 0, −0.05), pitched 90°):

| Image direction | Body direction | Control consequence |
|---|---|---|
| up | forward (+x) | target low in frame is *behind* → negative surge |
| right | starboard (−y) | target right of centre → negative sway |

Note that the down camera loop contains **no yaw term at all** for bin
work — both image axes map onto translation, making it a
station-keeping problem rather than a pointing one.

---

## 5. Mission Sequencer

The mission is executed by a 13-state finite state machine.

> **[FIGURE 4]** Redraw as a proper state diagram.
> *Caption: "Mission finite state machine. Solid arrows are nominal
> transitions; dashed arrows are recovery paths."*

```
                    ┌──────┐
                    │ INIT │  wait for cameras + depth estimate
                    └───┬──┘
                        │
                    ┌───▼──┐
                    │ DIVE │  descend to cruise depth, settle 1 s
                    └───┬──┘
                        │
        ┌───────────────▼────────────────┐
   ┌───►│         LINE_FOLLOW            │◄──────────┐
   │    │  down camera: orange path      │           │
   │    │  front camera: buoy lookout    │           │
   │    │  down camera: bin lookout      │           │
   │    └───┬────────────────────┬───────┘           │
   │        │ buoy in range      │ bin in view       │
   │   ┌────▼─────────┐     ┌────▼──────────┐        │
   │   │BUOY_APPROACH │     │ BIN_APPROACH  │        │
   │   │  IBVS on     │     │ centre + read │        │
   │   │  pixel error │     │ O/X symbol    │        │
   │   └───┬──────┬───┘     └───┬───────┬───┘        │
   │       │      │ lost        │ wrong │ correct    │
   │       │  ┌───▼──────┐      │ symbol│            │
   │       │  │BUOY_     │  ┌───▼─────┐ │            │
   │       │  │SEARCH    │  │BIN_     │ │            │
   │       │  └───┬──────┘  │REJECT   ├─┼────────────┤
   │       │      │         └─────────┘ │            │
   │  ┌────▼──────▼──┐            ┌─────▼───────┐    │
   │  │  BUOY_TOUCH  │            │ BIN_DESCEND │    │
   │  └───────┬──────┘            └─────┬───────┘    │
   │  ┌───────▼──────┐            ┌─────▼───────┐    │
   │  │ BUOY_BACKOFF │            │  BIN_HOLD   │◄─┐ │
   │  └───────┬──────┘            └─────┬───────┘  │ │
   │          │                   ┌─────▼───────┐  │ │
   └──────────┘                   │  BIN_DROP   ├──┘ │
                                  └─────┬───────┘    │
                                  ┌─────▼───────┐    │
                                  │  BIN_DONE   ├────┘
                                  └─────────────┘
```

Every state runs at a fixed **20 Hz** on the most recent frame from each
camera, rather than being driven by image callbacks. This decouples the
control period from camera jitter and gives a constant `dt` to every
derivative and integrator in the system.

---

## 6. Mission Methods

### 6.1 Mission 1 — Path Following

**Perception.** The orange path is segmented in HSV, then morphologically
opened and closed. Rather than fitting a single line across the whole
contour, the mask is sampled in two horizontal bands: one directly
beneath the vehicle and one at a lookahead distance of 70 px. This is
deliberately more robust than a line fit, which becomes numerically
ill-conditioned as the path approaches horizontal in the image.

**Control.** The vehicle is fully actuated in the horizontal plane and its
sway authority equals its surge authority. This permits an unusual and
much better-behaved decomposition:

- **Cross-track error → sway.** The vehicle translates sideways onto the
  path.
- **Heading error → yaw.** Yaw only has to keep the nose aligned.

A conventional differential-drive robot must fix a lateral offset with
yaw, which produces the familiar weaving oscillation. Decoupling the two
removes it entirely. A small cross-track term is fed into yaw so the
vehicle also points back at the path rather than crabbing alongside it.

| Gain | Value |
|---|---|
| Cross-track proportional / derivative | 16.0 / 4.0 |
| Heading proportional / derivative | 9.0 / 2.0 |
| Cross-track → yaw coupling | 3.0 |
| Cruise thrust | 8.0 (≈ 0.82 m/s) |

Surge is scaled down when alignment is poor, so the vehicle slows into
corners and speeds up on straights.

> **[FIGURE 5]** Screenshot of the "Line Following — Down Camera" debug
> window showing the detected path, the near and lookahead sample points,
> and the overlay text.
> *Caption: "Path following. Green marker = sample directly beneath the
> vehicle; magenta = lookahead point used for steering."*

### 6.2 Mission 2 — Buoy Detection and Touch

**Perception.** Three flower buoys (red, green, yellow) are present. All
three colour bands are evaluated every frame. Because the buoys are
spheres, a minimum enclosing circle is a far better fit than a bounding
box: it yields the centre (u, v) and the radius `R_pixels` in a single
operation, and the ratio of contour area to enclosing-circle area gives a
scale-free **circularity** measure that rejects the orange path and the
support pipes.

**Target selection.** Under the pinhole model `Z = R_real · f / R_pixels`,
apparent radius is monotonic in inverse distance. The nearest buoy is
therefore simply the one with the largest apparent radius — a pure
image-space comparison that does not depend on the range model being
calibrated correctly. Once an approach commits, the colour is **latched**
so a rival buoy drifting into frame cannot steal the servo mid-manoeuvre.

**Control (IBVS).** Pixel errors are taken relative to the principal
point and normalised by half the image size so the gains are independent
of resolution:

```
    e_x = (u − u₀) / u₀        e_y = (v − v₀) / v₀
```

| Error | Drives | Rationale |
|---|---|---|
| `e_x` | yaw (primary), sway (trim) | With a single forward camera, horizontal pixel error *is* an angle — yaw is the axis that observes it. A small sway term prevents the vehicle arcing around the target. |
| `e_y` | depth-rate setpoint | Cascaded into the depth loop rather than commanded as thrust |

**Approach speed.** Surge is scaled inversely with apparent radius,
`v = k / R_pixels`, clamped to [0.8, 5.0]. Because `R_pixels ∝ 1/Z`, this
makes surge fall off roughly linearly with remaining distance — the
vehicle decelerates automatically as it closes, avoiding a high-impact
contact.

**Terminal phase.** Inside approximately 0.35 m the sphere overflows the
frame and detection stops being trustworthy. The final contact is
therefore made by dead reckoning from the last good range estimate: a
short open-loop surge impulse, then a controlled back-off.

> **[FIGURE 6]** Screenshot of the "Buoy Detection — Front Camera" window
> with multiple buoys detected, the selected target highlighted, and the
> range/error overlay visible.
> *Caption: "Multi-buoy detection. All candidates are circled; the
> selected (nearest) target is shown in green with its estimated range."*

### 6.3 Mission 3 — Obstacle Crossing (L-Bar)

> **Status: designed; arena prepared; not yet implemented.**

The obstacle is an L-shaped PVC bar standing upright across the path: a
horizontal crossbar at 0.5 m above the floor spanning the lane, joined at
one end to a vertical rod rising to 1.7 m.

**Perception.** The bar is segmented on colour and reduced to a minimum-
area rectangle, which yields centre, angle and length in one operation.
The angle gives the heading error required to cross perpendicular.

**Ranging — a design decision worth recording.** Two options exist:
ranging from the rod's apparent *thickness*, or from its angular
*depression* below the optical axis (the bar's height is known, and the
vehicle knows its own depth). The second is an order of magnitude better
conditioned:

| Range | Error per pixel, via thickness | via depression angle |
|---|---|---|
| 1.5 m | ±0.07 m | ±0.01 m |
| 3.0 m | ±0.30 m | ±0.03 m |

Ranging from a 0.06 m diameter rod is dominated by pixel noise; ranging
from a known height is not.

**A field-of-view constraint.** The L's corner is only visible when both
rods are in frame, which the 60° horizontal field of view permits only
beyond 2.6 m — and at that range the bar is barely 13 px wide. As the
vehicle closes, the corner leaves the frame. **The detection therefore
keys on the crossbar alone**, using the vertical rod as a confirming vote
when visible rather than as a gate.

**Control.** Align heading to cross perpendicular, raise the depth
setpoint to clear the bar, surge across, then descend and re-acquire the
path. No new control code is required — the climb is a setpoint change
into the existing depth cascade.

### 6.4 Mission 4 — Marker Dropping

**Route selection.** The path forks; the left branch leads to the bins,
the right to the torpedo target, and a connecting path joins the two.
Because both branches terminate at an octagon, the sequencer can either
complete one task and finish, or traverse the connecting path to attempt
both, as the remaining time allows.

**Bin detection.** Bins are navy squares. Detection uses HSV plus a
**squareness gate** (aspect ratio and fill extent), which is what keeps
the similarly-hued blue L-bar out of the results — a rod is not square.

**Symbol classification.** Each bin carries a white painted symbol: a
ring ("O") or a cross ("X"). The white threshold is applied *only inside
a bin that has already been located*, which makes it trivially reliable —
within the bin the only two things present are navy paint and white
paint. Two independent cues then classify the symbol:

| Cue | Ring (O) | Cross (X) |
|---|---|---|
| Circularity (area / enclosing circle) | 0.97 – 0.99 | 0.32 – 0.33 |
| Topology (inner contour present) | Yes | No |

Using two orthogonal cues means a single bad frame cannot flip the
decision.

**Target acquisition without position.** The vehicle has no horizontal
odometry, so "return to where the X bin was" is not an actionable
instruction. Instead the sequencer **inspects rather than remembers**: it
centres over whichever bin is beneath it, reads the symbol, and either
commits or steps over it and continues along the path until the next bin
appears. Rejected symbols are recorded so the lookout cannot walk back
into a bin it has already turned down.

**Servo target switching.** At release altitude the bin outline is 527 px
across in a 480 px frame — it runs off the edge and its centroid becomes
badly biased. The controller therefore tracks the **symbol**, not the bin
outline, falling back to the outline only when the symbol is not
resolvable.

**Visual ground speed.** A released marker inherits the vehicle's
horizontal velocity. Commanding zero velocity is not the same as being
stopped — the hull coasts. With no DVL, the only observable ground speed
comes from how fast the bin slides across the down camera, converted to
m/s through the known altitude:

```
    metres_per_pixel = altitude / focal_length
```

This is down-camera visual odometry, and it is what gates the release.

> **[FIGURE 7]** Screenshot of the down camera during bin approach with
> both bins visible, symbols classified, and the target highlighted.
> *Caption: "Bin classification. Circularity and contour topology
> separate the ring from the cross; the target symbol is highlighted."*

### 6.5 Mission 5 — Torpedo Through the Heart Cut-out

> **Status: designed; arena prepared; not yet implemented.**

The target is a 0.61 m square plate carrying a 0.34 × 0.31 m
heart-shaped cut-out.

**Perception.** The plate is found on colour; the *hole* is found through
**contour hierarchy** — an inner contour nested inside an outer one. This
is precisely the same topological cue already used to identify the ring
bin, and reusing it is a deliberate economy.

**Ranging.** From the known 0.61 m plate width via the pinhole model.

**Control.** Align yaw and depth to place the hole on the optical axis;
gate firing on both alignment and range being within tolerance.

**Ballistics.** This is the one genuinely new element. The projectile
loses height under net weight and decelerates in drag over its flight, so
the aim point must be raised above the hole by a computed amount rather
than placed on it.

### 6.6 Mission 6 — Surfacing Inside the Octagon

> **Status: designed; arena prepared; not yet implemented.**

**The key geometric fact.** Both path branches terminate *exactly at an
octagon centre* — the left branch at (8, −5) and the right at (8, +5).
The octagon has an apothem of 3.26 m against a vehicle half-width of
0.29 m, leaving roughly 3 m of margin. "Surface inside the octagon"
therefore reduces largely to "follow the path until it ends, then
ascend".

**The perception difficulty.** The octagon floats at the surface while
the vehicle cruises 1.5 m below it, and **there is no upward-facing
camera**. The floats are only within the front camera's vertical field of
view beyond about 2.9 m; closer in, the ring rises out of the top of the
frame.

> This is the third time this pattern appears — the L-bar corner leaves
> the frame inside 2.6 m, the bin overflows it at release altitude, and
> the octagon disappears above it. A 60° field of view is marginal for
> this arena, which is a concrete argument for a wide-angle lens behind a
> dome port on the physical vehicle (Section 8.3).

**Layered approach.** Rather than attempting to servo into the octagon at
close range:

1. **Arrive by path.** The path terminates at the octagon centre; a
   persistent loss of the line in the final leg *is* the arrival signal.
2. **Confirm on approach.** Detect the yellow floats at 3–6 m, before the
   cue disappears, to confirm the correct octagon.
3. **Stop and hold** before ascending.
4. **Ascend** by raising the depth setpoint through the existing cascade.
5. **Verify during ascent.** As the vehicle rises, the ring comes level
   with the front camera. A slow yaw scan then answers "am I inside?"
   directly: segments at roughly uniform range in all directions means
   inside; all clustered on one bearing means outside. Containment
   becomes a **bearing-distribution test** rather than a position
   estimate.
6. **Detect the surface** from pressure — unambiguous, requiring no
   interpretation.

---

## 7. Control and State Estimation

### 7.1 Thruster allocation

The four horizontal thrusters are vectored at 45°. Their contributions
were derived from the vehicle geometry rather than assumed:

| Thruster | Surge | Sway | Yaw |
|---|---:|---:|---:|
| T1 front-right | −0.707 | −0.707 | −0.164 |
| T2 front-left | −0.707 | +0.707 | +0.164 |
| T3 rear-right | +0.707 | −0.707 | +0.164 |
| T4 rear-left | +0.707 | +0.707 | −0.164 |

Inverting this gives the mixing law. T5–T8 are vertical and are driven
identically, which — because they are symmetric fore and aft about the
centre of gravity — produces **exactly zero pitch moment**. Combined with
the centre of buoyancy sitting 49 mm above the centre of gravity, the
vehicle is passively stable in pitch and roll and requires no attitude
controller.

When any thruster would saturate, the **entire command vector is scaled
down** rather than clipping one channel. Clipping a single thruster
changes the direction of the resultant force, silently turning a hard
turn into a turn plus an unwanted drift.

### 7.2 Depth control

A PID producing heave thrust. Two details are driven by the vehicle's
physics:

- Drag is **purely quadratic** — there is no linear damping term to lean
  on — so the derivative term damps on *measured rate*, not on
  differentiated setpoint error.
- The integral term exists to trim out the +1.36 N residual buoyancy
  without hard-coding it. In simulation it converges to exactly the
  −0.34 command that this trim requires.

Performance against the vehicle's true vertical dynamics: settles in
approximately 2 s with about 1 cm of overshoot.

### 7.3 Vertical state estimation — theoretical substantiation

This section documents the most substantive analytical result in the
stack.

**The problem.** The depth controller needs vertical *velocity* for its
damping term, but the pressure sensor measures only position. Obtaining
velocity by differencing consecutive readings amplifies noise severely.
For a sensor resolving 2 mm at 20 Hz:

```
    σ_v = σ_z · √2 / Δt = 0.002 × 1.414 / 0.05 = 0.057 m/s
```

Passing this through the derivative gain produces thrust chatter **larger
than the 1.36 N of buoyancy the loop is trimming**. The scaling is
perverse: polling the sensor *faster* makes the naive derivative worse,
not better.

**Why filtering alone is insufficient.** Low-pass filtering the
derivative trades noise against phase lag, and no setting is
simultaneously quiet and responsive.

**The solution.** Fuse the pressure measurement with inertial
acceleration in a two-state Kalman filter over `[z, ż]`. The prediction
step carries the fast dynamics from the accelerometer; the slow,
absolute, drift-free pressure measurement corrects the accelerometer's
bias. Each sensor covers precisely what the other cannot do:

| | Pressure (Bar30) | IMU accelerometer |
|---|---|---|
| Update rate | 20 Hz | 1 kHz |
| Noise | jittery | smooth |
| Long-term | never drifts | drifts (bias integrates) |

The measurement update also corrects **velocity**, not just position,
through the position–velocity correlation term in the covariance. This is
how a clean rate estimate is extracted from a position-only sensor.

**Measured result** (against the simulated vertical plant):

| Configuration | Thrust chatter | Estimator error |
|---|---:|---:|
| Ground truth (unavailable in reality) | 0.00 N | — |
| Pressure, raw backward difference | **3.67 N** | 1.1 cm |
| Pressure, constant-velocity filter | 0.56 N | 1.0 cm |
| **Pressure + IMU fused** | **0.08 N** | **0.4 cm** |

Fusion reduces thrust chatter by a factor of **46** with no change to the
controller gains.

> **[FIGURE 8]** Plot of commanded heave versus time for the three
> configurations in the table above, showing the chatter reduction.
> *Caption: "Vertical state estimation. Naive differencing of the
> pressure sensor produces thrust chatter exceeding the buoyancy trim;
> inertial fusion reduces it by a factor of 46."*

---

## 8. Simulation Constraints and Fidelity

A simulation that flatters the control system is worse than useless,
because gains tuned against it will not transfer. Three deliberate
fidelity decisions were made, and three known gaps are recorded honestly.

### 8.1 Fidelity deliberately added

**Sensor realism.** The simulator provides perfect, instantaneous ground
truth. This was explicitly *not* used for control. Instead, the pressure
sensor is modelled with 30 ms transport latency, 0.3 mbar RMS noise and
0.2 mbar quantisation, applied in the physical order (delay, then noise,
then quantisation, because an ADC digitises an already-noisy analogue
signal). IMU noise and bias are modelled at MTI-630R grade. Without this,
the estimator of Section 7.3 would appear unnecessary.

**Restricted state access.** The entire stack consumes exactly **two
scalars** of ground truth — depth and depth rate — and both are replaced
by the modelled sensor chain. No module uses ground-truth horizontal
position or heading. This is what makes the sim-to-hardware boundary
narrow and testable.

**Colour derived from physics.** Every HSV threshold was derived by
sampling the arena's declared material colours through the water volume's
alpha blend, rather than tuned by hand. For example, the red buoy's hue
migrates from 0 through 174 to 145 as the intervening water deepens,
while the orange path stays pinned at 11–13 — the bands were sized to
separate them across that whole range.

### 8.2 Arena corrections required

The supplied arena required correction before several tasks were
physically meaningful. These are recorded because they materially affect
what can be demonstrated:

| Issue | Correction |
|---|---|
| 30 scoring targets had no collision geometry (buoys, L-bar, torpedo plates, both octagons) — the vehicle passed through them | Collision added to all 30 |
| The two bins were geometrically identical; O/X existed only in link names | White ring and cross symbols added to the bin floors |
| The L-bar had both rods horizontal — an "L" only in plan view | One rod re-oriented vertical |
| The torpedo plates were solid, with no cut-out, and intersected each other | Rebuilt with a true heart-shaped hole; separation corrected |
| The octagons floated 0.2 m *below* the surface | Raised to the waterline |

### 8.3 Known gaps between simulation and reality

These are limitations of the simulation that will require attention
before pool trials, and are stated explicitly rather than discovered
later:

| Gap | Consequence | Mitigation |
|---|---|---|
| **Flat viewport refraction is not modelled.** The simulated camera is an ideal pinhole. A real camera behind a flat port has its effective focal length scaled by ~1.33 | Every monocular range estimate would read **25 % short** — the vehicle would stop 12 cm before reaching a buoy it believed it had reached | Camera intrinsics are read from a `CameraInfo` topic rather than hard-coded, so an underwater calibration drops in without code changes. Calibration must be shot **in the housing, in water** |
| **No optical attenuation model.** The simulated water is a translucent volume; it does not absorb red preferentially with distance | Real HSV values will shift more than modelled, particularly for the red buoy at range | Controlled lighting on the vehicle; re-derive thresholds from pool imagery |
| **No backscatter or ambient light variation** | Threshold stability in a real pool will be worse | Lights mounted wide and angled outward, away from the lens axis |
| **Rolling shutter not modelled** | Angle estimates acquire a turn-rate-dependent bias (0.2–2.1° depending on yaw rate) | Prefer a global-shutter sensor for the forward camera |
| **`dartsim` cannot build collision from mesh geometry** | The heart cut-out's collision is a rectangular frame around the hole, not the heart outline | Physics gives pass/block; precise "through the heart" scoring requires a geometric check against the true outline |

---

## 9. Validation and Results

All results below are from offline simulation of the vehicle's true
dynamics with the modelled sensor chain.

### 9.1 Perception

| Test | Conditions | Result |
|---|---|---|
| Buoy detection | 3 water-tint levels | Centre recovered exactly; radius within 0.8 px |
| Bin symbol classification | 3 altitudes × 3 tint levels | **18/18 correct**; metric position within 2 mm |
| Colour separation | All arena materials sampled through the water blend | Red buoy vs orange path separated at every plausible tint |

### 9.2 Control

| Test | Result |
|---|---|
| Depth acquisition | Settles ≈ 2 s, ≈ 1 cm overshoot, integrator converges to exact buoyancy trim |
| Vertical estimator | Chatter 3.67 N → 0.08 N; error 1.1 cm → 0.4 cm |
| Marker release accuracy | Both markers landed in the target bin, 1.8 cm and 3.3 cm from centre |

### 9.3 Full sequence

The sequencer was exercised end-to-end in an offline harness driving the
true vehicle dynamics with simulated imagery. It completed:

```
INIT → DIVE → LINE_FOLLOW → BUOY_APPROACH → BUOY_TOUCH → BUOY_BACKOFF
     → LINE_FOLLOW → BIN_APPROACH → BIN_REJECT → LINE_FOLLOW
     → BIN_APPROACH → BIN_DESCEND → BIN_HOLD → BIN_DROP
     → BIN_HOLD → BIN_DROP → BIN_DONE → LINE_FOLLOW
```

Note the `BIN_REJECT` transition: the vehicle correctly inspected the
wrong bin, classified it, declined it, stepped over it, and re-acquired
the correct one.

> **[FIGURE 9]** Composite screenshot or short video still sequence
> showing the vehicle at each mission phase.
> *Caption: "Mission sequence executed in simulation."*

---

## 10. Development Status

| Mission | Perception | Control | Arena | Status |
|---|:-:|:-:|:-:|---|
| 1 — Dive and path following | ✔ | ✔ | ✔ | **Implemented, validated** |
| 2 — Buoy touch | ✔ | ✔ | ✔ | **Implemented, validated** |
| 3 — L-bar crossing | ✖ | ✖ | ✔ | Designed; arena ready |
| 4 — Marker dropping | ✔ | ✔ | ✔ | **Implemented, validated** |
| 5 — Torpedo | ✖ | ✖ | ✔ | Designed; arena ready |
| 6 — Octagon surfacing | ✖ | ✖ | ✔ | Designed; arena ready |

**Honest statement of maturity.** Three of six missions are implemented
and validated offline against the vehicle's true dynamics. The remaining
three are designed in detail, and the arena has been corrected so that
each is physically realisable, but they are not yet coded. The
architecture is deliberately structured so that each remaining mission
requires one perception module and a small number of FSM states, reusing
the existing depth cascade and path follower without modification.

Full end-to-end execution inside the Gazebo GUI, as opposed to the
offline harness, remains to be demonstrated.

---

## 11. Forward Work

**Before the Critical Design Review:**

1. Implement L-bar crossing, torpedo firing and octagon surfacing.
2. Demonstrate the complete mission end-to-end in the live simulator and
   record the video simulation deliverable.
3. Add a route-selection state with an explicit time budget, so the
   vehicle attempts both branches only when time permits.
4. Add a fault state: leak and voltage-sag detection must pre-empt every
   other state and drive the vehicle to the surface.

**For the physical vehicle:**

5. Perform underwater camera calibration in the final housing, and
   publish the result through the existing `CameraInfo` interface.
6. Re-derive HSV thresholds from pool imagery under the vehicle's own
   lighting.
7. Replace the simulated pressure model with the real driver and re-tune
   the estimator against measured noise.

---

## 12. Summary

The software stack implements a complete autonomy pipeline for the SAVe
mission: perception from two cameras, fused vertical state estimation,
a cascaded control architecture, and a finite state machine sequencing
six mission tasks — in approximately 3,300 lines across eight modules.

Three design decisions define it:

1. **Separation of axes.** One behaviour owns the horizontal plane at a
   time; the depth loop runs beneath all of them, because a positively
   buoyant vehicle cannot hand depth between controllers.
2. **Image space over world space.** With no absolute horizontal
   reference available, every horizontal decision is made from what the
   cameras see now. The vehicle inspects rather than remembers.
3. **An honest simulation.** Sensor noise, latency and quantisation are
   modelled deliberately, and ground-truth access is restricted to two
   scalars, so that gains tuned in simulation have a realistic prospect
   of transferring to hardware.

> **[FIGURE 10]** Final render of the vehicle in the arena, ideally
> mid-mission.
> *Caption: "The vehicle executing the path-following phase."*
