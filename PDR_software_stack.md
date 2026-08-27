# Software and Autonomy Subsystem — PDR Contribution

**National Student Autonomous underwater Vehicle Competition (SAVe)**

> *Drop-in content for the official PDR format. This covers only the
> software architecture and control strategy bullets of §5 Theoretical
> Design, plus a contribution to §6 Experimental Results. Mechanical,
> electrical, sensors and power are written by their owners.*
>
> *Editing notes appear in blockquotes and should be deleted before
> submission. Figure placeholders are marked **[FIGURE n]**.*

---

## For §5 — Theoretical Design: Software Architecture

### Design constraint

The vehicle has no absolute horizontal position reference. A DVL
(Water Linked A50) measures velocity over the seabed, but velocity is not
position — integrating it drifts, and a magnetometer is unreliable
indoors near steel structure and the vehicle's own thrusters. A velocity
scale error of just 2%, integrated over the ~20 m course, accumulates to
a position error larger than a bin's half-width. Vision therefore remains
the primary reference for every task.

This produces the architecture's defining property: **the vehicle
inspects rather than remembers.** It never plans a path in world
coordinates for a task it can see. It centres over whatever target is
beneath it, reads it, and acts — because "return to where the X bin was"
is not a sentence it can act upon.

The one place position aiding earns its place is the short (~2 m) gaps
between tasks where nothing is visible at all. There, DVL-fed velocity
from ArduSub's EKF3 carries the vehicle across the gap, handing back to
vision the instant a target or path segment reappears.

### Structure

The stack is approximately 4,100 lines across nine ROS 2 modules.

| Module | Lines | Responsibility |
|---|---:|---|
| `line_buoy.py` | 1581 | Mission sequencer — 17-state finite state machine |
| `buoy.py` | 573 | Multi-colour buoy detection, ranking, visual servoing |
| `bluerov2_native_bridge.py` | 550 | Gazebo ↔ ROS 2 bridge; cameras, IMU, simulated Bar30, odometry |
| `line_follow.py` | 448 | Orange path detection, cross-track control, fork selection |
| `marker.py` | 396 | Bin detection, O/X classification, station-keeping, visual ground speed |
| `thruster_mixer.py` | 197 | Body-frame commands → 8 thrusters |
| `octagon.py` | 196 | Surfacing-ring detection and containment test |
| `depth_filter.py` | 145 | Pressure + inertial fusion for vertical state |
| `depth_control.py` | 45 | Depth PID |

### Actuation interface

Every module above produces body-frame velocity and depth-setpoint
commands, not raw thrust — the perception and mission logic are
autopilot-agnostic. Results in this report were generated against a
direct Gazebo interface (`bluerov2_native_bridge.py`, `thruster_mixer.py`)
built to validate that logic at development speed. **The target autopilot
is ArduSub, running on a Navigator flight-controller HAT, over MAVLink**;
production thruster mixing and stabilisation belong to ArduSub.
Integrating the mission stack against ArduSub SITL is the next milestone,
tracked for CDR.

### Separation of axes

One structural decision underpins the whole design:

> **Exactly one behaviour commands the horizontal axes at any instant,
> but the depth loop runs continuously beneath every state.**

The reason is physical. The hull displaces 13.14 kg of water against a
13.0 kg mass, leaving it **+1.36 N positively buoyant** — it floats to
the surface the moment nothing holds it down. Depth can therefore never
be handed between behaviours the way steering can.

Behaviours influence depth by *moving the setpoint*, never by commanding
thrust. The vertical axis is a cascade:

```
pixel error → commanded rate (m/s) → depth setpoint → PID → thrust
  outer loop         integrator        inner loop
```

Keeping the buoyancy-trim integrator in exactly one place is what stops
two controllers fighting for the same axis.

> **[FIGURE 1]** Block diagram: perception modules feeding the sequencer,
> sequencer arbitrating surge/sway/yaw, depth loop running underneath all
> states into the thruster mixer.

### Mission sequencer

A 17-state finite state machine, driven by a fixed 20 Hz timer working on
the most recent frame from each camera rather than inside image
callbacks. This decouples the control period from camera jitter and gives
every derivative and integrator a constant `dt`.

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

> **[FIGURE 2]** Redraw the above as a proper state diagram; solid arrows
> nominal, dashed arrows recovery paths.

### Perception approach

Classical computer vision — colour segmentation and contour analysis —
rather than a learned detector. The targets are of known size, shape and
colour; a classical pipeline is deterministic, needs no training data,
runs on embedded hardware, and can be justified analytically.

Two techniques recur across tasks:

**Minimum enclosing circle for spheres.** The buoys are spheres, so a
minimum enclosing circle fits far better than a bounding box: it yields
centre and radius in one operation, and the ratio of contour area to
circle area gives a scale-free *circularity* measure. Circularity alone
separates a sphere (~0.97) from the flat square torpedo target (~0.38)
at any range.

**Contour topology.** A ring has an inner contour; a cross does not. This
distinguishes the O and X bins, and is the same cue the heart cut-out
will use.

**Thresholds are measured, not modelled.** HSV bounds are taken from
rendered frames rather than derived from material colours. This matters:
the rendered scene is 2–2.5× darker than an alpha blend of the SDF
material predicts, because the arena has one directional light and the
floor lies under a metre of translucent water.

| Surface | Predicted from material | **Measured in frame** |
|---|---|---|
| White bin symbol | (106, 47, 226) | **(0, 0, 97)** |
| Navy bin | (111, 188, 137) | (112, 131, 68) |
| Grey floor | (108, 70, 168) | (120, 6, 80) |

### Image-based visual servoing

Approach behaviours close their loop on pixel error directly, with no
intermediate pose estimate. Errors are normalised by half the image size
so gains are resolution-independent:

```
e_x = (u − u₀)/u₀        e_y = (v − v₀)/v₀
```

With a single forward camera, horizontal pixel error *is* an angle, so
yaw is the axis that observes it; `e_y` is cascaded into the depth loop.
Surge is scaled inversely with apparent radius, so the vehicle
decelerates automatically as it closes.

Camera frame conventions are stated explicitly, since sign errors here
are the most common failure in a visual servoing system. Body frame is
FLU (x forward, y port, z up):

| Camera | Image right | Image down/up |
|---|---|---|
| Front (unrotated) | starboard (−y) | down (−z) |
| Down (pitched 90°) | starboard (−y) | image up = forward (+x) |

The down-camera loop contains **no yaw term at all** — both image axes
map onto translation, making bin work a station-keeping problem rather
than a pointing one.

---

## For §5 — Theoretical Design: Control Strategy

### Thruster allocation

Four horizontal thrusters vectored at 45°, four vertical. Contributions
derived from vehicle geometry:

| Thruster | Surge | Sway | Yaw |
|---|---:|---:|---:|
| T1 front-right | −0.707 | −0.707 | −0.164 |
| T2 front-left | −0.707 | +0.707 | +0.164 |
| T3 rear-right | +0.707 | −0.707 | +0.164 |
| T4 rear-left | +0.707 | +0.707 | −0.164 |

T5–T8 are vertical and driven identically. Because they are symmetric
fore and aft about the centre of gravity, heave produces **exactly zero
pitch moment** by construction. With the centre of buoyancy 49 mm above
the centre of mass, the vehicle is passively stable in pitch and roll and
needs no attitude controller.

On saturation the **entire command vector is scaled down** rather than
clipping one channel — clipping changes the direction of the resultant
force, silently turning a hard turn into a turn plus drift.

### Vertical state estimation

This is the most substantive analytical result in the stack, and it
addresses a problem that only appears once the sensor is modelled
realistically.

**The problem.** The depth controller needs vertical *velocity* for
damping, but the pressure sensor measures only position. Differencing
consecutive readings amplifies noise severely — for a sensor resolving
2 mm at 20 Hz:

```
σ_v = σ_z·√2/Δt = 0.002 × 1.414 / 0.05 = 0.057 m/s
```

Through the derivative gain this produces thrust chatter **larger than
the 1.36 N of buoyancy the loop is trimming**. The scaling is perverse:
polling the sensor *faster* makes the naive derivative worse.

**Why filtering is insufficient.** Low-pass filtering trades noise
against phase lag; no setting is both quiet and responsive.

**The solution.** Fuse pressure with inertial acceleration in a two-state
Kalman filter over `[z, ż]`. The prediction step carries fast dynamics
from the accelerometer; the slow, absolute, drift-free pressure
measurement corrects its bias. Each sensor covers what the other cannot:

| | Pressure (Bar30) | IMU accelerometer |
|---|---|---|
| Rate | 20 Hz | 1 kHz |
| Noise | jittery | smooth |
| Long term | never drifts | drifts (bias integrates) |

The measurement update also corrects **velocity**, through the
position–velocity correlation term in the covariance — which is how a
clean rate estimate is extracted from a position-only sensor.

**Measured against the vehicle's vertical dynamics:**

| Configuration | Thrust chatter | Estimator error |
|---|---:|---:|
| Pressure, raw backward difference | **3.67 N** | 1.1 cm |
| Pressure, constant-velocity filter | 0.56 N | 1.0 cm |
| **Pressure + IMU fused** | **0.08 N** | **0.4 cm** |

Fusion reduces chatter by a factor of **46** with no change to controller
gains.

> **[FIGURE 3]** Commanded heave versus time for the three configurations,
> showing the chatter reduction.

### Ground speed and station-keeping

Marker release requires the vehicle to be genuinely stopped — a released
marker inherits the vehicle's horizontal velocity, and commanding zero is
not the same as being stopped, because the hull coasts.

The production source is the DVL, feeding ArduSub's EKF3 and delivered to
the mission stack over MAVLink once ArduSub SITL integration is complete
(see Actuation interface).

The Gazebo-only development testbed — which bypasses ArduSub entirely —
validated the same requirement against a software-only estimate: how fast
a tracked target slides across the down camera, converted to metres
through the known altitude:

```
metres_per_pixel = altitude / focal_length
```

This down-camera estimate gated every marker release tested in
simulation, and remains a useful independent cross-check once the DVL is
integrated — it fails by a different mechanism than an acoustic Doppler
return, so the two are not vulnerable to the same fault at once.

---

## For §6 — Experimental Results

### Simulation environment

| Item | Selection |
|---|---|
| Simulator | Gazebo Sim 8 (Harmonic), `dartsim` physics, 1 ms step |
| Middleware | ROS 2 Humble |
| Vehicle | BlueROV2, 8 thrusters, 13.0 kg |
| Arena | 25 × 20 m tank, 2.5 m water column, full competition course |

**Sensor realism was added deliberately.** The simulator provides
perfect, instantaneous ground truth, which flatters any controller tuned
against it. Instead the pressure sensor is modelled with 30 ms latency,
0.3 mbar RMS noise and 0.2 mbar quantisation, applied in physical order
(delay, then noise, then quantisation). IMU noise and bias are modelled
at MTI-630R grade.

The entire stack consumes exactly **two scalars** of ground truth —
depth and depth rate — and both are replaced by the modelled sensor
chain. No module uses ground-truth horizontal position or heading. This
is what keeps the sim-to-hardware boundary narrow and testable.

> **[FIGURE 4]** Screenshot of the arena in Gazebo, showing the orange
> path, flowers, L-bar, bins, torpedo target and both octagons.

### Full-mission result

The complete sequence has been executed end-to-end in simulation:

```
DIVE → LINE_FOLLOW → BUOY_APPROACH → BUOY_TOUCH → BUOY_BACKOFF
     → FORK (route selected) → BIN_APPROACH → BIN_DESCEND
     → BIN_HOLD → BIN_DROP ⇄ BIN_HOLD → BIN_DROP → BIN_DONE
     → OCTAGON_ARRIVE → OCTAGON_HOLD → OCTAGON_ASCEND → SURFACED
```

> Executed against an earlier reconstruction of the arena. The arena has
> since been corrected to match the documented layout — the orange path
> is discontinuous rather than continuous — so the fork- and
> line-loss-based transitions above are being reworked; end-to-end
> re-validation against the corrected arena is the immediate next test.
> Per-task detection and control (buoy, bin, depth estimation,
> containment) do not depend on path continuity and remain valid.

Representative run:

| Event | Result |
|---|---|
| Buoy touched | red flower, committed at 0.89 m |
| Route selected | 2 legs detected at fork, starboard taken |
| Marker 1 released | 5.6 cm from bin centre, 6.2 cm/s ground speed |
| Marker 2 released | 4.0 cm from bin centre, 5.2 cm/s ground speed |
| Surfaced | 0.12 m depth, 2.99 m from octagon centre |
| Containment confirmed | 35/36 frames saw the ring surrounding |

Course covered: x from −11.00 m to 9.06 m. The octagon apothem is 3.26 m
against a vehicle half-width of 0.29 m, so surfacing 2.99 m from centre
is inside the structure.

> **[FIGURE 5]** Composite of debug camera views at each mission phase.

### Component results

| Test | Conditions | Result |
|---|---|---|
| Buoy detection | 3 water-tint levels | Centre exact; radius within 0.8 px |
| Bin symbol classification | 3 altitudes × 3 tints | 18/18 correct; position within 2 mm |
| Depth acquisition | vs. true vertical dynamics | Settles ≈ 2 s, ≈ 1 cm overshoot |
| Vertical estimator | modelled sensor chain | Chatter 3.67 N → 0.08 N |
| Octagon containment | inside vs. 4 m outside | 600 px vs 98 px densest row — 5.8× separation |

### Development status

| Mission | Status |
|---|---|
| 1 — Dive and path following | **Demonstrated in simulation** |
| 2 — Buoy touch | **Demonstrated in simulation** |
| 3 — L-bar crossing | Designed; cleared incidentally by 0.5 m |
| 4 — Marker dropping | **Demonstrated in simulation** |
| 5 — Torpedo through cut-out | Designed; arena target built |
| 6 — Surfacing in octagon | **Demonstrated in simulation** |

Four of six mission tasks are implemented and demonstrated end-to-end.
The remaining two are designed in detail and the arena has been prepared
so each is physically realisable; they are scheduled for implementation
before the Critical Design Review. The architecture is structured so each
requires one perception module and a small number of FSM states, reusing
the existing depth cascade and path follower unchanged.

### Known limitations

Recorded explicitly, as they set the agenda for hardware trials:

| Limitation | Consequence | Mitigation |
|---|---|---|
| Flat viewport refraction not modelled | A real camera behind a flat port has effective focal length scaled by ~1.33, making every monocular range read **25 % short** | Intrinsics are read from a `CameraInfo` topic, not hard-coded, so an underwater calibration drops in with no code change. Calibration must be shot in the housing, in water |
| No optical attenuation model | Real HSV values will shift more than modelled, especially red at range | Controlled vehicle lighting; re-derive thresholds from pool imagery |
| Rolling shutter not modelled | Angle estimates acquire a turn-rate-dependent bias (0.2–2.1°) | Prefer a global-shutter forward camera |
| `dartsim` cannot build mesh collision | The heart cut-out's collision is a rectangular frame, not the heart outline | Physics gives pass/block; precise scoring needs a geometric check |

---

## Forward work to CDR

1. Integrate the mission stack against ArduSub SITL over MAVLink, so
   simulation matches the real actuation and stabilisation path — this is
   also what brings live DVL-fed velocity into the mission stack.
2. Build and validate DVL-aided transit across the blind gaps between
   tasks, against the corrected, discontinuous-path arena.
3. Implement L-bar crossing and torpedo firing.
4. Enable multi-buoy touching (machinery present; currently set to one).
5. Add a fault state — leak and voltage-sag detection pre-empting all
   states and driving the vehicle to the surface.
6. Underwater camera calibration in the final housing.
7. Re-derive HSV thresholds from pool imagery under vehicle lighting.
