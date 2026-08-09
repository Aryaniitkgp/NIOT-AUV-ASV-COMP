# Electronics and Power Subsystem — PDR Content

**NIOT Student Autonomous Underwater Vehicle Competition (SAVe)**
Sections 6 and 7 of the Preliminary Design Report

> Editing notes appear in blockquotes and should be removed before
> submission. Figure placeholders are marked **[FIGURE n]** with a
> description of what to draw and a suggested caption.

---

## 6. Electronics System

### 6.1 Overview and Processing Layers

The electronics architecture is organised around one principle: **keep
high-current propulsion paths electrically and physically separate from
low-current logic.** Brushless thrusters draw tens of amps and switch
that current at kilohertz rates. The resulting conducted noise and
radiated EMI are the dominant cause of processor brownouts and sensor
corruption in small underwater vehicles.

The system is therefore split into three processing layers, each with a
distinct time budget:

| Layer | Hardware | Period | Responsibility |
|---|---|---|---|
| Perception and autonomy | Raspberry Pi 5 | 50 ms (20 Hz) | Image processing, state estimation, mission logic, control laws |
| Real-time I/O | Blue Robotics Navigator | ~1 ms | Sensor aggregation, hardware PWM generation |
| Actuation | 8× ESC | continuous | DC to 3-phase commutation |

**Why the split matters.** The Pi runs a general-purpose Linux kernel
and cannot guarantee microsecond-accurate pulse timing; a scheduling
delay of even 1 ms would visibly jitter thruster output. The Navigator's
dedicated PWM hardware generates pulses independently of operating
system load. Conversely, the Navigator's microcontroller cannot process
two camera streams. Each layer does what its hardware is suited to.

> **[FIGURE 6.1]** Three-layer block diagram. Top box: Raspberry Pi 5
> (perception, estimation, mission, control). Middle: Navigator (sensor
> aggregation, PWM). Bottom: ESC bank and thrusters. Annotate the
> downward arrow "PWM channels 1–10" and the upward arrow "I²C sensor
> telemetry via 40-pin GPIO".
> *Caption: "Three-layer processing architecture. Each layer operates at
> the timescale its hardware can guarantee."*

---

### 6.2 Components List

| Component | Model | Function |
|---|---|---|
| Main computer | Raspberry Pi 5 (8 GB) | ROS 2 perception, state estimation, mission FSM, control, logging |
| Flight controller | Blue Robotics Navigator | Sensor aggregation, hardware PWM generation |
| IMU | XSENS MTi-630R | 6-DOF inertial data for vertical state estimation |
| Forward camera | Global-shutter colour (IMX296 / AR0234 class) | Buoy, obstacle and torpedo-target detection |
| Downward camera | Raspberry Pi Camera Module 3 Wide | Path tracking, bin detection, visual ground speed |
| ESC | 8× Blue Robotics Basic ESC | Battery-to-thruster power interface |
| Thrusters | 8× Blue Robotics T200 | 4 horizontal (vectored 45°), 4 vertical (corners) |
| Battery | 2× 4S LiPo | Main power supply |
| Pressure sensor | Bar30 (MS5837) | Depth measurement |
| Underwater lights | 2× Blue Robotics Lumen R2 | Controlled illumination for colour segmentation |
| Marker dropper servo | DSServo DS3225 | Rotary release, one marker per actuation |
| Torpedo firing servo | DSServo DS3235 | Latch release for spring-loaded launcher |
| Power monitors | 2× INA219 | Per-pack voltage and current |
| Storage | NVMe SSD via M.2 HAT | Boot and mission logging |

#### Explaining the table

The list is best read as four groups, each answering a different
question.

**Compute (rows 1–2)** — *what makes decisions.* The Pi 5 was selected
over the Pi 4B because the perception stack processes two camera streams
at 20 Hz while simultaneously running state estimation and control. The
Pi 5 is roughly 2–3× faster in the same form factor and retains
Navigator compatibility. Storage is NVMe rather than microSD: card
corruption after an abrupt power cut is among the most common failure
modes in vehicles that are killed mid-write, and a corrupted card means
losing both the vehicle's software and the log that would explain the
failure.

**Perception (rows 3–5, 9, 10)** — *what the vehicle knows.* Two
cameras, one pressure sensor, one IMU, and lights. Note what is absent:
there is no DVL and no acoustic positioning, so **the vehicle has no
absolute horizontal position reference.** This single fact drives the
entire autonomy architecture and is discussed in the software section.

**Actuation (rows 6–7, 11–12)** — *what the vehicle does.* Eight
thrusters through eight ESCs, plus two payload servos. Independent ESCs
per thruster allow the allocation matrix to command each unit
separately, which is what makes vectored control possible.

**Power and monitoring (rows 8, 13)** — *what keeps it alive and
watches it.* Two packs and two current monitors, one per pack, so a
fault in either is independently detectable.

#### Component selection rationale

Three selections differ from a conventional BlueROV2-derived build, and
each is a considered engineering decision rather than a substitution of
convenience.

**No optical flow sensor.** A PMW3901-class optical flow module is a
common choice for bounding inertial drift. It was evaluated and
rejected for three reasons:

1. *Fixed focus.* The sensor is optimised for a narrow altitude band. The
   vehicle operates from roughly 0.65 m to 1.5 m above the floor and
   descends further for marker release, moving outside that band during
   exactly the manoeuvre where velocity measurement matters most.
2. *Texture dependence.* The sensor correlates frame-to-frame pixel
   patterns. A tiled pool floor under a metre of water offers low
   contrast and little high-frequency detail.
3. *Silent degradation.* This is the decisive objection. The module
   outputs a velocity value with no confidence metric. A failed lock
   produces a plausible-looking number — typically near zero — which is
   indistinguishable from the vehicle genuinely being stopped. A control
   loop cannot defend against a sensor that lies confidently.

Horizontal velocity is instead derived from the downward camera, which
is already fitted, has autofocus, and produces an image that can be
logged and inspected after a failure. This also removes one hull
penetration.

**Forward camera: global shutter, not a depth camera.** A stereo depth
camera such as the Intel RealSense D435i derives range from an 850 nm
structured-light projector. Water absorbs 850 nm at approximately
4 m⁻¹, and the light must travel to the target and back:

| Target range | IR surviving the round trip |
|---|---|
| 0.10 m | 42 % |
| 0.25 m | 12 % |
| 0.50 m | 1.4 % |
| 1.00 m | 0.02 % |

Beyond half a metre the projected pattern is effectively gone, leaving
passive stereo against a low-texture scene. The depth stream cannot be
relied upon. Range is instead obtained geometrically from known target
dimensions.

Global shutter is preferred over rolling shutter because the vehicle's
two most important measurements — orange-path heading and L-bar
orientation — are *angles*. A rolling-shutter sensor reads out row by
row over 10–30 ms; if the vehicle is yawing during readout, straight
lines are recorded skewed. The resulting angle error scales with turn
rate (0.2–2.1° across realistic rates), making it a velocity-dependent
bias rather than random noise, which is the kind of error that can drive
a control loop into oscillation.

**Underwater lights.** The perception stack is built on HSV colour
segmentation. Ambient pool illumination varies with time of day,
surface disturbance and depth, and water attenuates red wavelengths
first — a red target appears brown-grey at a few metres. Controlled
on-board lighting, with white balance and exposure locked once the
lights dominate ambient, is what makes colour thresholds repeatable
between runs. This is the single highest-value addition for perception
reliability.

---

### 6.3 Component Placement

| Component | Position | Reason |
|---|---|---|
| Main computer | Electronics enclosure | Thermal path to hull wall |
| Flight controller | Electronics enclosure, stacked under Pi | Eliminates external signal wiring |
| IMU | Electronics enclosure, near centre of mass | Minimises lever-arm acceleration artefacts |
| Pressure sensor | Electronics enclosure, rear end cap | Away from thruster wash |
| Forward camera | Front, behind dome port | See dome port note below |
| Downward camera | Bottom exterior | Unobstructed view of floor |
| ESC ×8 | Electronics enclosure, isolated tray | Separates switching noise from logic |
| Lights ×2 | Outer frame rails, angled outboard | Backscatter mitigation |
| Battery ×2 | Battery enclosure | Thermal and failure isolation |

#### Explaining the placement decisions

**The dual-enclosure split** is the most consequential choice. Batteries
generate heat under load and are the highest-severity failure item in
the vehicle; isolating them in a separate vented pod means a cell fault
does not vent into the electronics, and packs can be swapped poolside
without opening the main chamber.

**Dome port on the forward camera.** A flat viewport refracts light at
the water-to-air boundary — the same effect that makes a straw appear
bent in a glass. Two consequences follow, both significant:

1. Effective focal length is scaled by the refractive index ratio
   (≈1.33), so every monocular range estimate reads about 25 % short
   unless the camera is recalibrated underwater.
2. Field of view narrows by roughly the same factor. An 80° lens in air
   becomes about 60° in water.

The second consequence matters more than it appears. Several targets
leave the field of view at close range — analysis of the arena geometry
shows the L-bar corner is only within frame beyond 2.6 m, and the
octagon ring rises out of frame inside about 2.9 m. Narrowing the field
of view worsens an already-marginal situation.

A dome port admits light perpendicular to its surface at every point,
so no refraction occurs and both effects vanish. **The camera's entrance
pupil must be positioned at the dome's centre of curvature.** Mounted
off-centre, a dome ceases to act as a window and behaves as a lens,
introducing spherical aberration — worse than the flat port it replaced.
This is the most commonly missed detail in dome installations.

**Lights mounted wide and angled outboard**, never adjacent to the lens.
A light beside the optical axis illuminates suspended particles directly
in front of the camera, and that near-field scatter dominates the
returned signal — the same reason headlights are counterproductive in
fog. Separating the light sources from the optical axis means their
beams intersect the target without illuminating the water column the
camera is looking through.

**Pressure sensor away from thruster wash.** The Bar30 measures static
pressure; propeller wash produces local dynamic pressure fluctuations
that appear directly as depth noise, on top of the sensor's own noise
floor.

> **[FIGURE 6.2]** Cutaway or exploded view of the two enclosures with
> components labelled in place. Mark the dome port, the two light
> positions on the frame rails, and the bottom-exterior camera.
> *Caption: "Component placement. Batteries are isolated in the lower
> pod; lights are mounted outboard of the optical axis to avoid
> backscatter."*

---

### 6.4 Connection Analysis

The system divides into four functional domains. This partitioning
addresses the three dominant failure modes in marine robotics:
electrical noise, data processing bottlenecks, and control latency.

#### 1. Power distribution and electrical isolation

Two 4S LiPo packs connect through XT60 connectors to a Power
Distribution Board. Delivery splits into two independent paths:

**Propulsion rail** — raw 4S delivered directly to the eight ESCs, which
commutate DC into 3-phase drive for the brushless thrusters.

**Logic rail** — raw 4S into a 5 V / 6 A Battery Eliminator Circuit,
producing a regulated rail that powers the Navigator, which shares it
with the Pi through the stacking connector.

The BEC is the electrical barrier. Thrusters produce inductive voltage
transients and back-EMF on the propulsion rail; without a dedicated
regulator these would reach the processors as brownouts.

#### 2. High-level processing and perception (Raspberry Pi 5)

The Pi hosts the ROS 2 graph: both camera pipelines, the vertical state
estimator, the mission state machine, and all control loops.

| Sensor | Interface | Reason for choice |
|---|---|---|
| Forward camera | USB 3.0 | Bandwidth for uncompressed colour frames without added latency |
| Downward camera | 15-pin CSI ribbon | Bypasses USB; zero-copy frame access to the image signal processor |
| IMU | USB 2.0 serial | Isolated, collision-free path for high-rate inertial data |

The downward camera uses CSI specifically because it feeds the
highest-rate vision loop in the system — path following runs every
control cycle — and CSI avoids USB bus contention with the forward
camera.

#### 3. Low-level sensor aggregation (Navigator)

The Navigator stacks beneath the Pi on the 40-pin GPIO header:

| Sensor | Bus | Rate |
|---|---|---|
| Bar30 pressure | I²C | 20 Hz |
| INA219 ×2 | I²C | Low rate |

Both are low-bandwidth telemetry streams, well matched to a two-wire
bus. They are passed up the GPIO header into the Pi for state
estimation.

#### 4. Closed-loop actuation and payload control

The Pi's control loops publish body-frame commands — surge, sway, yaw
and heave. A thruster mixer node converts these into eight individual
thrust demands, which cross the stacking connection to the Navigator:

| Channel | Destination | Function |
|---|---|---|
| PWM 1–8 | ESC bank | Individual thruster commands |
| PWM 9 | DS3225 servo | Marker release |
| PWM 10 | DS3235 servo | Torpedo launcher latch release |

> **Terminology note for the mechanical team:** Channel 10 actuates the
> torpedo *release* mechanism. Earlier revisions described this as a
> "reclamation mechanism", which implies retrieval rather than firing
> and is inconsistent with the launcher description in Section 4.4.

---

### 6.5 Connection Diagram

> **[FIGURE 6.3]** Redraw the following as a proper connection diagram.
> *Caption: "System interconnect. Note the two-path power split at the
> PDB — the propulsion rail never touches logic."*

```
                      ┌──────────────────┐
                      │  2× 4S LiPo      │
                      └────────┬─────────┘
                          XT60 │ high current
                      ┌────────▼─────────┐
                      │ Power Distribution│
                      │      Board        │
                      └──┬────────────┬───┘
                  raw 4S │            │ raw 4S
              ┌──────────▼──┐   ┌─────▼────────┐
              │ 5V/6A BEC   │   │  ESC bank ×8 │
              └──────┬──────┘   └─────┬────────┘
                     │ 5 V            │ 3-phase
              ┌──────▼──────┐   ┌─────▼────────┐
              │  Navigator  │   │ Thrusters ×8 │
              └──┬───┬───┬──┘   └──────────────┘
        GPIO 40p │   │   │ PWM 9,10
              ┌──▼───┴┐  └──────► Payload servos
              │ Pi 5  │
              └─┬──┬──┘
        USB 3.0 │  │ CSI
        ┌───────▼┐ ├─────────┐
        │ Forward│ │ Downward│
        │ camera │ │ camera  │
        └────────┘ └─────────┘
              │
        USB 2.0 └──► IMU

        I²C to Navigator: Bar30, INA219 ×2
```

#### Interface rationale

| Interface | Justification |
|---|---|
| USB 3.0 — forward camera | 5 Gbps pipeline; streams uncompressed colour without frame drops |
| CSI ribbon — downward camera | Direct to image signal processor; no USB contention, zero-copy access |
| USB 2.0 serial — IMU | Isolated path; inertial data feeds the estimator prediction step |
| GPIO stacking — Pi to Navigator | No external signal wires; minimises EMI exposure and latency |
| I²C — Bar30, INA219 | Low-overhead two-wire bus, appropriate for low-rate telemetry |
| PWM 1–10 | Hardware-timed; isolates actuator timing from OS scheduling |
| 3-phase from ESC | Required commutation for reversible brushless drive |
| XT60 → PDB → BEC | Dedicated regulator forming a barrier against back-EMF |

---

## 7. Power System

### 7.1 Power Requirements

| Component | Source | V | I (A) | Limit | P (W) |
|---|---|---:|---:|---|---:|
| Thrusters ×8 | Raw battery | 16 | 6.25 each | 25 A each | 800 |
| Lights ×2 | Raw battery | 16 | 1.5 each | 3 A each | 48 |
| Marker dropper | AUX 5 V | 5 | 2 | 3 A | 10 |
| Torpedo launcher | AUX 6 V | 6 | 3 | 5 A | 18 |
| Raspberry Pi 5 | Pi power plane | 5 | 4 | 5 A | 20 |
| Navigator | 5 V/6 A BEC | 5 | 0.5 | 6 A | 2.5 |
| Forward camera | Pi USB 3.0 | 5 | 0.7 | 700 mA | 3.5 |
| Downward camera | Pi CSI | 3.3 | 0.25 | 350 mA | 0.82 |

#### Explaining the table

Read the Power column: **thrusters dominate at 800 W, roughly 90 % of
peak system draw.** Everything else combined is under 105 W. This is the
justification for the dual-pack architecture — sizing a single pack for
peak thruster current would mean the logic rail shares a supply whose
voltage swings tens of volts under load.

The 800 W figure is a *peak* number assuming all eight thrusters at
6.25 A simultaneously. In practice the allocation matrix rarely
saturates every channel; horizontal and vertical clusters are seldom at
full demand together. Sustained mission draw is closer to 35 A total,
which is the figure used for endurance.

The **Limit** column is the protective ceiling, not the operating
point. Each ESC is rated to 25 A, giving roughly 4× headroom over the
6.25 A nominal — sized for the transient inrush when a thruster reverses
direction, which briefly draws far more than steady-state.

Two entries changed from the previous revision: the Pi 5 draws 20 W
against the Pi 4B's 15 W, and the two lights add 48 W. Both are within
budget.

---

### 7.2 Main Battery Selection

A 4S Lithium Polymer configuration (14.8 V nominal, 16.8 V fully
charged) was selected for four reasons:

**Thruster voltage matching.** 14.8 V nominal sits on the T200's
efficiency curve, allowing full rated thrust without exceeding thermal
limits.

**Current and heat reduction.** For a given power, higher rail voltage
means lower current. Since resistive heating scales with the square of
current, this materially reduces heat inside a sealed hull with no
convective path to ambient — and avoids ESC thermal throttling
mid-mission.

**Regulator headroom.** A 4S pack remains above the 5 V BEC dropout even
at low state of charge (≈13.2 V). This is what prevents a thruster
current surge from browning out the Pi at the end of a run.

**Energy density against mass budget.** High C-rating chemistry supplies
the burst currents needed for multi-axis manoeuvres while keeping total
mass within the buoyancy budget. The vehicle is trimmed slightly
positively buoyant, and battery mass is the largest single contributor
to that trim.

#### System integration

**Dual-pack architecture** isolates high-current thruster draw from
logic, suppressing EMI and voltage transients at the source rather than
filtering them downstream.

**Mission endurance** — at a typical 35 A continuous multi-axis draw,
12000 mAh gives approximately 20 minutes. The full course is expected to
take well under this, leaving margin for repositioning and retries.

**Field maintenance** — packs are in a dedicated isolated pod, allowing
poolside swaps without opening the electronics chamber. This matters for
competition turnaround between runs.

---

### 7.3 Battery Specifications

| Parameter | Specification |
|---|---|
| Chemistry | Lithium Polymer (LiPo) |
| Configuration | 4S |
| Nominal voltage | 14.8 V |
| Fully charged | 16.8 V |
| Capacity | 12000 mAh total (2 packs) |
| C-rating | High-C (15–20 C) thruster pack; low-C logic pack |
| Mass | 5 kg |
| Endurance | ~20 min at 35 A |

#### Explaining the asymmetric C-rating

The two packs are deliberately **not** identical. C-rating expresses how
much current a pack can deliver relative to its capacity.

The **thruster pack** must supply burst currents during aggressive
manoeuvres, so it needs a high C-rating (15–20 C). The **logic pack**
supplies a steady few amps to the Pi and Navigator and never sees a
transient — a high C-rating there would add cost and mass for capability
that is never used. A lower-C pack with steadier output voltage is
better matched to a regulator input.

---

### 7.4 Power Distribution Architecture

> **[FIGURE 7.1]** Redraw as a two-column block diagram.
> *Caption: "Dual-chain power distribution. The kill switch acts on the
> thruster chain only, so the vehicle loses propulsion but retains
> compute and logging."*

```
   LOGIC CHAIN                        THRUSTER CHAIN
   Low-C 4S pack                      High-C 4S pack
        │                                   │
      FUSE                                FUSE
        │                                   │
  Current sensor                      Current sensor
    (INA219)                            (INA219)
        │                                   │
  5 V regulator                   HARDWARE KILL SWITCH
        │                                   │
        ├─── Raspberry Pi 5            ESC bank ×8
        └─── Navigator                      │
                                       Thrusters ×8
```

#### Explaining the architecture

Three features deserve attention.

**The kill switch sits on the thruster chain only.** This is
deliberate. Cutting propulsion stops the vehicle immediately, which is
the safety requirement — but leaving the Pi powered means it continues
logging. If a run is aborted, the record of *why* survives. Cutting all
power would destroy exactly the diagnostic information the abort makes
valuable.

**Fuses precede current sensors.** The fuse is the last-resort
protection against a short; placing it first means even a sensor fault
cannot bypass it.

**Independent current sensors per chain.** Because the packs are
monitored separately, a fault in either is independently detectable.
This is what allows the failsafe logic to treat under-voltage on *either*
pack as mission-ending — losing the logic pack means losing control,
losing the thruster pack means losing propulsion, and neither is
survivable mid-mission.

---

### 7.5 Battery Management and Balancing Safety

#### Charging

- Balance charger with per-cell monitoring, preventing inter-cell
  voltage divergence — a principal cause of premature LiPo failure.
- Charge rate never exceeding 1 C.
- Performed inside a LiPo-safe fire-containment bag, never unattended.
- Charged to full (4.2 V/cell) only shortly before a run; not stored
  charged.

#### In-mission monitoring

- Per-pack voltage and current via INA219 on the I²C bus.
- Either pack going under-voltage triggers the failsafe. Because the
  packs are independently monitored, this is detected regardless of the
  other pack's state.

#### Storage and handling

- Packs idle beyond 48 hours are brought to storage voltage
  (~3.8 V/cell) to maximise cell lifespan.
- Visual inspection before every session for swelling, puncture or
  connector damage. Any damaged cell is retired, not charged.

#### Failure response

- A swollen or damaged cell is isolated, placed in a fire-safe
  container, and disposed of per LiPo guidelines — never recharged.
- The hardware kill switch removes propulsion power on any observed
  regulator collapse or pack anomaly.

> **Software integration gap, recorded for CDR:** the monitoring
> described above is specified in hardware, but the autonomy stack does
> not yet implement a corresponding abort state. A fault state that
> pre-empts all mission states and drives the vehicle to the surface on
> leak or under-voltage detection is scheduled before the Critical
> Design Review.

---

## Summary of Changes from Previous Revision

| Change | Reason |
|---|---|
| ESC count 6 → 8 | Matches the eight-thruster propulsion system |
| PMW3901 optical flow removed | Fixed focus band, texture dependence, and silent failure mode; replaced by down-camera visual odometry |
| RealSense D435i → global-shutter colour camera | 850 nm depth sensing is unusable underwater; rolling shutter biases angle measurements |
| Lights added (2× Lumen R2) | Colour segmentation requires controlled, repeatable illumination |
| Pi 4B → Pi 5, NVMe boot | Compute headroom for dual camera streams; SD corruption is a common field failure |
| Dome port specified | Eliminates 25 % range error and 25 % field-of-view loss from flat-port refraction |
| Power budget updated | Accounts for Pi 5 and lights |
