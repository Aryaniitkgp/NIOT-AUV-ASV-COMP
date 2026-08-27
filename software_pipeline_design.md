# ROS 2 Companion Stack — Software Pipeline Design

**Team Nautilus — NIOT Student AUV Competition (NSCP/2026/REG/0077)**

> Detailed design for the architecture described in the submitted PDR,
> §3.7–3.8 (Control System Design / Software Architecture), Table 4
> (navigation mode arbitration), Table 5 (per-task perception and
> actuation chain), and Table 6 (allocation of control functions between
> layers). This is the design referenced as Forward Work item 1 in
> `PDR_software_stack.md`: integrating the mission stack against ArduSub
> over MAVLink, replacing the current direct-Gazebo actuation path.
>
> This is a design document, not code. It defines the node graph,
> topic/message contracts, and package layout the implementation should
> follow, and maps each existing simulation module onto its place in the
> new structure.

---

## 1. Why this shape

The PDR commits to a specific separation of concerns (§3.8):

> *"a mission sequencer holding the state machine and no perception or
> control mathematics; per-task perception behaviours, each detecting
> its own targets and producing an error signal without knowledge of the
> mission; a command generation layer converting error into MAVLink
> setpoints; and a bridge handling the MAVLink interface."*

The current simulation code (`line_buoy.py`, `buoy.py`, `marker.py`,
`octagon.py`, `line_follow.py`) does not yet have this shape. Detection,
visual-servo control, and mission logic are mixed inside single files —
`buoy.py`, for example, both finds the buoy *and* runs the yaw/sway PID
loops that steer toward it. That coupling was fine for developing and
validating the algorithms quickly against a direct Gazebo thrust
interface, but it is exactly what §3.8 promises to have separated before
CDR. This document is that separation, made concrete.

The four layers are kept apart for a specific reason each:

- **Perception behaviours know nothing of the mission** so a new task
  costs one new node and a couple of FSM states, not a change to control
  or estimation (PDR §3.6.6 makes this claim explicitly; this design is
  what makes it true).
- **The mission sequencer contains no math** so the FSM can be read,
  reviewed, and modified by someone who has never touched a PID gain.
- **Command generation is centralised** so gain tuning happens in one
  place, and so the vision/DVL arbitration of Table 4 is a single
  decision, not six copies of the same logic scattered across task
  modules.
- **The bridge is the only thing that speaks MAVLink** so ArduSub can be
  swapped, mocked, or run against SITL without touching perception or
  mission code at all.

---

## 2. Node graph

```
                         ┌─────────────────────────┐
   forward camera ──────▶│  buoy_perception_node    │──┐
                         └─────────────────────────┘  │
                         ┌─────────────────────────┐  │
   forward camera ──────▶│  gate_perception_node    │──┤
                         └─────────────────────────┘  │
                         ┌─────────────────────────┐  │
   forward camera ──────▶│ torpedo_perception_node  │──┤   /perception/<task>/error
                         └─────────────────────────┘  │   /perception/<task>/status
                         ┌─────────────────────────┐  │        (best-effort, KEEP_LAST 1)
   downward camera ─────▶│  path_perception_node    │──┤
                         └─────────────────────────┘  │
                         ┌─────────────────────────┐  │
   downward camera ─────▶│  bin_perception_node     │──┤
                         └─────────────────────────┘  │
                         ┌─────────────────────────┐  │
   forward cam + DVL ───▶│ octagon_perception_node  │──┘
                         └─────────────────────────┘
                                                        │
                                                        ▼
                                        ┌───────────────────────────┐
                     fused pose/depth   │   mission_sequencer_node   │
                     (from bridge) ────▶│   FSM · search/recovery   │
                                        │   vision/DVL arbitration  │
                                        └───────────────────────────┘
                                                        │
                                     /mission/active_behaviour
                                     /mission/depth_intent
                                     /mission/payload_trigger
                                                        │
                                                        ▼
                                        ┌───────────────────────────┐
                                        │  command_generation_node  │
                                        │  per-loop PID · gain set  │
                                        │  depth cascade            │
                                        └───────────────────────────┘
                                                        │
                                     /cmd/body_velocity (TwistStamped)
                                     /cmd/depth_setpoint
                                     /cmd/payload_servo
                                                        │
                                                        ▼
                                        ┌───────────────────────────┐
                                        │    mavlink_bridge_node    │◀──▶ ArduSub
                                        │  SET_POSITION_TARGET_     │   (MAVLink/UDP,
                                        │  LOCAL_NED (vel fields)   │    loopback)
                                        │  MAV_CMD_DO_SET_SERVO     │
                                        └───────────────────────────┘
                                                        │
                                     /state/pose /state/velocity
                                     /state/depth /state/attitude
                                     (fed back to sequencer + cmd-gen)
```

Perception nodes are leaves that only produce; the mission sequencer is
the only node that reads across all of them; command generation and the
bridge only ever see one behaviour's output at a time, selected by the
sequencer. This is what keeps a new task from touching existing code —
it plugs a new leaf into the graph and adds sequencer states, nothing
downstream changes.

---

## 3. Perception behaviours (Table 5, realised as nodes)

Each node subscribes to its camera topic(s), applies the detection
method already validated in simulation, and publishes an error/status
message at the camera's native rate. None of them subscribe to mission
state — this is the "no knowledge of the mission" property, enforced
structurally rather than by convention.

| Node | Camera(s) | Detection method | Publishes |
|---|---|---|---|
| `path_perception_node` | downward | HSV threshold, band-centroid | cross-track error, heading error, path-present flag |
| `buoy_perception_node` | forward | HSV + circularity gate | yaw/lateral pixel error, apparent radius, locked colour, target-present flag |
| `gate_perception_node` | forward | Bilateral blur → Canny → Hough | heading error, range (from angular depression) |
| `bin_perception_node` | downward | Colour + contour topology | ring/cross classification, pixel offset, confidence |
| `torpedo_perception_node` | forward | Contour hierarchy (inner contour) | alignment error, range, target-present flag |
| `octagon_perception_node` | forward | Ring density | containment boolean, pixel error (pre-containment) |

Each existing simulation module maps onto one of these with the
detection logic kept and the control logic removed:

| Current file | Detection logic → | Control logic (PID) removed to → |
|---|---|---|
| `line_follow.py` | `path_perception_node` | `command_generation_node` (path heading/cross-track loops) |
| `buoy.py` | `buoy_perception_node` | `command_generation_node` (visual-servo yaw/sway/heave-rate loops) |
| `marker.py` | `bin_perception_node` | `command_generation_node` (station-keeping loops) + mission sequencer (drop-latch logic) |
| `octagon.py` | `octagon_perception_node` | mission sequencer (containment-hold timer) |
| — (new) | `gate_perception_node` | — |
| — (new) | `torpedo_perception_node` | — |

`gate_perception_node` and `torpedo_perception_node` are new — L-bar
crossing and torpedo firing are designed but not yet implemented in
simulation (PDR §4.4 / development-status table), so there is no
existing file to split.

---

## 4. Mission sequencer

Owns the finite state machine — a direct evolution of `line_buoy.py`'s
current 17-state machine, with the visual-servo and station-keeping PID
calls stripped out (they move to command generation) and two additions
required by the target architecture that don't exist in the direct-Gazebo
version:

**Vision/DVL arbitration (Table 4).** A `TRANSIT` state family, entered
whenever the active perception behaviour reports `target-present = false`
and no path segment is visible. In `TRANSIT`, the sequencer commands a
guided move toward the known relative offset of the next task using
ArduSub's EKF3 local-position estimate (via the bridge's fused-pose
feedback), rather than any pixel error. The instant any perception node's
`target-present`/`path-present` flag goes true, the sequencer aborts the
guided move and hands control back to the corresponding approach state —
this is the "hand back to vision immediately" row of Table 4, implemented
as a single check in one place rather than duplicated per task.

**Bounded recovery, unchanged in spirit.** Lost-target search states,
per-task attempt counters, and the time-budget check against the mission
clock stay exactly as designed in the current FSM — this part of the
architecture doesn't change with the actuation path, only its outputs
do: instead of a chosen behaviour writing directly to a thrust topic, the
sequencer writes `/mission/active_behaviour` (which node's error the
command generator should act on) and `/mission/depth_intent` (a symbolic
depth target — cruise, touch-altitude, station-keep — not a raw setpoint;
resolving it to metres is the command generator's job, per PDR §3.7.1's
"outer loops are kinematic" framing).

---

## 5. Command generation layer

The only node that runs PID math. It subscribes to
`/mission/active_behaviour` to know which perception topic to act on this
tick, and to that topic itself; every other perception topic is ignored
for the duration of the state. This is the single place the per-loop
gains live — currently split across `buoy.py`, `marker.py`, and
`line_follow.py` — collapsing six copies of "read error, run PID, publish
velocity" into one parametrised implementation with one gain table.

Output is exclusively a body-frame velocity setpoint plus a resolved
depth setpoint — never a thruster command, per Table 6 ("ArduSub:
conversion of the commanded velocity into a body-frame wrench; thruster
mixing... "; "ROS 2: conversion of pixel or positional error into a
commanded body-frame velocity"). `thruster_mixer.py`, which exists in the
current codebase to drive Gazebo's eight thrust topics directly, has no
equivalent here at all — that job now belongs entirely to ArduSub's frame
mixer, on the other side of the bridge.

Payload triggers (marker release, torpedo fire) are passed through from
the mission sequencer largely unchanged, since they are discrete
one-shot commands rather than continuous setpoints — the command
generator forwards them to the bridge without modification.

---

## 6. MAVLink bridge

The only node that imports a MAVLink library. Responsibilities:

- **Outbound.** Body-frame velocity setpoints become
  `SET_POSITION_TARGET_LOCAL_NED` messages with only the velocity fields
  populated (position/acceleration fields masked off), sent to ArduSub in
  `GUIDED` mode — matching §3.7.1's description exactly. Payload triggers
  become `MAV_CMD_DO_SET_SERVO` on channels 9 and 10.
- **Inbound.** Subscribes to ArduSub's telemetry stream
  (`LOCAL_POSITION_NED`, `ATTITUDE`, `VFR_HUD`/depth, `SYS_STATUS`) and
  republishes it as ROS 2 topics — `/state/pose`, `/state/velocity`,
  `/state/depth`, `/state/attitude` — that the mission sequencer and
  command generator read for feedback. This is the fused state Table 6
  assigns to ArduSub and Table 4 assigns as the basis for DVL-aided
  transit; nothing upstream ever touches a raw sensor topic, matching the
  "ROS 2 stack never sees raw sensor data" statements in §3.5 and §3.6.
- **Mode/arming.** Issues the arm/disarm and mode-change commands the
  mission sequencer requests at `INIT` and `SURFACED`.

Because ArduSub runs as a native process on the same Raspberry Pi 4B, the
bridge talks to it over MAVLink/UDP on loopback — no network hop, no
serial link, matching §3.8's stated interface.

**Development-time substitution.** For as long as the ArduPilot–Gazebo
bridge plugin (PDR §4.1/§4.2, the current top development priority)
remains unavailable, this node can be swapped for a stand-in that
republishes the existing `bluerov2_native_bridge.py`/`thruster_mixer.py`
path under the same topic names — `/state/*` fed from Gazebo ground
truth or the modelled sensor chain, `/cmd/body_velocity` mixed straight
to the eight Gazebo thrust topics. Everything upstream of the bridge is
identical in both configurations; only this one node's internals differ.
This is what lets perception and mission-logic development continue
without waiting on the ArduSub SITL integration to land.

---

## 7. Package layout

```
nautilus_ws/
├── nautilus_msgs/            # PerceptionError, MissionState, DepthIntent, etc.
├── nautilus_perception/
│   ├── path_perception_node.py
│   ├── buoy_perception_node.py
│   ├── gate_perception_node.py
│   ├── bin_perception_node.py
│   ├── torpedo_perception_node.py
│   └── octagon_perception_node.py
├── nautilus_mission/
│   └── mission_sequencer_node.py     # FSM + transit supervisor
├── nautilus_control/
│   └── command_generation_node.py    # PID gain table, depth cascade
├── nautilus_bridge/
│   ├── mavlink_bridge_node.py        # target: real ArduSub
│   └── gazebo_bridge_node.py         # dev-time stand-in, same topic contract
└── nautilus_bringup/
    ├── launch/sim.launch.py          # perception + mission + control + gazebo_bridge
    └── launch/hardware.launch.py     # perception + mission + control + mavlink_bridge
```

The two launch files are the whole point of the split: swapping ArduSub
in for Gazebo is a one-line change to which bridge node gets launched,
not a rewrite of six perception files and a mission FSM.

---

## 8. QoS (per PDR §3.8.1, unchanged, restated for this graph)

| Traffic | Topics | Policy |
|---|---|---|
| Camera frames, perception error/status | `/perception/*`, `/state/pose`, `/state/velocity`, `/state/depth`, `/state/attitude` | Best-effort, `KEEP_LAST(1)` — a stale reading should be overwritten, not queued |
| Commands, mission-state transitions | `/mission/active_behaviour`, `/cmd/body_velocity`, `/cmd/depth_setpoint`, `/cmd/payload_servo` | Reliable, `KEEP_LAST(1)` — a dropped command must not leave the vehicle silently acting on an older one |

---

## 9. Worked example: buoy touch, end to end

1. `buoy_perception_node` sees the locked-colour buoy every frame,
   publishes yaw/lateral pixel error and apparent radius on
   `/perception/buoy/error`, regardless of mission state.
2. `mission_sequencer_node` is in `BUOY_APPROACH`; it has already set
   `/mission/active_behaviour = buoy` and `/mission/depth_intent = cruise`.
3. `command_generation_node`, seeing `active_behaviour = buoy`, reads
   `/perception/buoy/error`, runs the visual-servo yaw/sway/heave-rate
   PID loops, and publishes a body-frame `TwistStamped` on
   `/cmd/body_velocity` plus a resolved depth value on
   `/cmd/depth_setpoint`.
4. `mavlink_bridge_node` packs both into one
   `SET_POSITION_TARGET_LOCAL_NED` and sends it to ArduSub, `GUIDED` mode.
5. ArduSub's EKF3, attitude/rate loops, and frame mixer (Table 6) turn
   that into PWM on the eight ESCs.
6. On commit range, the sequencer transitions to `BUOY_TOUCH`, changing
   `depth_intent` to `touch_altitude` — command generation resolves the
   new depth target and the cascade carries the vehicle up before contact,
   exactly as described in §3.8's buoy-touch narrative. No perception or
   bridge code is touched by this transition; only the sequencer's state
   and one symbolic intent value changed.

---

## Open design questions for CDR

- Whether `command_generation_node` should be one node with an internal
  behaviour switch (as drawn above) or six small always-running nodes
  gated by a shared enable topic — the single-node version centralises
  gain tuning but is a bigger unit to test in isolation.
- Exact `nautilus_msgs` field layout for `PerceptionError` — whether it's
  one general-purpose message reused by all six perception nodes, or a
  typed message per task. A shared message keeps command generation
  simple; typed messages catch a wiring mistake at compile time instead
  of at the mixer.
- Whether the dev-time `gazebo_bridge_node` is worth maintaining
  long-term as a SITL-free fast-iteration path even after the ArduPilot–
  Gazebo bridge is working, given how much of this session's debugging
  loop depended on it.
