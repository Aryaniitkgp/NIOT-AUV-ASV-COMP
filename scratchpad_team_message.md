Hey team — went through Sections 4–7 (Mechanical, Propulsion, Electronics, Power) against what's actually built in the software/sim side. Found a few things worth fixing before we submit, plus confirmed one thing that's now settled.

## Settled — no action needed

**8 thrusters is correct.** Confirmed with Aryan — we're going with 8, not 6. This actually resolves a contradiction that was already in the doc (see below), so nothing to change here except making the numbers consistent.

## Needs fixing

**1. Thruster count is inconsistent across the doc itself — pick 8 everywhere.**

Three places disagree right now:
- Section 5.1/5.2 says "six-thruster vectored propulsion system"
- Section 7.1's own power table says "Thrusters x 8 ... 6.25 x 8 ... 800 [W]"
- Section 6.4 says PWM channels "1-8" drive the thrusters — which only makes sense with 8, not 6

Since we're confirmed on 8, Section 5 needs rewriting: 4 horizontal thrusters is unchanged, but vertical goes from 2 to 4 (T5–T8), not 2. The allocation matrix in Section 10.2 also needs updating — `B` becomes 4×8, not 4×6.

This matters beyond just the count: the pitch-cancellation claim in 5.1 ("driven identically to... cancel out pitching moments") depends on the vertical thrusters being placed symmetrically fore-and-aft about the CoG. With 4 vertical thrusters that's straightforward to arrange and verify; with 2 it's a tighter constraint. Whoever finalizes the thruster mounting layout should double check the fore/aft placement so that claim is actually true, not just asserted.

**2. PMW3901 optical flow sensor — recommend dropping it, reuse the downward camera instead.**

We already carry a Raspberry Pi Camera Module 3 Wide facing down (mounted in the same bottom air-gap window Section 4.2/6.3 currently reserves for the PMW3901). That camera can do the same job — estimating horizontal velocity — better than the PMW3901 would here:

- PMW3901 is fixed-focus, tuned for a narrow altitude band. Outside that band (e.g. during a bin-drop descent or near-surface ascent) it silently degrades — no error flag, just bad numbers feeding the EKF.
- It needs high-contrast textured ground. Pool tile is uniform and murky water lowers contrast further — bad match for the sensor.
- It's a black box: velocity number out, no way to inspect why a reading is wrong.

The Camera Module 3 Wide has autofocus, gives us an actual image we can log/debug, and is already running for path detection — so velocity comes essentially free from data we're already capturing (standard sparse optical flow / feature tracking on the same feed, scaled to metric units using Bar30 altitude — this is the same altitude-scaling idea already written into Section 9.3, just pointed at the camera instead of the PMW3901).

If we do this: PMW3901 and its breakout board come out entirely (one less waterproof penetration, one less thing to seal/leak-check), its UART line in 6.4/6.5 goes away, and Section 6.3's "Bottom Window (Air-Gap)" entry just becomes the down camera's mount, nothing added. Software side, I'll handle wiring the tracker into the estimator — no action needed from you there.

Worth a quick pool test with just the camera before we lock this in, but that's a laptop and ten minutes, not a purchase decision.

**3. Section 6.4 — "torpedo reclamation mechanism" should probably say "release" or "firing."**

Section 4.4 describes the torpedo servo as spring-loaded, latch-disengage-to-fire — a release mechanism. "Reclamation" in 6.4's payload output line implies retrieving/resetting the torpedo, which isn't what 4.4 describes. Probably just a word swap, but a reviewer will notice the mismatch — can whoever wrote 6.4 fix the wording?

## What's fine as-is

Everything else in Mechanical/Electronics/Power reads as self-consistent and doesn't touch anything on the software side — frame material, dual pressure hull split, O-ring sealing, battery chemistry/charging, INA219 monitoring, component placement. No changes needed there from my end.

One integration note for later, not urgent: if the battery low-voltage failsafe (Section 7.5) is meant to force the vehicle to abort and surface, there's currently no FSM state on the software side that does that — it's a gap on my end, not yours, just flagging so it doesn't fall through the cracks before CDR.
