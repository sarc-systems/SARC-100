# SERVO — module guide for Claude Code

This file orients an agent working in `modules/servo/`. It is operational, not a design treatise.
Full rationale lives in `docs/` (SARC-100 module index + design notes); read those for *why*, read
this for *what* and *how*.

## What SERVO is
A cybernetic servo/governor module for the SARC-100 (a patch-programmable analog field computer;
this module is prototyped in C++ on a **Bela Gem Multi**). SERVO observes a relationship between two
input signals, drives an effector to make a controlled variable track a painted target relationship,
and **reports its own sustained success as a confidence signal**. It is the first module built; its
DSP primitives become the shared core (`lib/`) that later modules (phase network, etc.) reuse.

Designed to be used in **multiples** and coupled in feedback networks — nothing in the code should
assume a single global instance.

## Core behavior (the control loop)
- Inputs REF and SIG are the two observed signals.
- A **5-point spline** (with a TENSION control) paints the target: desired controlled-variable value
  as a function of the reference. The servo tracks Y-given-X along this curve, not a fixed setpoint.
- A **PID** produces the EFFECTOR output that drives the plant. Includes derivative filtering and
  **anti-windup** (integral clamping / back-calculation) — REQUIRED, because in feedback patches the
  servo is frequently asked for an unreachable target and must not wind up.
- **Confidence** = a SLOW integration of sustained tracking success (NOT the PID integral term; a
  separate, slower integrator). It is the module's judgment of how well it has been holding the
  target over time. This is a first-class output, not a meter.

## Two timescales (do not conflate)
1. **Loop timescale** (`TIME` input, governs PID I + D): how fast the servo pursues. Scales the
   integral rate and derivative-filter together by fixed design-time ratios that set the loop "feel."
2. **Confidence timescale** (`GOVERNOR TIME MULT` input): how much slower the confidence integration
   runs relative to the loop. Confidence must be slower than the PID integral.
3. **Envelope-follower timescale** (`ENV RESPONSE` input, on REF & SIG): KEEP SEPARATE from the loop
   timescale. The followers track the input envelopes and must stay valid regardless of how slow the
   loop is set; do NOT gang them to TIME.

## I/O (Bela analog — all DC-coupled; some require the wing-off audio inputs, see below)
Inputs:
- `REF` — reference signal
- `SIG` — controlled/observed signal
- `SPLINE 1..5` — five spline control points (target curve)
- `TENSION` — spline tension (angular ↔ smooth interpolation)
- `TIME` — loop (PID I+D) timescale
- `GOVERNOR TIME MULT` — confidence-timescale multiplier (slower than loop)
- `ENV RESPONSE` — envelope-follower timescale for REF & SIG

Outputs:
- `EFFECTOR` — PID control output (drives the plant)
- `ERROR` — instantaneous tracking error
- `CONFIDENCE` — slow integrated success (the governor judgment)

Digital:
- `FREEZE` (gate) — hold the loop: stop integrator accumulation, hold EFFECTOR at current value.
- `RESET` (trigger) — clear integrator + confidence. Behavior set by a panel switch `RESET 0/last`:
  - `0` → effector returns to neutral (0 for unipolar / center for bipolar)
  - `last` → effector held at current value; integrator re-seeded so resumption is BUMPLESS
    (clearing the integrator naively would lurch — seed it to reproduce the held output).

Switches (panel, not CV):
- `EFFECTOR MODE` — invert and unipolar/bipolar. The PID assumes a plant sign (more effector →
  more controlled variable); patching a plant backwards requires the invert. Provide this.
- `RESET 0/last` — see above.

## Output signal type
EFFECTOR + CONFIDENCE together form the SARC universal type **(value, confidence)**. Keep them
coherent and co-emitted; downstream modules treat (value, confidence) as one tuple. Do not let
confidence become merely cosmetic.

## Memory hooks (build later, design for now)
SERVO holds NO long-term memory (that lives in a separate memory-surface module). But expose clean
state access (current effector, integrator state, confidence) and the FREEZE/RESET vocabulary so a
memory surface can attach later without retrofitting.

## Code organization (IMPORTANT)
- `lib/` is the SHARED, Bela-AGNOSTIC core. SERVO is the first module, so it CREATES these reusable
  units here (do not inline them in the module): PID (`lib/dsp/pid`), spline-with-tension
  (`lib/dsp/spline`), confidence/slow-integrator (`lib/dsp/confidence`), envelope follower
  (`lib/dsp/envelope`). Write them as plain C++ with NO Bela dependencies so they are unit-testable
  off-device.
- `modules/servo/` contains ONLY servo-specific code: the Bela render loop, analog I/O mapping,
  wiring the lib units together, panel/switch handling.
- Dependency rule: modules depend on `lib/`; `lib/` NEVER depends on a module.
- The phase-network module will reuse PID + spline + confidence verbatim — design them parameterized
  and general, not servo-specific.

## Testing
- Write a desktop test harness in `test/` that links the `lib/` units (no Bela) and exercises:
  - spline: correct interpolation, tension behavior, C0/C1 continuity, no discontinuities.
  - PID + plant sim: feed a simple simulated plant (gain, one-pole, saturating gain) and confirm the
    loop converges and does NOT wind up at saturation.
  - confidence: rises under sustained tracking, decays under disturbance, slower than the loop.
- Validate the math on desktop BEFORE running on the Bela. Only the I/O glue needs the device.

## Bela / hardware notes
- Target: Bela Gem Multi, DC-coupled analog I/O. The standard analog inputs are limited; several of
  these inputs require the **wing-off / 2x12 header** DC-coupled audio inputs with level-shifting
  conditioning (CV centered to the ADC range). Do not assume all inputs are on the standard analog
  ins. Confirm pin mapping with the user before wiring.
- Prefer 96k if it holds; the DSP here is cheap (biquads, integrators) and should fit easily.
- DC-coupled outputs required for EFFECTOR/ERROR/CONFIDENCE (they carry slow CV, not audio).

## Things that will bite (read before editing)
- Anti-windup is not optional. Skipping it looks fine until the servo saturates, then lurches.
- RESET `last` must be bumpless (re-seed the integrator), or it defeats its own purpose.
- Effector sign: if the loop diverges on a real plant, it's probably patched backwards — that's the
  EFFECTOR MODE invert, not a bug in the PID. Don't "fix" convergence by changing the PID.
- Keep envelope-follower timescale independent of loop timescale (see Two timescales).
- Don't build memory in. Confidence is a fast-ish local integrator here; long-term persistence is a
  different module.

## Canonical use (for context, not a code requirement)
The signature patch: `CONFIDENCE` → a plant parameter (e.g. a filter's resonance/Q). The servo holds
its amplitude target; as it succeeds, confidence rises and pushes the plant toward instability; the
servo struggles, confidence falls, the plant relaxes — a self-seeking edge-of-chaos loop. This is why
confidence must be a clean, slow, first-class output.
