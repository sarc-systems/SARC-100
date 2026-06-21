# SERVO — module guide for Claude Code

This file orients an agent working in `modules/servo/`. It is operational, not a design treatise.
Full rationale lives in `docs/` (SARC-100 module index + design notes); read those for *why*, read
this for *what* and *how*.

**2026-06-21: redesigned.** SERVO is now a two-timescale PID leveling/AGC element with TARGET as
its own independent input (no internal spline/curve-shaping) and CONFIDENCE redefined as a
magnitude-AND-stability metric on ERROR. This supersedes the original spline-based seed design
below.

## What SERVO is
A two-timescale PID leveling/AGC module for the SARC-100 (a patch-programmable analog field
computer; this module is prototyped in C++ on a **Bela Gem Multi**). SERVO drives SIG toward
TARGET via a PID stage, and reports three parallel taps off that core computation — ERROR (raw
deviation), EFF (the drive signal), and CONFIDENCE (requires error to be both small and steady) —
plus two fully independent magnitude-extraction outputs, MAGNI(ref) and MAGNI(sig). It is the
first module built; its DSP primitives become the shared core (`lib/`) that later modules
(phase network, etc.) reuse.

Designed to be used in **multiples** and coupled in feedback networks — nothing in the code should
assume a single global instance.

## Core behavior (the control loop)
- `MAGNI(ref)` and `MAGNI(sig)` extract magnitude/envelope from REF and SIG respectively.
  Independent outputs — **no internal routing to TARGET or ERROR**. MAGNI(ref) sits adjacent to
  the TARGET jack on the panel purely so a shorting bar or patch cable can manually bridge them if
  desired. No normalling, no default signal path. If you want the old "ref's envelope is the
  target" behavior, patch MAGNI(ref) into TARGET externally.
- `ERROR = TARGET - SIG`. Raw deviation, unprocessed — uses the raw jacks, not MAGNI.
- A **PID** produces `EFF`, the drive signal that acts on the plant. Includes derivative
  filtering and **anti-windup** (conditional integration) — REQUIRED, because in feedback patches
  the servo is frequently asked for an unreachable target and must not wind up.
- `CONFIDENCE` = magnitude AND stability of ERROR, normalized, exported for patching elsewhere —
  **not read back internally, does not gate or scale EFF**. Requires error to be BOTH small and
  steady: large error reads as low confidence even if perfectly steady, and small-but-jittery
  error also reads as low confidence. Computed as `magnitudeScore * stabilityScore` so either
  factor being low drags the whole thing down — this is a refinement of the original
  "`1 - slew(ERROR)`" framing, which omitted the magnitude term and would have let a large but
  steady error read as confident.
- ERROR, EFF, and CONFIDENCE are three parallel taps off the same core computation — none feeds
  back into another internally.

## Two timescales (do not conflate)
1. **Loop timescale** (`SERVO RATE` input, governs PID I + D): how fast the servo pursues. Scales
   the integral rate and derivative-filter together by fixed design-time ratios that set the loop
   "feel."
2. **Envelope-follower timescale** (`ENV RATE` input, on REF & SIG, feeding MAGNI): KEEP SEPARATE
   from the loop timescale — do NOT gang it to SERVO RATE.

CONFIDENCE's own smoothing (both the slew-rate estimate and its leaky-integrator rate) is a fixed
internal constant in `render.cpp`, not CV/GUI-exposed — there is no Governor-Time-Mult-style input
in this design. It's still kept slower than the loop by construction (a fixed fraction of the
loop's own rate), just not user-tunable yet.

## I/O (24 panel elements total)
CV inputs (5), each with jack + attenuverter + offset:
- `TARGET`, `REF`, `SIG`, `ENV RATE`, `SERVO RATE`

CV outputs (5), jack only:
- `ERROR`, `EFF` (+ UNI/BIPOLAR switch), `CONFIDENCE`, `MAGNI(ref)`, `MAGNI(sig)`

Digital I/O (4):
- `FREEZE` (jack) — gate. Holds **only** the PID/EFF stage at its current value (integrator stops
  accumulating, EFF locked). ERROR, MAGNI(ref), MAGNI(sig), and CONFIDENCE keep updating live off
  REF/SIG/TARGET — FREEZE gates the PID stage's output only, not the upstream computations.
- `RESET` (jack) — trigger. Clears the PID integrator. Behavior set by `RESET MODE`:
  - `0` → EFF returns to neutral (0 for unipolar / center for bipolar)
  - `last` → EFF held at current value; integrator re-seeded so resumption is BUMPLESS
- `RESET MODE` switch — 0V vs last-value/bumpless (see above)
- `EFF UNI/BIPOLAR` switch — output range -1..1 vs 0..1

Removed from the original seed: SPLINE 1-5, TENSION (no internal target-curve anymore — TARGET is
independent), GOVERNOR TIME MULT (CONFIDENCE's rate is now a fixed constant), EFFECTOR INVERT (not
part of the new 24-element panel count).

## Output signal type
EFF + CONFIDENCE together still form the SARC universal type **(value, confidence)** for
downstream patching — keep them coherent and co-emitted even though CONFIDENCE's definition
changed (stability, not tracking success). ERROR and MAGNI(ref)/MAGNI(sig) are additional
independent taps, not part of that tuple.

## Memory hooks (build later, design for now)
SERVO holds NO long-term memory (that lives in a separate memory-surface module). But expose clean
state access (current effector, integrator state, confidence) and the FREEZE/RESET vocabulary so a
memory surface can attach later without retrofitting.

## Code organization (IMPORTANT)
- `lib/` is the SHARED, Bela-AGNOSTIC core. PID (`lib/dsp/pid`), confidence/slow-integrator
  (`lib/dsp/confidence`), envelope follower (`lib/dsp/envelope`) are used by this module.
  `lib/dsp/spline` is no longer used by SERVO (TARGET is now independent) — it was generalized
  (`Spline5` -> templated `Spline<N>`, open curve, 3-way interpolation character) and is now
  owned by `modules/spline/`. Write all of these as plain C++ with NO Bela dependencies so they
  stay unit-testable off-device.
- `modules/servo/` contains ONLY servo-specific code: the Bela render loop, analog I/O mapping,
  wiring the lib units together, panel/switch handling.
- Dependency rule: modules depend on `lib/`; `lib/` NEVER depends on a module.

## Testing
- `test/test_servo_dsp.cpp` exercises the `lib/dsp/` units directly (PID + plant sim, confidence
  leaky-integrator behavior, spline — still tested even though SERVO itself no longer calls it,
  since other modules may). Validate math on desktop BEFORE running on the Bela; only the I/O
  glue needs the device.
- Not yet covered on desktop: the CONFIDENCE = magnitudeScore * stabilityScore formula lives in
  `render.cpp` itself (not `lib/dsp/`), since it's specific to this module's wiring rather than a
  generic reusable unit — there's no desktop test for it yet.

## Bela / hardware notes
- Target: Bela Gem Multi, DC-coupled analog I/O. TARGET/REF/SIG/ENV RATE/SERVO RATE are confirmed
  on the standard analog ins (A0-A4 in that order); EFFECTOR is confirmed on audio out 2
  (DC-coupled, empirically verified with a 0.2Hz test LFO). ERROR/CONFIDENCE/MAGNI(ref)/MAGNI(sig)
  (audio outs 3-6) are still ASSUMPTIONS in `pins.h` — confirm before wiring.
- Prefer 96k if it holds; the DSP here is cheap (biquads, integrators) and should fit easily.

## Things that will bite (read before editing)
- Anti-windup is not optional. Skipping it looks fine until the servo saturates, then lurches.
- RESET `last` must be bumpless (re-seed the integrator), or it defeats its own purpose.
- CONFIDENCE requires error to be BOTH small and steady — don't simplify it back to pure
  magnitude (the old "tracking success" measure) or pure slew (the first draft of this redesign,
  which let a large-but-steady error read as confident). It's a product of both scores on purpose.
- FREEZE must NOT gate ERROR/MAGNI/CONFIDENCE — only the PID/EFF stage. It's tempting to wrap the
  whole per-channel block in `if(!freezeActive)`; don't.
- RESET must NOT clear CONFIDENCE — it's decoupled from the PID/EFF stage RESET targets.

## Canonical use (for context, not a code requirement)
The signature patch: `CONFIDENCE` → a plant parameter (e.g. a filter's resonance/Q). As ERROR
shrinks and settles, confidence rises and pushes the plant toward instability; ERROR grows or
starts moving again, confidence falls, the plant relaxes — a self-seeking edge-of-chaos loop.
Requiring both smallness and steadiness (rather than either alone) means a stuck-but-steady error
no longer reads as falsely confident the way a pure-slew metric would.
