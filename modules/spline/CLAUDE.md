# SPLINE — module guide for Claude Code

This file orients an agent working in `modules/spline/`. It is operational, not a design treatise.

## What SPLINE is
A 6-knot open CV scanner for the SARC-100, prototyped in C++ on a **Bela Gem Multi**. SPLINE holds
6 independently-patchable knot values across an open curve (x = 0..1, no wraparound) and reads
them out at a position set by SCAN, with a continuously variable interpolation character between
knots — from discontinuous (nearest-neighbor hold) through linear to a smooth spline curve.
Designed to be used in **multiples**, chained via SCAN POSITION OUT so several SPLINE units can
scan in lockstep.

## Core behavior
- 6 **SEGMENT** inputs, each an independent CV (no normalling between them) — these are the
  curve's 6 knot values, evenly spaced at x = 0, .2, .4, .6, .8, 1.
- **SCAN** sets the read position, 0..1. The curve is OPEN, not circular: out-of-range SCAN
  clamps to whichever end is closer rather than wrapping — there is no "segment 6 blends back
  into segment 1."
- **INTERPOLATE** is a single continuous control over the read character, computed as a
  cross-fade rather than a switch:
  - 0 → discontinuous: output holds the nearest knot's value, jumping at the midpoint between
    two knots (sample & hold) — every knot is reachable, including both endpoints.
  - 0.5 → linear: output ramps linearly between the two surrounding knots.
  - 1 → smooth: output curves through all 6 knots via a Catmull-Rom-style cardinal cubic
    (continuous value AND slope at knots), with the two end knots duplicated for the tangent
    math so curvature is zero at the two ends (a "clamped" boundary, not a "natural" one).
  - Values between these morph continuously (e.g. 0.25 blends discontinuous and linear) — there
    is no hard switch between the three characters.
- **OUT** is the scanned/interpolated value. **SCAN POSITION OUT** is the clamped, effective
  scan position (0..1) actually used to read the knots — *after* this module's own SCAN
  attenuverter/bias, not the raw external CV. Patch this into a second SPLINE's SCAN IN (with
  that unit's own attenuverter at unity / bias at 0) to chain multiple units at the exact same
  scan position, since each unit's own SCAN attenuverter/bias would otherwise shift the same
  external CV differently per unit.

## DSP unit
- `lib/dsp/spline.h` — `Spline<N>`, the open N-knot interpolating curve described above.
  Templated on knot count (not hardcoded to 6) so other modules can reuse it at a different
  size; this is the same unit `modules/servo/` originally scaffolded as `Spline5` before its
  spline stage was removed in the 2026-06-21 redesign — SPLINE is the module that actually owns
  it now. Bela-agnostic, header-only, like the rest of `lib/dsp/`.
- Tested in `test/test_spline_dsp.cpp`: knot exactness across all interpolation settings,
  discontinuous-mode nearest-neighbor behavior (including reaching the end knots), linear-mode
  exactness, smooth-mode C1 continuity + no NaN/inf, open-curve clamping at the ends (NOT
  wrapping), and continuous morphing between interpolation characters.

## Voltage conventions
- **SEGMENT values and OUT are bipolar** (±full-scale, 0.5 = 0 V), matching the rest of the
  system: segment CV is read `2v−1`, knot sliders are bipolar (default 0 = center), OUT is
  written `0.5 + 0.5·x` (clamped — smooth-mode Catmull-Rom can overshoot the knot range).
- **SCAN and INTERPOLATE stay unipolar [0,1]** — a read position and a morph amount, not
  bipolar values. SCAN POSITION OUT is written raw [0,1] for chaining (its exact chained
  voltage is unconfirmed — see below).
- **Assumes the segment CV front-end centers 0V at ADC mid** (0V → 0.5), like every other
  bipolar input in the system. If a segment jack instead reads 0V → 0 (as observed on the
  bench before this was set), knots sit low until the input is biased to center — a
  front-end/calibration matter, not code.

## I/O — UNCONFIRMED, no pin numbers were specified when this module was designed
See `pins.h`. Current placeholder uses the full standard 0-7 analog range (6 segments + SCAN +
INTERPOLATE = exactly 8, fits without a wing-off header) and audio outs 2/3 (DC-coupled,
following the pattern EFFECTOR established in `modules/servo/`) for OUT / SCAN POSITION OUT.
Confirm before wiring a real panel.

## Code organization
- `lib/` is the SHARED, Bela-AGNOSTIC core — `lib/dsp/spline.h` lives there, not inlined here.
- `modules/spline/` contains ONLY module-specific code: the Bela render loop, analog I/O
  mapping, wiring `Spline<N>` to the GUI/CV surface.
- Dependency rule: modules depend on `lib/`; `lib/` NEVER depends on a module.

## Real-time constraints
Same as every other module here — no heap allocation or blocking calls in `render()`; GUI
metering runs in the auxiliary task, not the audio thread.
