# CLAUDE.md — REMAP

Coding-agent handoff for the REMAP module (SARC-100, Bela Gem Multi, 96 kHz).

## What REMAP is

One-knob voltage-domain transformation module. Input voltage **X** and bipolar
translation parameter **θ** produce **six simultaneous outputs**: three
transforms (ROTATE, MIRROR, FOLD) at two winding densities (×1, ×2). No mode
switches, no normalling, no hidden state. All six outputs are always live.

REMAP treats voltage as a periodic coordinate on the circle [0, V). ROTATE is
translation, MIRROR is reversal, FOLD is reflection. ×2 performs the same
operation on the half-interval and rescales to full range.

## Signal convention

- Internal reference range: **0 to V**, with `V = 5.0` (single compile-time
  constant; do not scatter literals).
- The six transforms are computed internally as unipolar **[0, V)**, but the module
  **outputs bipolar** using the house DC-coupled convention (RESOLVED 2026-07-19, was
  open question #1). Realized not by a code recenter but by the output stage: outputs go
  to DC-coupled AUDIO outs, and render writes a normalized [0,1] value
  (`remap::mapOutput` = `outV/v`) that the output stage swings bipolar at the jack — so
  the transform range's center (V/2) sits at 0 V. Single mapping point per output kind
  in `lib/dsp/remap.h` (`mapOutput` for the [0,V) transforms, `mapBipolarOutput` for the
  already-bipolar θ output).
- Negative X and θ wrap (ROTATE/MIRROR) or reflect (FOLD) periodically.
  **Never clip.**

## Core math

All formulas use **floored modulo** (result always in [0, m), regardless of
sign of the argument). C's `fmod` is truncated — wrap it:

```c
static inline float wrapf(float x, float m) {
    float r = fmodf(x, m);
    return (r < 0.0f) ? r + m : r;
}

// triangular fold of t into [0, half]... full form used below
static inline float foldf(float t, float range) {   // folds into [0, range]
    return range - fabsf(wrapf(t, 2.0f * range) - range);
}
```

Per sample, with X and θ in volts:

| Output    | Formula                          | Equivalent               |
| --------- | -------------------------------- | ------------------------ |
| ROTATE ×1 | `wrapf(X + θ, V)`                | —                        |
| ROTATE ×2 | `2 * wrapf(X + θ, V/2)`          | `wrapf(2*(X+θ), V)`      |
| MIRROR ×1 | `wrapf(θ - X, V)`                | —                        |
| MIRROR ×2 | `2 * wrapf(θ - X, V/2)`          | `wrapf(2*(θ-X), V)`      |
| FOLD ×1   | `foldf(X + θ, V)`                | `V - |wrapf(X+θ, 2V) - V|` |
| FOLD ×2   | `2 * foldf(X + θ, V/2)`          | `V - |2*wrapf(X+θ, V) - V|` |

FOLD argument is `X + θ` (translation *before* reflection): θ moves the fold
point through the range.

### Verification cases (implement as asserts/tests before hardware)

Full-range rising ramp X: 0→V, θ = 0:
- ROTATE ×1 = X (identity); ×2 = full-scale rising ramp at 2× frequency.
- MIRROR ×1 = falling ramp (with the wrap discontinuity at X = 0);
  ×2 = falling ramp at 2× frequency.
- FOLD ×1 = X (rising ramp); sweep θ: 0 → V/2 → V gives
  rising ramp → triangle → falling ramp, continuously.

Bipolar X, θ = 0:
- FOLD ×1 = |X| for X ∈ [−V, V] (full-wave rectifier). Test with X = −2.5
  → expect +2.5.

Spot values (V = 5):
- ROTATE ×1: X=4, θ=3 → wrapf(7,5) = 2.
- MIRROR ×1: X=1, θ=0 → wrapf(−1,5) = 4.
- FOLD ×1: X=6, θ=0 → foldf(6,5) = 4.
- FOLD ×2: X=3, θ=0 → 2*foldf(3,2.5) = 2*(2.5−0.5) = 4.

**Status: the core math above is implemented in `lib/dsp/remap.h` and all
verification cases + spot values pass in `test/test_remap_dsp.cpp` (run via
`test/run.sh`). The FOLD formulas are confirmed self-consistent with every
behavioral requirement; still pending Timm's sign-off on the spec intent.**

## X and θ conditioning

Both **X** and **θ** are single bipolar CV inputs with **jack + attenuverter + bias**
(house style, RESOLVED 2026-07-19 — was open question #2; the attenuverter/bias replace
the old `θ_knob + θ_cv` split, with the bias knob serving as the one-knob translation
offset). Attenuverter and bias are **analog front-end** (one conditioned ADC read per
control — see drivetrain §2), so the ADC delivers a unipolar [0,1] value that render
expands to bipolar volts **[−V, +V]** via `remap::expandBipolar` (`(cv*2−1)*V`). Range
is the usual ±V (RESOLVED — was open question #4).

Read paths differ **by input type**, matching servo (which does both) and the house rule
"smooth control params, read signals raw":
- **X = signal.** Raw `analogRead` **per audio frame** — no smoothing (may be audio-rate;
  REMAP does no hidden processing on the signal). Like servo's SIG / dual_fulcrum's a,b.
- **θ = control parameter.** Read **block-rate through `CvIn`** (one-pole smoothed,
  `lib/io/cv_input.h`) to de-zipper ADC/pot noise. Like servo's control CVs, and the
  stated "new modules should use `CvIn`" standard (drivetrain).

**This overrides the original spec's "no slew/smoothing/deadband on θ" line** (2026-07-19):
FOLD is sensitive to θ quantization steps, and the spec itself said to raise it rather
than leave it — smoothing lives on θ only, never on the audio-rate X path. If a use case
needs an unsmoothed θ, that's a follow-up question, not a silent change.

### FREEZE θ (added 2026-07-19)

`FREEZE_θ` — digital gate (level). While high, the conditioned θ is **held**
(sample-and-hold, `gHeldTheta` in render.cpp); while low it tracks live θ. The six
transforms keep updating live off X using the held θ, and the θ output emits the held
value. This is an explicit gated input (a jack), not hidden state — it supersedes the
"no hidden state" line for this control, the same way SERVO documents its FREEZE.

## Hardware mapping

Pin indices are **DUMMY placeholders** (penciled 2026-07-19, panel in progress —
see `pins.h`). Confirm against the locked panel before wiring.

| Bela channel | Function | Notes |
| ------------ | -------- | ----- |
| ADC 0 | X in | jack+atten+bias, unipolar [0,1] read → bipolar [−V,+V] |
| ADC 1 | θ in | jack+atten+bias, unipolar [0,1] read → bipolar [−V,+V] |
| Digital in 0 | FREEZE_θ | gate, level (hold conditioned θ while high) |
| Audio out 2–7 | ROTATE×1, ROTATE×2, MIRROR×1, MIRROR×2, FOLD×1, FOLD×2 | DC-coupled, bipolar at jack |
| Audio out 8 | θ out | conditioned/held θ, bipolar |

**Outputs use `audioWrite` to the Gem Multi's DC-coupled AUDIO outs (2–9), NOT
`analogWrite`** — on this hardware `analogWrite` targets a nonexistent/short buffer and
segfaults (see drivetrain notes). This overrides the spec's "DAC 0–5 / analogWrite"
table. **Budget now: 2/8 ADC, 7/8 DC-coupled audio outs (ch 9 spare), 1 digital in** —
the added θ output raised the count (open question #3 still stands: full seven vs a
trimmed first build). Output count is a compile-time table so trimming never touches the
DSP: `remap::kOutputs` (`lib/dsp/remap.h`) + `kOutChannel` (`render.cpp`).

Standing hardware rules that apply here:
- Terminate unused ADC inputs to GND (floating inputs ghost across all
  channels).
- J3 GND (ADC domain) vs GNDA (codec domain): do not mix.
- Input scaling/biasing calibration is a **prerequisite** — raw ADC counts →
  volts conversion must go through the shared calibration path, not ad-hoc
  constants. Until the interface PCB exists, clamp-rail constraints apply at
  the jacks.

## Bela implementation notes

- Per-sample processing in `render()`; all six outputs every audio frame.
  X may be audio-rate; do not decimate X.
- θ knob and θ CV may be read at analog frame rate; document the choice.
- Math is 6 wraps + 2 fabs per sample — no performance concern at 96 kHz.
- Modulo/fold discontinuities at audio rate alias. This is the raw,
  intended behavior (no internal anti-aliasing, consistent with no hidden
  processing). Do not add oversampling without a spec change.
- Output mapping: single function volts[0,V] → DAC code, shared by all six.
  (`remap::mapOutput` in `lib/dsp/remap.h` is that function.)

## Code organization

- `lib/dsp/remap.h` — SHARED, Bela-agnostic core: `wrapf`/`foldf`, `transform`,
  the `kOutputs` compile-time table + `computeAll`, and the θ-conditioning /
  output-mapping seams. Plain C++, unit-tested off-device. No Bela deps.
- `modules/remap/render.cpp` — Bela glue only: raw per-frame X/θ reads +
  `expandBipolar`, FREEZE_θ sample-and-hold on θ, per-frame `computeAll`, DC-coupled
  `audioWrite` via `mapOutput`/`mapBipolarOutput`, GUI meters. Pin indices (`pins.h`) are
  dummies pending the panel; `stubs/` mirrors the other modules for off-device IntelliSense.
- Dependency rule: modules depend on `lib/`; `lib/` NEVER depends on a module.

## Open questions

1. ~~Unipolar vs bipolar outputs~~ — RESOLVED 2026-07-19: bipolar, house DC-coupled
   convention (write normalized [0,1] to audio outs).
2. ~~θ attenuverter vs fixed-gain CV sum~~ — RESOLVED 2026-07-19: attenuverter + bias,
   house style; X too.
3. **7/8 DC-coupled audio outs on one module** — resident module or trimmed first build?
   Still open (worsened by the added θ output). `kOutputs`/`kOutChannel` make trimming a
   table edit.
4. ~~θ range~~ — RESOLVED 2026-07-19: ±V.

Remaining hardware unknowns (not decisions, just pending): exact panel pin indices
(`pins.h` values are dummies), and the shared ADC→volts calibration path (does not exist
yet; render currently expands raw ADC directly).

## Status

Spec from panel planning brief, 2026-07-19. Core DSP (`lib/dsp/remap.h`) + tests passing
(`test/test_remap_dsp.cpp`, run `test/run.sh`). Bela render glue written
(`render.cpp`, `pins.h`, `stubs/`) with **dummy pin indices** and the 2026-07-19 spec
additions (θ output, FREEZE_θ, bipolar outputs, X/θ atten+bias). Not yet built/run on
hardware. FOLD formulas confirmed against all behavioral requirements by executable
tests; core-math spec intent still pending Timm's final sign-off.
