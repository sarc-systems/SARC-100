# DUAL FULCRUM — CLAUDE.md
## SARC-100 Module Spec / Coding Agent Handoff

---

## Overview

DUAL FULCRUM is two independent FULCRUM instances on a single 4-column Serge 4U panel. Each FULCRUM is a rotational equal-power crossfader with independent magnitude control. Two FULCRUMs share a panel for space efficiency; they are electrically and computationally independent — no shared state, no normalling between instances.

---

## Core Math (per instance)

```
a_scaled  = A_in * a_atten + a_bias   (when bias engaged)
b_scaled  = B_in * b_atten + b_bias   (when bias engaged)

theta_eff = (theta_in * theta_atten + theta_pot) * REVERSE_sign
r_eff     = (r_in * r_atten + r_pot) * INVERT_sign

OUT       = r_eff * (a_scaled * cos(theta_eff) + b_scaled * sin(theta_eff))
QUAD_OUT  = r_eff * (-a_scaled * sin(theta_eff) + b_scaled * cos(theta_eff))
```

Where:
- `REVERSE_sign` = −1 when REVERSE gate is high, +1 otherwise (maps θ → −θ, equivalent to π/2−θ shift, swaps which input is favored as θ increases)
- `INVERT_sign` = −1 when INVERT gate is high, +1 otherwise (flips polarity of entire output)

OUT and QUAD_OUT are orthogonal: `OUT² + QUAD_OUT² = r_eff² * (a_scaled² + b_scaled²)`. Total power is conserved independent of θ.

---

## Voltage / Range Conventions

- **θ range:** [−π/2, π/2] maps to the full CV input range (±V). At θ=0: OUT = r·a_scaled, QUAD_OUT = r·b_scaled. At θ=+π/2: OUT = r·b_scaled. At θ=−π/2: OUT = −r·b_scaled (or equivalently: full B, inverted).
- **r:** bipolar. Negative r inverts the output continuously — this is intentional behavior, not a fault condition.
- **A/B inputs:** accept audio or CV.
- **Outputs:** DC-coupled bipolar audio.
- **Gate inputs (INVERT, REVERSE):** standard digital gate, high/low threshold per SARC-100 convention.

---

## Panel Elements (per instance — 16 elements, 8 per column)

### Column 1 — Signal

| Position | Element | Type | Notes |
|---|---|---|---|
| 1 | A IN | jack | Audio/CV input |
| 2 | A ATTEN | knob | Attenuverter (bipolar, center = unity, CCW = inverted) |
| 3 | A BIAS | pull-pot | DC offset on A; pulled out = hard bypass (0V), pushed in = active |
| 4 | B IN | jack | Audio/CV input |
| 5 | B ATTEN | knob | Attenuverter (bipolar, center = unity) |
| 6 | B BIAS | pull-pot | DC offset on B; pulled out = hard bypass (0V), pushed in = active |
| 7 | OUT | jack | Primary output |
| 8 | QUAD OUT | jack | Orthogonal output (90° phase offset from OUT) |

### Column 2 — Control

| Position | Element | Type | Notes |
|---|---|---|---|
| 1 | θ IN | jack | Rotation/crossfade CV |
| 2 | θ POT | knob | Manual θ offset, summed with CV |
| 3 | θ ATTEN | knob | Attenuverter on θ IN |
| 4 | r IN | jack | Magnitude CV (bipolar) |
| 5 | r POT | knob | Manual r offset, summed with CV |
| 6 | r ATTEN | knob | Attenuverter on r IN |
| 7 | INVERT | jack | Gate in — high = flip output polarity |
| 8 | REVERSE | jack | Gate in — high = negate θ (swap which input is favored) |

---

## Behavioral Notes

### A/B BIAS (pull-pots)
A plain centered knob gives "approximately 0V" bias. Pull-pot gives a guaranteed hard zero when disengaged — physical disconnect, not just a centered knob. Default state (pulled out) is zero bias, consistent with the system's no-hidden-defaults philosophy. When engaged (pushed in), the bias knob's range is ±V, summed after the attenuverter.

### INVERT gate
Equivalent to negating r. Since r is already bipolar and can be driven negative by CV, INVERT adds a manual/gate path for the same operation without consuming the r CV input. Useful both as a standalone polarity-invert on a mixed signal, and as part of the quadrant-counter sine-derivation patch.

### REVERSE gate
Maps θ → −θ. At any fixed θ value, this swaps cosθ and sinθ (since cos(−θ) = cos(θ) and sin(−θ) = −sin(θ)), changing which input is favored as θ increases. Useful as a standalone crossfade-direction flip, and as the other half of the quadrant-counter patch.

### Quadrant-counter sine derivation patch
Connect an external 2-bit ripple counter (or pair of toggle flip-flops) to INVERT and REVERSE gate inputs, clocked off USG's reset edge. Results in exact, click-free sine derivation from a [0, V] sawtooth ramp:
- FF1 output → REVERSE IN
- FF2 output (clocked by FF1 output) → INVERT IN
- USG ramp → θ IN (with offset to center at 0V)
- +V reference → B IN (A IN grounded or zero)
- OUT = exact sine at 1/4 USG ramp frequency
- QUAD OUT = exact cosine, same rate

No tuning, no gain trimming required. Correctness is guaranteed by the symmetry of the quarter-sine segments, not by component matching.

### QUAD OUT
The naturally-computed orthogonal component: r × (−A sinθ + B cosθ). Always available alongside OUT at zero extra cost. Forms an I/Q pair with OUT. When used for the sine derivation patch, QUAD OUT carries cosine (same rate, 90° offset).

---

## Panel Layout

DUAL FULCRUM occupies a full 4-column Serge 4U panel (100.4mm × 175mm):
- **Columns 1–2:** FULCRUM 1
- **Columns 3–4:** FULCRUM 2

No connections between instances. Panel label: **DUAL FULCRUM**. Each instance labeled **FULCRUM 1** / **FULCRUM 2** or simply **F1** / **F2** at the column header.

---

## Implementation Notes

### Analog I/O budget
- Runs on Bela Gem Multi (PocketBeagle 2, 96kHz, 8 analog in / 8 analog out, digital GPIO).
- **8 analog inputs total, 4 per instance:** A IN, B IN, θ IN, r IN. Fills the Bela ADC pool exactly.
- **4 analog outputs total, 2 per instance:** OUT, QUAD OUT. Leaves 4 Bela output channels unused on this module.
- **All panel pots (attenuverters, bias, offset) are analog pre-conditioning** — they scale and offset the CV signal in the analog domain before it reaches the ADC. The ADC sees the post-pot, pre-processed signal. No pot requires a separate ADC read.
- **INVERT and REVERSE are digital GPIO** — gate inputs, no ADC budget consumed.

### DSP
- cos/sin computation: use lookup table or hardware-efficient polynomial approximation valid over [−π/2, π/2]. At 96kHz with two instances, cost is modest.
- REVERSE is applied before the trig lookup: negate θ_eff before passing to cos/sin.
- INVERT is applied after the multiply: negate r_eff (or negate the final output — equivalent).
- Gate inputs work correctly as level-sensitive (high = active). If driven by an external toggle flip-flop for the quadrant-counter patch, the level at each audio callback is all that matters — no edge detection required in software.

### Pull-pot bias
Pull-pot engage/disengage is a hardware concern (Forest). From the software side, treat A BIAS and B BIAS as standard CV offsets summed after the attenuverter. If the pull-pot hard-disconnects to a known 0V rail when pulled out, software sees 0V and needs no special case. Confirm with Forest before assuming this.

---

## Module Status

**Coded (first pass).** Panel spec locked. Preceded by SERVO, TURBINE, TAG (all coded).

### Implementation (as coded — `render.cpp` / `pins.h`)

- **Equal-power rotation lives in the shared lib**: `lib/dsp/crossfade.h`
  (`equalPowerRotate(a,b,cosθ,sinθ) → {out,quad}`, plus a `[0,1]` `equalPowerFade`
  convenience). Bela-agnostic, unit-tested (`test/test_crossfade_dsp.cpp`). Reuse it for any
  A/B equal-power blend elsewhere. `render.cpp` is just I/O + voltage mapping around it.
- **Pin map** (`pins.h`): analog in — F1 A/B/θ/r = 0/1/2/3, F2 = 4/5/6/7. Outputs are the
  Gem Multi's **DC-coupled audio outs** via `audioWrite` (not `analogWrite`): F1 OUT/QUAD =
  2/3, F2 OUT/QUAD = 4/5. Digital in — F1 INVERT/REVERSE = 0/1, F2 = 2/3.
- **Voltage mapping (defaults; resolves Open Q1)** — Bela analog in is unipolar [0,1] with
  bipolar signals centered at 0.5 by the front-end: `a,b,r = 2v−1`; `θ = (v−0.5)·π` →
  [−π/2, π/2]; outputs `0.5 + 0.5·x` clamped to [0,1]. The two mapping lines (`bipolar`,
  `thetaMap`) are the tuning point once the real front-end scaling is confirmed.
- **Correlated-input headroom**: with A and B carrying the same full-scale signal, OUT peaks
  at √2·r at θ=45° (the equal-power +3 dB bump) and is clamped. Normal (uncorrelated /
  single-input) patches — including the sine-derivation patch — stay within range.
- Pots (att/bias/offset) are analog front-end; code reads one value per input. Runs
  per-sample at 96 kHz so θ can be swept at audio rate (for the quadrant-counter sine patch).

---

## Open Questions (not blocking first implementation)

1. Exact voltage scaling for θ: confirm ±V = ±π/2 mapping against Bela ADC range and SARC-100 CV standard.
2. Pull-pot bias: confirm hardware implementation approach with Forest before coding the engage/disengage detection.
3. Attenuverter center-detent behavior: confirm whether pots have mechanical detent at unity or just a nominal center position.
