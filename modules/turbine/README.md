# turbine

Bidirectional phase accumulator for Bela Gem Multi — edit on Mac in Cursor, deploy with `bela-tools`. See `CLAUDE.md` for the full design (seed doc).

## Signal flow (per sample)

1. `exp_cv = bias + atten * expSignal`, `lin_cv = bias + atten * linSignal` — pull-pot bias summed with attenuverted patched signal; both bipolar, both read at audio rate (through-zero FM is meant to work with a hot audio-rate signal)
2. `exp_rate = sign(exp_cv) * exp(k * |exp_cv|)` (true zero at exp_cv==0 — not a standard 1V/oct converter), `lin_rate = k_lin * lin_cv`
3. SYNC (edge) resolves before FREEZE (level) in the same sample: SYNC snaps `phi = 0`; FREEZE then skips the integration step entirely (inputs stay live, only the accumulate is suspended)
4. `phi += (exp_rate + lin_rate) * dt`, then wrapped to `[0, 2pi)` unconditionally
5. SIN/COS/RAMP/RAMP⊥/Ramp×4/Ramp×2 and the EVEN/HALF gates are all pure functions of the *current* `phi` register — read fresh every sample, not from an edge-counted history, so they stay correct under reversal

## Controls (GUI — dev; map to CV + panel later)

| Control | Role |
|---------|------|
| Exp FM Bias / Attenuverter | Pull-pot + attenuverter for the through-zero exponential path |
| Lin FM Bias / Attenuverter | Same, for the linear path |
| Exp FM / Lin FM IN (A0/A1) | Read-only meters — raw patched signal, remapped to bipolar -1..1 |
| Exp CV / Lin CV (eff) | Read-only meters — bias + atten*signal, what actually drives each rate term |
| Rate Hz (eff) | Read-only meter — combined rate converted to an approximate Hz for diagnostics |
| Sin / Cos / Ramp / Ramp Perp / Ramp x4 / Ramp x2 / Even / Half (eff) | Read-only output meters (aux task → GUI) |

No SYNC/FREEZE meters — both are plain digital pins, watch them directly.

## I/O (placeholder — see `pins.h` and `CLAUDE.md`)

| Jack | Signal |
|------|--------|
| Analog in 0 | EXP FM (patched signal, pre attenuverter/bias) |
| Analog in 1 | LIN FM (patched signal, pre attenuverter/bias) |
| Audio out 0 | SIN — AC-coupled, bipolar, written directly |
| Audio out 1 | COS — AC-coupled, bipolar, written directly |
| Audio out 2 | RAMP — DC-coupled, unipolar [0,1) |
| Audio out 3 | RAMP⊥ — DC-coupled, unipolar [0,1), phase + 90° |
| Audio out 4 | Ramp ×4 — DC-coupled, unipolar [0,1), `phi mod 90°` |
| Audio out 5 | Ramp ×2 — DC-coupled, bipolar, `phi mod 180°`, remapped to 0..1 for the DC write |
| Digital in 0 | SYNC — edge-triggered, snaps phi to 0 |
| Digital in 1 | FREEZE — level-sensitive, suspends integration while high |
| Digital out 2 | EVEN gate — quadrant-index parity |
| Digital out 3 | HALF gate — rotation-half bit |

CLAUDE.md says "analog conversion deferred" — no real panel exists yet, so every index above is an unconfirmed placeholder, same status as spline's were before servo-style hardware confirmation.

## Internal constants (not on the panel, not CV-controllable)

`kExpFmK` (EXP FM steepness) and `kLinFmK` (LIN FM scale) in `render.cpp` — CLAUDE.md's 16-element panel list has no control for either, so these are fixed. Picked so both paths reach a comparable maximum rate (~3.5kHz) at full attenuverter+bias swing; untested on hardware, tune by ear.

## Status

New module — GUI-controllable, builds clean, not yet deployed to hardware (verify with `bela sync -n turbine-verify` / `bela build -n turbine-verify` once Bela is reachable). No dedicated desktop test — the phase/quadrant math is simple enough and tightly coupled to this module's own I/O that it wasn't split into a `lib/dsp/` unit (unlike PID/Spline/Envelope/Confidence).

## Deploy

```bash
bela deploy ~/scripts/SARC-100/modules/turbine
```

Or Cursor: **Run Task → Bela: Deploy**
