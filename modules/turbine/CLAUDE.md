# TURBINE — bidirectional phase accumulator

SARC-100 module. Bela implementation only for now (analog conversion deferred).

## Core model

TURBINE is a single phase accumulator, φ, integrated each sample from a rate
derived from two independent FM paths. There is no internal oscillator
"mode" — φ either wraps (free rotation) or holds (zero rate / FREEZE).
Everything else (rotation, slewing, waveshaping) is left to patching with
BLEND and SERVO, per the system's non-modal design principle.

```
rate = exp_rate + lin_rate
φ(t) += rate(t) * dt
φ wrapped mod 2π every sample (numerical hygiene — prevents float drift
over long runtimes; this wrap is unconditional and not user-facing)
```

## FM paths

**EXP FM** — through-zero exponential, NOT a standard 1V/oct converter.

```
exp_rate = sign(exp_cv) * exp(k * abs(exp_cv))
```

Must hit literal 0 at `exp_cv == 0`. A standard unipolar exponential
converter does not have a true zero (0V = some calibrated reference
frequency, not silence) — that's the wrong circuit here. `exp_cv` is the
pull-pot bias summed with the attenuverted patched signal, per standard
panel grammar.

**LIN FM** — ordinary linear, through-zero by construction (no special
casing needed):

```
lin_rate = k_lin * lin_cv
```

Same bias/attenuverter convention as EXP FM.

Negative rate = accumulator runs backward. Positive = forward. Zero = held
(see "Zero-rate state" below). This applies to the *sum* of both paths —
either path alone can drive direction.

## Transport

**SYNC in** — edge-triggered (rising edge). On trigger, φ snaps to 0
instantaneously. Not level-sensitive, not held. One-shot hard reset.

**FREEZE in** — hard override, not an additive zero-rate trick. While
FREEZE is high, integration is suspended entirely regardless of what's
patched into EXP FM or LIN FM — φ holds at its current value even under a
hot audio-rate FM signal. This is a brake on the accumulator, not a
zeroing of its inputs. Resuming (FREEZE low) continues integration from
the held value — bumpless by construction, since φ was never reset.

**Same-sample ordering**: if SYNC and FREEZE fire in the same sample,
resolve SYNC first (write φ=0), then apply FREEZE (suspend). This gives
"snap to 0 and hold" as a single combined action when both are pulsed
together.

## Outputs

**SIN out** = sin(φ), audio-rate, bipolar.
**COS out** = cos(φ), audio-rate, bipolar.

**RAMP out** = φ itself, as a wrapped sawtooth, unipolar [0, +V].
General-purpose raw phase output — not BLEND-specific (BLEND is served by
the dedicated Ramp ×4 / Ramp ×2 outputs below). Useful for phase-slaving a
second TURBINE (patch into its LIN FM with that unit's own rate at zero)
or driving anything downstream that wants raw phase. Note: being
unipolar, this only ever pushes a slaved TURBINE's phase forward, never
in reverse — revisit as bipolar if that turns out to matter.
**RAMP⊥ out** = same, offset +90° (φ+π/2, wrapped, unipolar). Shares phase
origin with COS, the way RAMP shares phase origin with SIN.

**Ramp ×4 out** = `φ mod 90°`, unipolar [0, +V]. Repeats four times per
full rotation. This is the φ-fold output sized for **BLEND's θ input in
unipolar mode**: BLEND's shaper covers a full crossfade (all-A→all-B) over
one quadrant [0, π/2], so a unipolar quadrant-fold drives it directly.
Use together with **both** EVEN and HALF gate outputs below to
reconstruct a full rotation on BLEND.

**Ramp ×2 out** = `φ mod 180°`, bipolar [−V, +V]. Repeats twice per
rotation. Sized for **BLEND's θ input in bipolar mode**: BLEND's negative-θ
behavior inverts a term, which is the same operation as TURBINE's HALF
bit — so when feeding this output to BLEND, the ramp's own sign already
carries that information. Use together with **EVEN only** (HALF is
redundant in this specific patch).

Both fold outputs are a free byproduct of wrap rate doubling pitch — `Ramp
×2` is a literal octave-up tap on φ, `Ramp ×4` is two octaves up, derived
from the same accumulator with no extra oscillator circuitry.

**EVEN gate out** — high when TURBINE is in an even-indexed quadrant
(0 or 2), low in an odd-indexed quadrant (1 or 3). Toggles every 90°.
Must be read directly from the current φ register state, **not** from an
edge-counted ripple counter — required for correct behavior when φ runs
backward (TURBINE is bidirectional; a counter that only increments on a
positive edge would desync under reversal).

**HALF gate out** — high in the second half of rotation (φ ≥ π, quadrants
2 or 3), low in the first half (quadrants 0 or 1). Toggles every 180°.
Same register-state-read requirement as EVEN.

Together, EVEN + HALF + Ramp ×4 implement standard quarter-wave symmetry
reconstruction of a full sine from one quadrant of shaper. Note that
(HALF, EVEN) read together as a 2-bit pair is literally the quadrant
index (0–3) in binary:

| Quadrant | φ range | EVEN | HALF |
|---|---|---|---|
| 0 | 0–90° | high | low |
| 1 | 90–180° | low | low |
| 2 | 180–270° | high | high |
| 3 | 270–360° | low | high |

## Zero-rate state ("off")

With both pull-pots pulled and nothing patched into either FM jack, net
rate is exactly 0 and φ holds — but this is a **passive** rest state, not
a lock. The accumulator is a free shaft: any CV subsequently patched into
LIN FM (or EXP FM) immediately and proportionally moves it. This is
distinct from FREEZE, which is an active override.

What patching into a zero-rate TURBINE actually does depends on the
timescale of the incoming signal:
- **Audio-rate periodic CV** → ordinary through-zero linear FM with no
  carrier tone (e.g. a repeating ramp gives a repeating 0→X→0 pitch rise;
  a square wave gives a triangle out of SIN/COS).
- **Control-rate / slow / non-periodic CV** → φ genuinely drifts/wanders
  as the running integral of the input (windmill behavior — turns back and
  forth in proportion to a centered wandering signal, net-drifts if the
  signal has DC bias). This is the live integrator case, and it's also a
  novel kind of comparator: EVEN/HALF will flip state every time the
  *accumulated history* of the input crosses a 90°/180°/270° boundary —
  a level-crossing detector on the integral, not on the instantaneous CV.

## Explicitly NOT implemented (intentional, by design)

- **No phase-offset / "phi" input.** Static rotation is BLEND's job
  (RAMP/RAMP⊥ → BLEND θ in). Continuous phase-target tracking/slewing is
  SERVO's job (SERVO with TARGET = moving CV, SIG = RAMP out). TURBINE
  stays a pure rate accumulator.
- **No internal cosine-feedback / self-FM path.** Already achievable
  externally by patching SIN out (or COS out) back into LIN FM — no
  dedicated jack needed.
- **No internal wrap/clamp mode switch for slew-type behavior.** One-shot
  ramp/envelope-like patches are built from SYNC + FREEZE + an external
  comparator watching RAMP out, not a TURBINE feature.
- **No uni/bipolar range switch.** RAMP/RAMP⊥ are fixed unipolar by
  convention (starting point — revisit if the phase-slaving use case wants
  bidirectional push). The two BLEND-fold outputs (Ramp ×4, Ramp ×2) have
  their own fixed, purpose-built ranges as specified above.
- **SYNC mode (edge vs. level) is resolved as edge-triggered.** Decided,
  not configurable. No panel or firmware switch needed.

## Panel — 16 elements total

**Outputs (4)**
1. SIN out (sin φ)
2. COS out (cos φ)
3. RAMP out (φ, unipolar)
4. RAMP⊥ out (φ+90°, unipolar)

**EXP FM (3)**
5. EXP FM in
6. EXP FM attenuverter
7. EXP FM pull-pot (bias)

**LIN FM (3)**
8. LIN FM in
9. LIN FM attenuverter
10. LIN FM pull-pot (bias)

**Transport (2)**
11. SYNC in (edge-triggered, resets φ→0)
12. FREEZE in (hard override, suspends integration)

**Quadrant-state gates (2)**
13. EVEN gate out (quadrant-index parity, register-state read)
14. HALF gate out (rotation-half bit, register-state read)

**BLEND-fold outputs (2)**
15. Ramp ×4 out (φ mod 90°, unipolar — BLEND θ unipolar mode)
16. Ramp ×2 out (φ mod 180°, bipolar — BLEND θ bipolar mode)

No normalled connections, no internal defaults — all routing explicit via
patch cables, per system-wide panel convention. Pull-pots provide hard
disconnect (pulled = 0 contribution), not just a centered value.
