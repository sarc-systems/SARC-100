# servo

Two-timescale PID leveling/AGC element for Bela Gem Multi — edit on Mac in Cursor, deploy with `bela-tools`. See `CLAUDE.md` for the design.

## Signal flow (per frame)

ERROR, EFF, and CONFIDENCE are three parallel taps off the same core computation — none feeds back into another internally. MAGNI(ref)/MAGNI(sig) are independent of all three.

1. `MAGNI(ref) = envelope(REF)`, `MAGNI(sig) = envelope(SIG)` (`ENV RATE`, own timescale) — exported outputs, not routed to TARGET or ERROR internally
2. `ERROR = TARGET - SIG` — raw deviation, unprocessed (uses the raw jacks, not MAGNI)
3. `EFF = pid.update(ERROR)` — gains driven by `PID Kp` (flat) and `Servo Rate` (scales ki/kd together); switchable uni/bipolar
4. `CONFIDENCE = magnitudeScore(ERROR) * stabilityScore(ERROR)` — requires error to be both small and steady; large-but-steady or small-but-jittery error both read as low confidence; always live, never gated by FREEZE

## Controls (GUI — dev; map to CV + panel later)

| Control | Role |
|---------|------|
| Env Rate | MAGNI(ref)/MAGNI(sig) envelope-follower timescale — independent of Servo Rate |
| Servo Rate | PID loop timescale — scales ki/kd together (not kp) |
| PID Kp | Proportional gain (flat, not scaled by Servo Rate) |
| Effector Bipolar | Output range -1..1 vs 0..1 |
| Reset Mode | 0 = hard reset (neutral), 1 = bumpless hold-and-continue |
| Effector / Error / Confidence / Magni Ref / Magni Sig (eff) | Read-only meters (aux task → GUI) |
| Env Rate / Servo Rate / Target IN (A2-A4) | Read-only meters showing the raw CV at each jack |

FREEZE (digital in, level-sensitive) holds only the PID/EFF stage at its current value — ERROR, MAGNI(ref), MAGNI(sig), and CONFIDENCE keep updating live off REF/SIG/TARGET regardless. RESET (digital in, rising edge) clears the PID integrator only; Reset Mode controls whether EFF snaps to neutral or holds bumplessly. CONFIDENCE is not cleared by RESET — it's a standalone observation of ERROR's behavior, decoupled from the PID/EFF stage.

Removed in the 2026-06-21 redesign: the internal SPLINE/TENSION target-curve (TARGET is now its own independent input — patch MAGNI(ref) into it externally if you want the old ref-tracks-target behavior), GOVERNOR TIME MULT (CONFIDENCE's own smoothing is now a fixed internal constant, not CV-exposed), and EFFECTOR INVERT (not in the new panel's 24-element count).

## I/O (confirmed + assumed — see `pins.h`)

| Jack | Signal |
|------|--------|
| Analog in 0 | TARGET |
| Analog in 1 | REF |
| Analog in 2 | SIG |
| Analog in 3 | ENV RATE CV (summed with GUI) |
| Analog in 4 | SERVO RATE CV (summed with GUI) |
| Audio out 2 | EFFECTOR — confirmed DC-coupled |
| Audio out 3 | ERROR — **assumption**, not yet confirmed |
| Audio out 4 | CONFIDENCE — **assumption**, not yet confirmed |
| Audio out 5 | MAGNI(ref) — **assumption**, not yet confirmed |
| Audio out 6 | MAGNI(sig) — **assumption**, not yet confirmed |

Not yet wired at all: FREEZE/RESET digital pins, RESET MODE / EFF UNI-BIPOLAR panel switches (GUI toggles stand in for both — see `pins.h` and `CLAUDE.md`).

## Status

TARGET/REF/SIG/ENV RATE/SERVO RATE ins (A0-A4) and EFFECTOR out (audio out 2) are hardware-confirmed. The four new audio outs (ERROR/CONFIDENCE/MAGNI ref/sig, outs 3-6) are code-side placeholders pending confirmation. `lib/dsp/` (envelope, PID, confidence) is shared, Bela-agnostic, desktop-tested in `test/` — `spline.h` is no longer used by servo; it was generalized and is now owned by `modules/spline/`.

## Deploy

```bash
bela deploy ~/scripts/SARC-100/modules/servo
```

Or Cursor: **Run Task → Bela: Deploy**
