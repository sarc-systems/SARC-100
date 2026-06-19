# servo

Cybernetic PID servo/governor module for Bela Gem Multi — edit on Mac in Cursor, deploy with `bela-tools`. See `CLAUDE.md` for the full design (seed doc).

## Signal flow (per frame)

1. **REF / SIG** envelope-followed independently (`ENV RESPONSE`, own timescale)
2. `target = spline(refEnv)` — 5-knot spline (`SPLINE 1-5`, `TENSION`)
3. `error = target - sigEnv` (optionally inverted)
4. `effector = pid.update(error)` — gains driven by `PID Kp` (flat) and `TIME` (scales ki/kd together)
5. `confidence` — slow leaky integrator on tracking success, rate capped below the loop's own rate via `GOVERNOR TIME MULT`

## Controls (GUI — dev; map to CV + panel later)

| Control | Role |
|---------|------|
| Spline 1-5 | Knot values for the REF→target curve, x = 0, .25, .5, .75, 1 |
| Tension | 0 = smooth (cardinal spline), 1 = linear segments between knots |
| Time | PID loop timescale — scales ki/kd together (not kp) |
| Governor Time Mult | Confidence timescale, always a fraction of Time's rate (provably slower) |
| Env Response | REF/SIG envelope-follower timescale — independent of Time |
| PID Kp | Proportional gain (flat, not scaled by Time) |
| Effector Invert | Flips error sign |
| Effector Bipolar | Output range -1..1 vs 0..1 |
| Reset Mode | 0 = hard reset (neutral), 1 = bumpless hold-and-continue |
| Effector / Error / Confidence (eff) | Read-only meters (aux task → GUI) |

FREEZE (digital in, level-sensitive) holds the PID/confidence state. RESET (digital in, rising edge) clears the integrator and confidence; Reset Mode controls whether the effector snaps to neutral or holds bumplessly.

## Status

All I/O in `pins.h` is placeholder — GUI sliders carry every parameter until real CV/panel wiring is confirmed (see `CLAUDE.md`). `lib/dsp/` (envelope, spline, PID, confidence) is shared, Bela-agnostic — desktop-tested in `test/`.

## Deploy

```bash
bela deploy ~/scripts/SARC-100/modules/servo
```

Or Cursor: **Run Task → Bela: Deploy**
