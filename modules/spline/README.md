# spline

6-knot open CV scanner for Bela Gem Multi — edit on Mac in Cursor, deploy with `bela-tools`. See `CLAUDE.md` for the design.

## Signal flow (per frame)

1. 6 **SEGMENT** CVs (GUI slider + CV summed) set the 6 knot values, evenly spaced across an open x = 0..1 curve
2. **SCAN** (CV, audio-rate) sets the read position, 0..1 — clamps at the ends, does NOT wrap
3. **INTERPOLATE** (CV) cross-fades the read character: 0 = discontinuous (nearest-knot hold), 0.5 = linear, 1 = smooth (Catmull-Rom, clamped end knots) — continuous morph, not a switch
4. `OUT` = `Spline<6>::evaluate(scanPosition)`
5. `SCAN POSITION OUT` = the clamped, effective scan position actually used — lets multiple Splines chain to the same scan position regardless of each unit's own SCAN attenuverter/bias

## Controls (GUI — dev; map to CV + panel later)

| Control | Role |
|---------|------|
| Segment 1-6 | Knot values, evenly spaced across the open curve |
| Scan | Read position, 0..1, clamps at the ends |
| Interpolate | Read character: 0 = discontinuous (nearest knot), 0.5 = linear, 1 = smooth — continuous morph |
| Out / Scan Position (eff) | Read-only meters (aux task → GUI) |
| Segment 1-6 / Scan / Interpolate IN (A0-A7) | Read-only meters showing the raw CV at each jack |

## I/O (placeholder — see `pins.h` and `CLAUDE.md`)

| Jack | Signal |
|------|--------|
| Analog in 0-5 | SEGMENT 1-6 |
| Analog in 6 | SCAN |
| Analog in 7 | INTERPOLATE |
| Audio out 2 | OUT |
| Audio out 3 | SCAN POSITION OUT |

No pin numbers were given when this module was designed — this uses the full 0-7 analog range and follows servo's audio-out-2+ DC-coupled pattern. Unconfirmed; flag before wiring a real panel.

## Status

New module — GUI-controllable, builds clean, not yet deployed to hardware (verify with `bela sync -n spline-verify` / `bela build -n spline-verify` once Bela is reachable). `lib/dsp/spline.h` (`Spline<N>`, replacing servo's old unused `Spline5`) is desktop-tested in `test/test_spline_dsp.cpp` (run via `test/run.sh`, which now runs every `test_*.cpp` in the directory).

## Deploy

```bash
bela deploy ~/scripts/SARC-100/modules/spline
```

Or Cursor: **Run Task → Bela: Deploy**
