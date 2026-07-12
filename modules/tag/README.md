# tag

Three-agent tag game — coupled 2D physics CVs for Bela Gem Multi. Edit on Mac in Cursor, deploy with `bela-tools`. See `CLAUDE.md` for the full design (seed doc).

## Signal flow (per sample — runs at audio rate)

1. Block-rate CVs (bias knobs + patched signals at unity) set per-player SPEED and global DAMPING
2. Each sample (if not FROZEN):
   - Check no-tag-backs clearing: if gIT's distance to the restricted player ≥ distance to the 3rd player, restriction clears
   - Find IT's eligible chase target (nearest non-IT, skip no-tag-back restricted player if any)
   - Physics: chase (IT) / flee (non-IT) acceleration + viscous drag + soft-wall spring past r_max; integrate velocity → position
   - Tag detection: if IT is within `kCatchRadius` of its target, transfer IT — new IT cannot immediately re-tag the old IT
   - Wall hit detection: rising-edge trigger when a player crosses r_max (single-sample pulse)
3. r = sqrt(x²+y²)/r_max, θ = (atan2(y,x)+π)/(2π) — normalized 0..1, written to DC-coupled audio outs

## Controls (GUI — dev; map to CV + panel later)

| Control | Role |
|---------|------|
| P1/P2/P3 Speed Bias | Pull-pot for each player's chase/flee acceleration — default 0.5 → ~2.5 units/s² → ~1 s to cross the space |
| Damping Bias | Global viscous drag — default 0.5; high = smooth/envelope-like, low = oscillatory/wiggly |
| P1-3 Speed IN (A0-A2) / Damping IN (A3) | Read-only meters — raw patched CV |
| P1-3 r / θ (eff) | Read-only meters — current normalized position outputs |
| IT player (1/2/3) | Read-only meter — which player is currently IT |

## I/O (placeholder — see `pins.h` and `CLAUDE.md`)

| Jack | Signal |
|------|--------|
| Analog in 0-2 | P1/P2/P3 SPEED CV |
| Analog in 3 | DAMPING CV |
| Audio out 2/3 | P1 r / P1 θ |
| Audio out 4/5 | P2 r / P2 θ |
| Audio out 6/7 | P3 r / P3 θ |
| Digital in 0 | RESET — rising edge re-spawns players at r_start × 120° apart; IT preserved |
| Digital in 1 | FREEZE — level, zeros velocities and holds positions while high |
| Digital out 2/3/4 | IT 1/2/3 gates (mutually exclusive, one always high) |
| Digital out 5 | TAG EVENT trigger (single-sample on IT transfer) |
| Digital out 6/7/8 | WALL HIT 1/2/3 gates — high while r >= r_max (position tracks freely past the boundary; r CV out clamped to 1.0) |
| Digital out 9 | ANY WALL HIT aggregate gate |

## Hardcoded constants (tunable — see `render.cpp`)

| Constant | Value | Notes |
|---|---|---|
| `kRMax` | 1.0 | Boundary radius |
| `kRStart` | 0.5 | Spawn radius on RESET |
| `kCatchRadius` | 0.15 | Tag transfer distance threshold |
| `kWallSpring` | 50.0 | Restoring force past r_max (units/s² per unit past wall) |
| `kMaxSpeed` | 5.0 | Maps full bias+cv → this acceleration (units/s²) |
| `kMaxDamping` | 5.0 | Maps full bias+cv → this drag coefficient (s⁻¹) |

CLAUDE.md marks r_max, catch radius as "TBD during Bela prototype tuning" — adjust these once tested on hardware.

## Open questions answered in implementation

- **Starting positions on RESET**: equally spaced 120° apart on a circle at kRStart (origin would give degenerate nearest-neighbor distance at start)
- **Physics update rate**: audio rate — 2D particle physics for 3 agents is trivial at 96kHz

## Status

Initial tests are good. No dedicated desktop test — game/physics logic is tightly coupled to this module's I/O rather than a reusable `lib/dsp/` unit.

## Deploy

```bash
bela deploy ~/scripts/SARC-100/modules/tag
```

Or Cursor: **Run Task → Bela: Deploy**
