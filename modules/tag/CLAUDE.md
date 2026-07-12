# TAG — Module Guide
## SARC-100 / Serge Low-Gain Format / Bela Gem Multi

This began as a pre-build spec; it now documents the **implemented** module. Physics, I/O,
and parameters below match `render.cpp` / `pins.h` as built. Design intent (concept, game
rules, patch ideas) is preserved.

---

## Concept

Three agents move in a bounded 2D disc under simple Newtonian physics. One agent is always
IT. IT chases its nearest eligible neighbor; non-IT agents flee IT. When IT closes within
the catch radius, a **tag** transfers IT to that agent. The three position outputs are
coupled, non-linear, continuously evolving CVs. Because SPEED now spans slow-LFO to
**audio rate**, those outputs work as modulation sources *or* audio oscillators.

Deterministic given initial conditions, but sensitive to parameters, gates, and patched
feedback. Not a random source.

---

## Game Rules

- **Tag transfer**: IT accelerates toward the nearest eligible non-IT player; when distance
  drops below `kCatchRadius`, that player becomes IT and the old IT becomes non-IT.
- **No-tag-backs**: a newly tagged IT may not target the player who just tagged them until
  that player has retreated to ≥ the current distance to the third player. Dynamic,
  three-body geometry; no panel control.
- **IT on startup**: Player 1 is IT at cold start. IT state persists through RESET.
- **Boundary**: a **reflecting** wall at `r_max` — players bounce off it (elastic, with
  restitution), backed by a soft containment gradient in the interior shell. No reset on
  contact; RESET is manual only. (This replaced the earlier soft-wall-spring + WALL-HIT +
  reset-on-wall scheme.)

---

## Physics (as implemented)

Per player, per audio sample — semi-implicit (symplectic) Euler, unit mass. "Speed" is an
acceleration magnitude, not a velocity.

1. **Steering** (constant-magnitude thrust, magnitude = SPEED accel):
   - **IT**: direct pursuit — unit vector straight toward the target. (No tangential term,
     so IT keeps closing and tagging; the tag/role-swap is what perturbs the system out of
     limit cycles.)
   - **Non-IT**: flee IT, with a **tangential (orbit) blend** `kTangential` rotating the
     flee direction toward the perpendicular so fleers curve *around* IT instead of running
     straight to the wall and pinning. Flee-only.
2. **Viscous damping**: `a -= DAMPING · v` (linear drag; the dissipation term).
3. **Containment gradient**: inward radial force, zero inside `kWallStart·r_max`, ramping
   ∝ g² to `kWallStrength` at `r_max`. Holds low-SPEED players in soft orbits; high-SPEED
   players overpower it and reach the reflecting wall. So SPEED sweeps smoothly from soft
   orbits → hard audio-rate bouncing.
4. **Integrate velocity**, then **clamp** its magnitude to `kVMax` (integration stability).
5. **Integrate position**.
6. **Reflecting boundary**: if `r > r_max`, project back onto the circle and invert the
   outward radial velocity (× `kRestitution`, keeping the tangential component). Contains at
   any speed without huge forces.

**Momentum**: yes — velocity is persistent state; players coast and carry inertia.
**Overshoot**: yes — constant-magnitude steering means IT sails *past* its target and
orbits rather than settling; `DAMPING` is the overshoot control (low = underdamped,
oscillatory; high = overdamped, envelope-like).

### Outputs per player (2 channels), polar or cartesian per the POLAR/CART switch
- **Polar** (switch low): `r = |pos| / r_max` clamped [0,1]; `θ = atan2(y,x)` mapped [0,1]
  over [−π,π]. θ is a phasor — it wraps 0↔1 at the −x axis (topologically unavoidable for a
  single-CV angle; reads as a sawtooth when a player circulates).
- **Cartesian** (switch high): `x, y ∈ [−1,1]` mapped to `0.5 + 0.5·(x|y)`.
- GUI meters always show polar regardless of the switch (fixed slider labels).

---

## Parameters (`render.cpp` constants)

| Constant | Value | Meaning |
|---|---|---|
| `kRMax` | 1.0 | boundary radius |
| `kRStart` | 0.5 | spawn radius (players 120° apart) |
| `kCatchRadius` | 0.15 | tag distance threshold |
| `kSpeedMin` / `kSpeedMax` | 0.5 / 6000 | SPEED CV → accel, **exponential** (LFO → audio rate) |
| `kMaxDamping` | 2.5 | DAMPING CV → drag, linear |
| `kTangential` | 0.15 | flee orbit amount (flee-only; 0 = pure radial) |
| `kWallStart` / `kWallStrength` | 0.75 / 60 | containment-gradient onset / strength |
| `kRestitution` | 0.8 | radial velocity kept on a wall bounce (<1 sheds energy) |
| `kVMax` | 24000 | velocity-magnitude clamp (stability) |

---

## Controls — CV only

No GUI control sliders and no in-code bias: each analog input is a single smoothed CV read.
The hardware front-end sums each input's **att + bias pots** into the CV before the ADC (the
established SARC convention), so the panel still has those knobs — they just don't appear in
code. The GUI shows **display-only** meters (Speed eff ×3, Damping eff, r eff / θ eff ×3, IT).

---

## Analog I/O (`pins.h`)

CV inputs (per-player SPEED + global DAMPING):

| Signal | Analog in |
|---|---|
| DAMPING | A0 |
| P1 SPEED | A1 |
| P2 SPEED | A2 |
| P3 SPEED | A3 |

Outputs — DC-coupled audio outs (bipolar via the output stage). Channel A carries **r** (or
**x** in cartesian); channel B carries **θ** (or **y**):

| Player | A (r/x) | B (θ/y) |
|---|---|---|
| P1 | 2 | 5 |
| P2 | 3 | 6 |
| P3 | 4 | 7 |

---

## Digital I/O (`pins.h`)

### Outputs
| Pin | Signal | Type | Notes |
|---|---|---|---|
| 0 | TAG EVENT | Trigger | fires on each IT transfer |
| 1 | IT 1 | Gate | high while Player 1 is IT |
| 2 | IT 2 | Gate | high while Player 2 is IT |
| 3 | IT 3 | Gate | high while Player 3 is IT (redundant = NOT(IT1∨IT2), for patching) |

### Inputs
| Pin | Signal | Type | Notes |
|---|---|---|---|
| 8 | RESET | Trigger | rising edge reinitializes positions; preserves IT state |
| 9 | FREEZE | Gate | zeroes velocities + holds positions while high (instant sample-hold of the output CVs) |
| 4 | POLAR/CART | Switch | high = cartesian (x,y); low = polar (r,θ) |

The IT gates are mutually exclusive — exactly one is always high.

---

## Panel Layout (condensing to 3 player columns + global)

WALL-HIT outputs are gone (4 slots freed). Per-player column: **r out, θ out, IT out,
SPEED in, SPEED att, SPEED bias**. Global column: **TAG out (+LED), DAMPING in, DAMPING att,
DAMPING bias, FREEZE, RESET, POLAR/CART switch**. Exact physical arrangement is being
finalized on the panel; this is the element inventory.

---

## Patch Examples

- **Coupled filter sweep**: P1 r → filter 1 cutoff, P2 r → filter 2, P3 r → filter 3 — three
  coupled envelopes (low SPEED) or three detuned oscillators (high SPEED).
- **θ as phasors**: P_n θ → anything wanting a rotating ramp; at audio rate they're
  saw-like tones.
- **Rhythm from tags**: TAG EVENT → clock/trigger; IT gates → route/gate three voices by
  who's IT.
- **Perturb the dynamics**: external gate → RESET, or → a SPEED CV, to kick the system.
- **Freeze on instability**: external limiter/envelope gate → FREEZE to hold game state.
- **Cartesian for scanning**: flip POLAR/CART for (x,y) — e.g. drive an X/Y scope or a 2D
  wavetable position.

---

## Implementation Notes / Gotchas

- Physics runs at **audio rate** (per-sample); outputs update every sample.
- Boundary is reflection, **not** reset — don't reintroduce reset-on-wall (it caused the
  respawn-loop the reflection fixed).
- SPEED is **exponential**; the bottom of the knob crawls, the top reaches audio rate. It
  never reaches true zero (exp floors at `kSpeedMin`), so FREEZE is the only true stop/hold.
- Keep `kTangential` on **flee only** — adding it to IT's chase lets IT circle instead of
  tagging, and the loss of tags collapses the system into limit cycles.
- The θ output has an unavoidable 0↔+V wrap (single-CV angle). Use cartesian if you need
  jump-free position.
- Tag detection is a per-sample proximity test; at very high SPEED players can skip past
  each other between samples and miss a tag. Acceptable for now; revisit if it bites.

---

## Open Questions / TODO

- Expose `kTangential` (orbit amount) on a spare analog input as a live "orbit" knob?
- Optional trade: bottom-of-SPEED dead-zone → true stop, to free the FREEZE jack
  (deferred — FREEZE retained for instant sample-hold at any speed).
- Anti-alias the θ phasor wrap / r bounce at the top of the audio-rate range if it's harsh.
- Per-player mass or inter-player collisions (richer physics) — not modeled.
