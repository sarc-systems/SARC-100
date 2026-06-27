# TAG — Module Specification
## SARC-100 / Serge Low-Gain Format / Bela Gem Multi

---

## Concept

Three agents move in a bounded 2D space under simple physics. One agent is always IT. IT chases its nearest eligible neighbor; non-IT agents flee IT. When IT closes to within catch radius of an eligible target, a tag occurs: the target becomes the new IT, the old IT becomes non-IT. The three position outputs are coupled, non-linear, continuously evolving CVs — at high damping they behave like sluggish interdependent envelopes; at low damping they oscillate and overshoot. The game state (who is IT) determines the coupling topology.

The module is not a random source. Its outputs are deterministic given initial conditions, but sensitive to parameter changes, gate inputs, and patched feedback from the rest of the system.

---

## Game Rules

**Tag transfer**: IT player accelerates toward nearest eligible non-IT player. When distance drops below hardcoded catch radius, that player becomes IT. Old IT immediately becomes non-IT.

**No-tag-backs**: Newly tagged IT may not target the player who just tagged them until that player has retreated to a distance greater than or equal to the current distance to the third player. No panel control — threshold is determined dynamically by the three-body geometry.

**IT on startup**: Player 1 is IT at cold start. IT state persists through RESET — last IT before reset remains IT after reset.

**Boundary**: Soft wall at r_max (hardcoded). Spring-like restoring force activates as player approaches boundary. WALL HIT gate fires when a player reaches r_max. No hardwired behavior on wall hit — patch WALL HIT → RESET or leave unpatched as desired.

---

## Physics

Each player maintains:
- Position: (x, y) in normalized 2D space, center at origin
- Velocity: (vx, vy)

Each tick:
1. Compute target direction (toward nearest eligible neighbor for IT; away from IT for non-IT)
2. Apply acceleration in target direction, scaled by SPEED
3. Apply damping force opposing velocity, scaled by DAMPING
4. Apply soft-wall restoring force if approaching r_max
5. Integrate velocity → position

Output r = sqrt(x² + y²), normalized 0 to +V  
Output θ = atan2(y, x), normalized 0 to +V over -π to +π

---

## Hardcoded Parameters

| Parameter | Value | Notes |
|---|---|---|
| Catch radius | TBD | Distance threshold for tag transfer |
| r_max | TBD | Boundary radius |
| No-tag-backs threshold | Distance to 3rd player | Dynamically determined |

---

## Analog I/O

### Outputs (6 jacks across 3 player columns)

| Jack | Signal | Range |
|---|---|---|
| P1 r | Player 1 radial distance from origin | 0 to +V |
| P1 θ | Player 1 angular position | 0 to +V |
| P2 r | Player 2 radial distance | 0 to +V |
| P2 θ | Player 2 angular position | 0 to +V |
| P3 r | Player 3 radial distance | 0 to +V |
| P3 θ | Player 3 angular position | 0 to +V |

### Inputs (12 jacks across 3 player columns + global column)

| Jack | Function | Notes |
|---|---|---|
| P1 SPEED in | CV for Player 1 speed | Scaled by attenuverter, summed with bias |
| P2 SPEED in | CV for Player 2 speed | |
| P3 SPEED in | CV for Player 3 speed | |
| DAMPING in | CV for global damping | Scaled by attenuverter, summed with bias |

### Panel Pots

| Pot | Function |
|---|---|
| P1 SPEED att | Attenuverter for P1 SPEED CV |
| P1 SPEED bias | Pull-pot, sets resting speed for P1 |
| P2 SPEED att | Attenuverter for P2 SPEED CV |
| P2 SPEED bias | Pull-pot, sets resting speed for P2 |
| P3 SPEED att | Attenuverter for P3 SPEED CV |
| P3 SPEED bias | Pull-pot, sets resting speed for P3 |
| DAMPING att | Attenuverter for DAMPING CV |
| DAMPING bias | Pull-pot, sets resting damping |

---

## Digital I/O (Bela GPIO)

### Outputs

| Pin | Signal | Type | Notes |
|---|---|---|---|
| — | IT 1 | Gate | High while Player 1 is IT |
| — | IT 2 | Gate | High while Player 2 is IT |
| — | IT 3 | Gate | High while Player 3 is IT |
| — | TAG EVENT | Trigger | Fires on each IT transfer |
| — | WALL HIT 1 | Gate | High while Player 1 r >= r_max |
| — | WALL HIT 2 | Gate | High while Player 2 r >= r_max |
| — | WALL HIT 3 | Gate | High while Player 3 r >= r_max |
| — | ANY WALL HIT | Gate | Aggregate of WALL HIT 1/2/3 |

### Inputs

| Pin | Signal | Type | Notes |
|---|---|---|---|
| — | RESET | Gate | Reinitializes all positions; preserves IT state |
| — | FREEZE | Gate | Pauses physics while high; preserves all state |

---

## Panel Layout — 4 Columns × 8 = 32 Slots

### Column 1 — Player 1
1. r out
2. θ out
3. IT gate out
4. WALL HIT out
5. SPEED in
6. SPEED att (knob)
7. SPEED bias (knob)
8. *(spare)*

### Column 2 — Player 2
1. r out
2. θ out
3. IT gate out
4. WALL HIT out
5. SPEED in
6. SPEED att (knob)
7. SPEED bias (knob)
8. *(spare)*

### Column 3 — Player 3
1. r out
2. θ out
3. IT gate out
4. WALL HIT out
5. SPEED in
6. SPEED att (knob)
7. SPEED bias (knob)
8. *(spare)*

### Column 4 — Global
1. TAG EVENT out *(with LED)*
2. ANY WALL HIT out
3. DAMPING in
4. DAMPING att (knob)
5. DAMPING bias (knob)
6. FREEZE in
7. RESET in
8. *(spare)*

---

## Patch Examples

**Tag resets winner to center**  
WALL HIT 1 → RESET (via external logic selecting P1 RESET — or global RESET if full reset is acceptable)

**Score tracking**  
WALL HIT 1 → integrator 1 input  
WALL HIT 2 → integrator 2 input  
WALL HIT 3 → integrator 3 input  
Integrator outputs accumulate a score voltage per player

**Score feeds back into speed**  
Integrator 1 out → P1 SPEED in (high score → faster)

**Poisson stimulation (external)**  
External Poisson gate → RESET or into player SPEED CV to perturb dynamics

**Coupled filter sweep**  
P1 r → filter 1 cutoff  
P2 r → filter 2 cutoff  
P3 r → filter 3 cutoff  
Three coupled envelopes driving spectral content

**Freeze on network instability**  
External limiter gate → FREEZE (hold game state while feedback network stabilizes)

---

## Implementation Notes

- Coordinate system: normalized, center at origin, r_max defines boundary
- Physics update rate: audio rate or control rate TBD during Bela prototype
- No-tag-backs state: single flag per player, cleared when tagger distance ≥ 3rd player distance
- WALL HIT is a gate: high while r >= r_max, low while r < r_max. Position tracks freely beyond r_max (soft-wall spring provides the restoring force); the r output is clamped to 1.0 for the CV out, but the underlying position is not clamped
- TAG EVENT fires as a trigger on IT transfer, not continuous
- IT gates are level outputs (high while IT, low otherwise)
- All three IT gates are mutually exclusive; exactly one is always high
- IT 3 is redundant (IT3 = NOT(IT1 OR IT2)) but provided for patching convenience
- DAMPING: high value = overdamped (smooth, envelope-like); low value = underdamped (oscillatory, wiggly)
- FREEZE input: while high, all velocities and accelerations set to zero; positions held
- RESET: rising edge reinitializes positions to starting configuration (TBD — equally spaced at r_start, or all at center); IT state preserved

---

## Open Questions

- Starting positions on RESET: equally spaced on a circle at r_start, or all at origin?
- r_max and catch radius values: set during Bela prototype tuning
- Physics update rate: audio rate vs. lower control rate
- 4 spare panel slots (1 per column): reserved for noise source or future expansion
