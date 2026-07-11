# DRIVETRAIN — module guide for Claude Code

Orients an agent working in `modules/drivetrain/`. Operational. The full design
rationale is the spec handoff (kept verbatim below from §3 onward); this top matter
is *what* and *where*.

## What DRIVETRAIN is
A single **master** phase accumulator (the shaft, TUNE-tuned) with two **sub**
phases computed as exact rational multiples `p/q` of the master via cycle-counting.
The master sine/cosine are **phase-modulated** by the two subs (3-operator PM: one
carrier, two modulators) with depth/balance and an equal-power split. Subs are also
exposed raw (sine + ramp). Synthesis of the ROTOR concept (master + phase-locked
subs) and GEARBOX (programmable rational division) — but division is **exact
computed gearing**, not a PLL: no slip, no settling.

Single Bela Gem Multi. 8 analog in / 6 analog out (2 spare). 2 digital in.

## Current build deviations from the spec below
- **Outputs use `audioWrite` to DC-coupled audio outs 2–7**, not `analogWrite` (see Bela
  notes). `pins.h` analog-in channels were hand-mapped to the panel wiring.
- **The JI lattice is fixed at 9-odd-limit / 7-prime-limit** (`kOddLimit = 9`,
  `kPrimeLimit = 7`, passed to `gTable.build`) — 77 ratios over [0.25, 4],
  the 9-odd-limit tonality diamond (includes 9/8, 10/9, 8/7, 7/6, 6/5, 9/7, 9/5, 16/9…).
  Odd-limit (not tree depth) gates density, because tree depth over-penalizes near-unison
  ratios — 9/8 is tree-depth 9. The table is built to depth `D_MAX = 14` so the limits, not
  depth, do the gating. Replaces the spec's "no prime-limit control" note. `lib/dsp/
  sternbrocot.h` gained optional `primeLimit` and `oddLimit` args.
- **The COMPLEXITY input (`AN_IN_FUND_NULL`) now drives a fundamental-null control**, not
  depth. It sets `gFundNull ∈ [0,1]` = the depth of a **steep notch (RBJ, Q=`kNotchQ`)
  tracking f_master**, blended onto SINE/COSINE (0 = dry, 1 = full notch). The notch runs
  every sample to stay warm; coeffs recompute per block as TUNE moves. Chosen over the
  earlier J0-subtraction (which couldn't handle subharmonic ratios). Trade-off: it also
  dents sidebands within ~f_master/Q of the fundamental. `Biquad` is a local struct.

## Where the code lives
- `render.cpp` — the Bela loop: control-rate CV reads, the master phase engine,
  per-sample sub reconstruction + PM, output writes, display meters. Module-specific
  wiring only.
- `lib/dsp/cycledivide.h` — exact cycle-counted rational phase division (§5).
  Bela-agnostic, unit-tested.
- `lib/dsp/sternbrocot.h` — Stern-Brocot table + ratio selection (§6). Bela-agnostic,
  unit-tested.
- `lib/io/cv_input.h` — `CvIn`: the **standardized smoothed analog CV read** used by
  every control here (and the intended standard for all modules). Wraps `analogRead`
  with a one-pole `Smoother` (`lib/dsp/smooth.h`) at the 10 ms `kCvSmoothTimeConstS`.
- `pins.h` — I/O map (§1), single source of truth.
- `test/test_drivetrain_dsp.cpp` — desktop validation of the two correctness-critical
  units: phase continuity across master wraps, exact period, and Stern-Brocot
  value-sorting / nesting / persistence. Run `test/run.sh`.

## Things that will bite (read before editing)
- **The critical invariant (§3) is not negotiable.** Phase state advances from TUNE
  alone; everything else is a pure function downstream. PM is additive at the output
  read ONLY. Never fold PM into the accumulator, never derive subs from the PM'd
  output, never cross-PM. Those are the unstable topologies this design removes.
- **Sub phase must use `cycledivide`, never `frac((p/q)·φ_master)`.** The naive form
  resets the sub every master cycle → a phase discontinuity at each wrap (the
  CAMSHAFT delta-integrator bug class).
- **Latch `(p,q)` at the master wrap only** (and on RESET). Changing the ratio
  mid-cycle scales `φ_master` and jumps the sub. `selectRatio()` computes the target
  at control rate; `latchSub()` copies it at the wrap.
- **Address ratios by Stern-Brocot value-coordinate (table index), never by sorted
  sub-list position.** Indexing a depth-filtered list reflows pitch when the set
  resizes. The persistence guarantee (fixed RATIO → same ratio as COMPLEXITY rises)
  depends on this — covered by `test_sternbrocot_persistence`.
- Outputs are bipolar via the DC output stage: sines write `0.5+0.5·x`, ramps write
  `φ` directly. Do **not** AC-couple the outputs — the ramps carry DC.

## Standardized CV read (lib/io/cv_input.h)
All eight controls read through `CvIn` (block-rate, smoothed). This is the shared
pattern TINE previously did inline, now factored into the lib so every module reads
CV identically. New modules should use `CvIn`; existing modules can migrate to it.
For controls needing audio-rate tracking, drive `lib/dsp/Smoother` per-sample instead.

---

## 0. Architecture in one paragraph

A single **master** phase accumulator (TUNE-tuned) is the shaft. Two **sub** phases
are computed as exact rational multiples `p/q` of the master via cycle-counting (not
by scaling the single-cycle sawtooth). The master sine/cosine outputs are
**phase-modulated** by the two subs (3-operator PM: one carrier, two modulators),
with depth/balance control and equal-power split. The subs are also exposed raw
(sine + ramp) as voices and as memory-surface raster sources. Everything downstream
of the master is a **pure function of the clean master phase** — strict DAG, no
feedback (§3).

## 1. Hardware I/O map (fixed) — see pins.h

Analog in: 0 TUNE, 1 RATIO A, 2 RATIO B, 3 COMPLEXITY, 4 PM DEPTH, 5 PM BALANCE,
6 PHASE A, 7 PHASE B.
Analog out: 0 SINE, 1 COSINE, 2 SINE A, 3 SINE B, 4 RAMP A, 5 RAMP B, 6/7 spare.
Digital in: 0 FREEZE (level), 1 RESET (rising edge).

Master ramp is not a dedicated output. Recover it: RATIO A = 1/1 → RAMP A is the
clean master ramp, SINE A the clean master sine (1/1 is the Stern-Brocot root,
always exactly reachable).

## 2. Signal conventions
- Att + pull-pots are analog front-end conditioning, not code. One number per control.
- Inputs unipolar `[0,1]`. Bipolar-in-meaning controls (PM BALANCE, phase offsets)
  are encoded `[0,1]` and expanded in code (`balance = cv*2 − 1`).
- Outputs DC-coupled bipolar audio. Sines: write `0.5+0.5·x`. Ramps: write `φ`
  directly. DC coupling required.

## 3. THE CRITICAL INVARIANT
Phase state advances from TUNE alone. Everything else is a pure function evaluated
downstream. Never write a downstream value into phase state. Strict DAG per sample:
```
φ_master  (state; advanced by TUNE only; held by FREEZE; zeroed by RESET)
   ├─→ φ_A = cycledivide(p_A,q_A, φ_master, C) + offset_A
   ├─→ φ_B = cycledivide(p_B,q_B, φ_master, C) + offset_B
   └─→ out = sin(2π·φ_master + d_A·sin(2π·φ_A) + d_B·sin(2π·φ_B))
```
Forbidden: folding PM into the accumulator; deriving subs from the PM'd output;
cross-PM (A's arg uses B's PM'd output and vice versa). Topology is two modulators →
one carrier, acyclic.

## 4. Master phase engine
State `φ_master ∈ [0,1)`, `C` (int64). Per sample: if !FREEZE advance by `f_master·dt`,
wrap → `C++`; rising RESET → `φ=0, C=0`. FREEZE holds both φ and C (gear train stops
coherently). `f_master` from TUNE (§9).

## 5. Sub phase — exact cycle-counted rational division
Do NOT compute `frac((p/q)·φ_master)`. Absolute master phase `C + φ_master` is
monotonic; exact sub phase = `frac((p/q)·(C + φ_master))`, continuous across master
wraps. `C mod q` keeps the integer term bounded; use `double` for the fraction.
Implemented in `lib/dsp/cycledivide.h`.

## 6. Ratio selection — Stern-Brocot, complexity = depth ceiling
Per sub, map (RATIO cv, COMPLEXITY cv) → `(p,q)`:
1. Depth `D` from COMPLEXITY, hysteretically quantized.
2. Admitted set = Stern-Brocot rationals of depth ≤ D within `[R_MIN,R_MAX]`.
   Membership is nested (depth D+1 ⊃ depth D).
3. Position target = RATIO cv log-mapped onto `[R_MIN,R_MAX]`.
4. Selected = admitted node nearest in log-ratio, with hysteresis.

Persistence guarantee: a fixed RATIO cv selects the same ratio as COMPLEXITY rises —
deeper levels insert nodes only in the gaps. Address by tree value-coordinate (table
index), never by sorted-list index. No prime-limit control (depth-gating admits all
primes at sufficient depth). Latch `(p,q)` at master wrap only. Implemented in
`lib/dsp/sternbrocot.h`.

## 7. Phase modulation — depth/balance, equal-power, sine modulator
```
balance = PM_BALANCE_cv*2 − 1            # [-1,1]
θ       = (balance+1)·π/4                 # [0,π/2]
depth   = index_map(PM_DEPTH_cv)          # exp, [0,MAX_INDEX] rad
d_A = depth·cos(θ);  d_B = depth·sin(θ)   # equal-power: d_A²+d_B² = depth²
arg = 2π·φ_master + d_A·sin(2π·φ_A) + d_B·sin(2π·φ_B)
OUT_SINE = sin(arg);  OUT_COSINE = cos(arg)
```
Modulators are the rotated sub phases (offset baked in). PM indices are free-running
per-sample — they touch only the output read, never phase state; do NOT cycle-quantize
them. Only `(p,q)` latches at wrap.

## 9. Control mapping & calibration (see constants in render.cpp)
```
F_MIN=2.0  OCT_RANGE=12.0   f_master = F_MIN·2^(tune·OCT_RANGE)
MAX_INDEX=8.0  EXP_K=3.5     index_map(cv)=MAX_INDEX·(exp(EXP_K·cv)−1)/(exp(EXP_K)−1)
D_MAX=12                      depth_quantize(cv)=round(1+cv·(D_MAX−1)) w/ hysteresis
R_MIN,R_MAX=0.25,4.0          RATIO cv log-mapped; HYST_RATIO dead-zone in log-ratio
```
COMPLEXITY's analog bias+atten do the limit-shaping that replaced a prime-limit rotary.

## 10. Subs have no independent state
Subs are computed windows onto the master. No per-sub FREEZE/RESET; the only per-sub
control is the static offset (PHASE A/B). Rate/freeze/reset belong to the master and
propagate through the gearing.

## 11. Deferred / flagged
1. Self-reporting brightness/density CV (spare out ch 6) — compute spectral spread
   analytically from depth + ratios, emit as CV so SERVO governs brightness with no
   FFT. Build when wiring DRIVETRAIN↔SERVO.
2. Trigger-resettable PHASE A/B — static offsets this revision.
3. Balance-axis SERVO target — no clean scalar error signal yet; defer.

## 12. System fit
SPLINE sequences depth/balance/ratio. TINE takes the PM'd carrier into the ladder
resonator. FULCRUM rotates the master SINE/COSINE pair (why cos is exposed). SERVO
governs depth via brightness. Degenerate case: RATIO A = 1/1 → clean master sine+ramp.

## Bela / hardware notes
- `settings.json` mirrors TINE's (48k, uniform sample rate).
- **Outputs use `audioWrite` to the Gem Multi's DC-coupled audio outs 2–7, NOT
  `analogWrite`.** The spec §8 pseudocode says `analogWrite`, but on this hardware the
  DC-coupled outputs are audio outs 2–9 (audioWrite); `analogWrite` targets a
  nonexistent/short analog-out buffer and segfaults. Same convention as turbine/tag.
  CV inputs are read with `analogRead` (block-rate, via `CvIn`), which is correct.
- All analog I/O is bounds-guarded (`< audioOutChannels` on writes; `CvIn` guards
  `< analogInChannels` on reads) so a channel-count mismatch degrades instead of crashing.
- Four trig calls/sample (`sinf_neon`/`cosf_neon`); cheap, should hold 48k easily.
- Pin assignments in `pins.h` follow §1 — confirm against the locked panel before wiring.

## Deploy
`bela deploy modules/drivetrain` (or Cursor Run Task). `lib/` is pulled in via the
`lib/...` include path at build time, same as every other module.
