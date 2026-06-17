# CLAUDE.md — kosc / SARC Empath

Dual-ladder harmonic resonator for Bela Gem Multi.
Developed in Cursor + SSH Remote; deployed with `bela-tools`.

---

## Project identity

- **Instrument name:** SARC Empath [working title]
- **Repo:** github.com/MoodOrgan/kosc
- **Hardware:** Bela Gem Multi (PocketBeagle 2, quad-core A53 @ 1.4GHz, 10 audio I/O)
- **DAC:** ES9080Q — audio outputs 2–9 can be DC-coupled
- **Dev environment:** Mac + Cursor SSH Remote to Bela; deploy via `bela deploy ~/scripts/kosc` or Run Task → Bela: Deploy

---

## Architecture

### Core concept

Two independent **node ladders** (A and B), each with `NUM_OSCS` nodes tuned to harmonic/subharmonic ratios of a center F0. Default ratios: ⅛, ⅙, ¼, ⅓, ½, 1, ³⁄₂, 2, ⁵⁄₂, 3, ⁷⁄₂.

- **Ladder A** runs at `F0 × (1 - detune)`
- **Ladder B** runs at `F0 × (1 + detune)`
- Detune is log-shaped; floored at `FLT_MIN` so ladders always run with independent phases even at zero detune

### Signal flow (per audio frame)

1. **All-in** — audio inputs drive every node of each ladder uniformly
2. **Node coupling** — each node receives a weighted sum of neighbor `oscOut` values from the previous sample (one-tick delay avoids ordering dependency); weights from `couplingWeights[i][j]` matrix based on harmonic ratio simplicity
3. **Bandpass + envelope** — per-node bandpass filter centered at node frequency; envelope follower on bandpass output; sine oscillator amplitude = envelope value
4. **Ladder coupling** — cross-ladder envelope diffusion weighted by harmonic ratios, smoothed via `kCouplingMemCoeff = 0.997f`; uses sum (additive, same as intra-ladder coupling)
5. **Cross-ladder frequency pulling** — Adler/Kuramoto-style phase coupling between corresponding (same-index) nodes in A and B: `theta[ch][i] += omega[ch][i]*(1+drift) + freqLockDepth*omega[ch][i]*sin(prevTheta[other][i] - theta[ch][i])`. `freqLockDepth` = GUI-only "Freq Lock Depth" slider (0–0.5) + `detune * kDetuneLockDepthScale(5.0)` — detune pushes the ladders apart while simultaneously dialing up the pull trying to hold corresponding nodes together, keeping the system near the sync/drift boundary rather than just drifting apart unopposed. Independent of XCouple Amount/Symmetry. Bounded/self-stabilizing (sin term), unlike envelope coupling. Only valid between corresponding pairs (near-equal frequency via detune); intra-ladder or non-corresponding pairs would need a generalized harmonic locking term (`sin(a·θⱼ - b·θᵢ)`) — not implemented.
6. **Energy budget** — per-channel `gEnergyReserve` (0–1) constrains total envelope energy; replenishes from input signal, spends proportional to coupling activity; prevents runaway
7. **Drift LFOs** — three slow LFOs per channel (0.037, 0.059, 0.043 Hz) modulate node frequencies in harmonic families; irrational ratios prevent locking; currently disabled (`KOSC_DRIFT_ENABLED 0`) for CPU headroom — reversible
8. **Scan out** — equal-power crossfade between adjacent nodes; scan position from CV + GUI

### Coupling matrix

`buildCouplingWeights()` computes `couplingWeights[NUM_OSCS][NUM_OSCS]`:
- Ratio between node frequencies reduced to lowest terms
- Base weight = `1.0f / (num_r + 0.35f * (den_r - 1))`
- Asymmetry: lower-frequency nodes couple into higher ones more strongly (×1.15) than the reverse (×0.8); capped at 0.72 — models upward energy propagation in the harmonic series
- Values below `KOSC_EPSILON` (0.035f) are skipped — sparse matrix, O(N·K) not O(N²)
- `KOSC_LEGACY_COUPLING` flag available for comparison with old behavior

### Energy budget

```cpp
float gEnergyReserve = 0.45f;  // setpoint — natural CV control point
```

Replenishes from `|inputSignal|`; spends on coupling activity. Controls how "alive" the system is. Primary candidate for external CV control.

---

## I/O mapping (current)

| Jack | Signal |
|------|--------|
| Audio in 0 | All-in A |
| Audio in 1 | All-in B |
| Audio out 0 | Scan out A |
| Audio out 1 | Scan out B |
| Analog in 0 | Output scan A CV (0–1, summed with GUI) |
| Analog in 1 | Output scan B CV (0–1, summed with GUI) |
| Analog in 2 | Tune CV (0–1 → 0–1024 Hz offset) |
| Analog in 3 | Detune CV (0–1, summed with GUI detune slider) |
| Analog in 4 | Node Coupling A CV (0–1 → 0–2, summed with GUI) |
| Analog in 5 | Node Coupling B CV (0–1 → 0–2, summed with GUI) |
| Analog in 6 | X-Couple Amount CV (0–1, summed with GUI) — total cross-coupling budget |
| Analog in 7 | X-Couple Symmetry CV (0–1, summed with GUI) — splits Amount between directions: 0 = all A→B, 1 = all B→A, 0.5 = even |
| Digital in 0 | SYNC — rising edge resets both ladders |

**Planned (not yet wired):**
- Analog outs: Node envelope follower CVs (DC-coupled outputs 2–9)
- Digital out 1: Sync out (F0)
- Input scan A/B CV: deferred to IO expander

---

## Panel hardware (Empath Core prototype)

- **Format:** 4U Serge, 4-column Low-Gain boat (100.4mm × 175mm)
- **Jacks:** 7mm banana (Rean NYS230 or equivalent)
- **Pots:** Alpha RV16AF 16mm, 6.35mm shaft, solder lugs
  - CV attenuverters: B10K linear
  - Audio input attenuators: A10K log
- **Knobs:** Davies 1900h clones, 6.35mm shaft
- **Acrylic:** 3mm translucent, laser cut (OMTech Polar 350, 8mm/s 75% power 2 passes)
- **LED:** Bicolor (red/blue) single hole, PWM from digital GPIO — ladder A/B energy indicator

**Panel layout (top to bottom):**
- Row 1: Audio IN A, Sync OUT, Audio OUT A, Audio OUT B
- Row 2: IN A attenuverter, SYNC in, Audio IN B, IN B attenuverter
- Rows 3: Scan A CV (jack/att/offset), TUNE CV (jack/att/offset), DETUNE CV (jack/att/offset), Scan B CV (jack/att/offset)
- Rows 4-5: attenuverters & offset for above
- Rows 6: Node Couple A (jack/att/offset), X-Couple Amount, X-Couple Symmetry, Node Couple B (jack/att/offset)
- Rows 7-8: attenuverters & offset for above

---

## Key constants (render.cpp)

```cpp
constexpr int NUM_OSCS = 11;          // nodes per ladder — soft ceiling, watch N² coupling
constexpr float KOSC_EPSILON = 0.035f; // coupling weight threshold (sparse matrix)
const float kCouplingMemCoeff = 0.997f; // ladder coupling smoothing
float gEnergyReserve = 0.45f;          // energy budget setpoint
// Drift LFO rates (Hz): 0.037, 0.059, 0.043 — irrational ratios, never phase-lock
// Freq Lock Depth (GUI slider, 0-0.5): cross-ladder Adler/Kuramoto phase-pull strength
```

---

## Known issues / active TODOs

- **No allpass in ladder coupling path** — intra-ladder node coupling has allpass phase lag (via VCFQ-style patching in hardware); ladder A↔B coupling path has no phase lag. Add ~π/2 allpass for chimera-supporting inter-ladder behavior.
- **Controls update** - separate node coupling A/B, plus cross-coupling Amount/Symmetry (replacing independent A→B/B→A controls — Symmetry splits the Amount budget between directions, constant-sum, to keep total cross-injected energy bounded), now implemented with CV inputs on ain 4–7. Envelope follower attack/decay remain GUI-only (no panel controls planned).
- **Global phase not on panel** — currently GUI slider only; useful for fine-tuning tone. Reserve an analog input on IO expander.

[later - these will appear on an I/O expansion module]:
- **Input scanning not implemented** — audio inputs currently all-in only. Hardware analog scanner (CD4051 + zero-crossing detect) planned for IO expander.
- **Analog outputs not yet written** — node envelope follower values (`nodeEnvelope[ch][i]`) should be written to DC-coupled analog outputs for CV output and inter-unit coupling. 

---

## Real-time constraints

This runs in Bela's audio callback — **strict real-time rules apply:**

- No heap allocation in `render()`
- No blocking calls (no mutexes, no file I/O, no `printf`)
- GUI metering runs in auxiliary task, not audio thread
- Use `rt_printf()` for debug output if needed (Xenomai-safe)
- Peak hold meters update via `Bela_requestAuxiliaryTask()`

---

## Theoretical context

The instrument implements a **harmonic resonator network** where:
- Coupling topology is determined by number-theoretic relationships (just intonation ratios)
- The coupling matrix is a **harmonic-ratio graph** — neither purely local nor all-to-all
- Cross-ladder coupling with phase lag supports **chimera states** (coexistence of coherent and incoherent regions)
- The system is the fast (Empath) layer of a three-layer SARC architecture:
  - **Empath** (this): audio-rate morphogenetic layer
  - **Set**: analog PID governor / voltage processing
  - **Thoth**: tuple-based memory surface, slow reconfiguration

Multiple Empath units coupled via envelope CV outputs and FM inputs form a **hierarchical multiplex network** capable of exhibiting chimera behavior with as few as 3–4 units.

---

## Deploy

```bash
bela deploy ~/scripts/kosc
```

Or in Cursor: **Run Task → Bela: Deploy**

SSH access: `ssh root@bela.local`
