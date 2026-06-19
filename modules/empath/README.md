# empath

Dual-ladder harmonic resonator for Bela Gem Multi — edit on Mac in Cursor, deploy with `bela-tools`.

## Spectra (ladders)

Two **node ladders** (7 rungs each), ratios: ½, 1, ³⁄₂, 2, ⁵⁄₂, 3, ⁷⁄₂ of each ladder's F0:

- **Ladder A** (audio out L): detuned **down** from center `F0`
- **Ladder B** (audio out R): detuned **up** from center `F0`, log-shaped to ±10% max
- Ladders always run with independent phases (detune floored at `FLT_MIN` when the slider is at zero)

## I/O (current wiring)

| Jack | Path |
|------|------|
| **Audio in 0 / 1** | **All in** A / B — uniform drive on every node |
| **Audio out 0 / 1** | **Scan out** A / B — crossfade between adjacent rungs |
| **Analog in 0 / 1** | **Output A/B Scan** CV (0–1, summed with GUI) |
| **Analog in 2** | **Tune** CV (0–1 → 0–1024 Hz offset), audio-rate smoothed |
| **Analog in 3** | **Detune** CV (0–1, summed with GUI detune slider) |
| **Analog in 4 / 5** | **Node Coupling A/B** CV (0–1 → 0–2, summed with GUI) |
| **Analog in 6** | **X-Couple Amount** CV (0–1) — total cross-coupling budget |
| **Analog in 7** | **X-Couple Symmetry** CV (0–1) — splits Amount: 0 = all A→B, 1 = all B→A |
| **SYNC** (digital in 0) | Rising edge resets **both** ladders |

Scan in on dedicated jacks is planned; **Input A/B Scan** CV is reserved for that.

## Signal flow (per frame)

1. **Tune CV** read/smoothed at audio rate (faster than block rate) and feeds oscillator phase increment directly; filter coefficients stay block-rate
2. **All in** on audio per ladder
3. Per node: **Node Coupling** — weighted sum of neighbor `oscOut` (prev sample) into bandpass input; bandpass → envelope → sine
4. **Ladder Coupling** — cross-ladder envelope diffusion (ratio-weighted, smoothed)
5. **Cross-ladder frequency pulling** — Adler/Kuramoto-style phase coupling between corresponding A/B nodes (`Freq Lock Depth` + detune-scaled pull), keeps ladders near the sync/drift boundary
6. **Energy budget** — per-channel reserve constrains total envelope energy, replenished from input signal
7. **Drift LFOs** — currently disabled (`EMPATH_DRIFT_ENABLED 0`)
8. **Scan out** on main audio outputs

**Absolute-pitch energy scaling:** a lower F0 makes physically larger resonators harder to excite and slower to decay (piano bass-string analogy); `Pitch Excite Depth` / `Pitch Decay Depth` dial this in independently (0 = no effect).

## Controls

GUI knobs (no CV input):

| Control | Role |
|--------|------|
| Osc Phase | Global phase offset [0, 2π] |
| Node Env Attack / Decay | Envelope shape |
| Freq Lock Depth | Cross-ladder phase-pull strength [0, 0.5] |
| Pitch Excite Depth | Absolute-pitch excitation scaling [0, 1] |
| Pitch Decay Depth | Absolute-pitch decay scaling [0, 1] |

CV-driven (jack only; GUI shows the resulting effective value, read-only):

| Control | Role |
|--------|------|
| F0 / Detune | Center frequency (Hz) / ladder spread, A/B = F0×(1∓d) |
| Node Coupling A/B | Intra-ladder signal sum into bandpass [0, 2] |
| X-Couple Amount / Symmetry | Cross-ladder envelope diffusion budget + direction split |
| Output A/B Scan | Scan-out position on main audio outputs |

Meters (aux task → GUI): In/Out A/B Peak (1 = clip).

Internal: energy reserve, drift depth, coupling memory (`render.cpp`).

## Deploy

```bash
bela deploy ~/scripts/SARC-100/modules/empath
```

Or Cursor: **Run Task → Bela: Deploy**
