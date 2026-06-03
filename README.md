# kosc

Dual-spectrum harmonic resonator for Bela Gem Multi — edit on Mac in Cursor, deploy with `bela-tools`.

## Spectra

Two **node lists** (15 nodes each), frequency-ordered from subharmonics through harmonics:

- **Spectrum A** (audio out L): detuned **down** from center `F0`
- **Spectrum B** (audio out R): detuned **up** from center `F0`
- At **Detune = 0**, same frequencies and **phase-locked** oscillators

## I/O (current wiring)

| Jack | Path |
|------|------|
| **Audio in 0 / 1** | **All in** A / B — uniform drive on every node (+ cross-spectrum feedback bus) |
| **Audio out 0 / 1** | **Scan out** A / B — crossfade between adjacent nodes (`Output A/B Scan` CV) |
| **SYNC A / B** | Digital inputs → rising edge resets that spectrum’s phases |

Scan in on dedicated jacks is planned; **Input A/B Scan** CV is reserved for that. **Output A/B Scan** drives main audio outputs.

## Signal flow (per frame)

1. **All in** on audio + cross-spectrum bus per spectrum
2. Per node: uniform drive → bandpass → envelope → sine (slow family-correlated drift)
3. Coupling (**Spread**) through short memory + energy budget
4. **Scan out** on main audio outputs (Output A/B Scan CV)

## Controls (GUI — dev; map to 8 CV + digital SYNC)

| Control | Role |
|--------|------|
| F0 (Hz) | Center frequency |
| Detune | A = F0×(1−d), B = F0×(1+d) |
| Osc Phase | Global offset [0, 2π] |
| Spread | Coupling / diffusion |
| Input A/B Scan | Reserved for future scan-in jacks |
| Output A/B Scan | Scan-out position on main audio outputs |
| A bus → B in / B bus → A in | Cross-spectrum feed [0, 2] |
| Node Env Attack / Decay | Envelope shape |
| In A/B Peak (1=clip) | Main audio input peak hold — **≥ 1.0 ≈ clip** (label turns red via `sketch.js`) |
| Out A/B Peak (1=clip) | Main audio output peak hold — same scale and clip styling |

Internal: energy reserve, drift depth, coupling memory (`render.cpp`).

## Deploy

```bash
bela deploy ~/scripts/kosc
```

Or Cursor: **Run Task → Bela: Deploy**
