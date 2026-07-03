# DRIVETRAIN

Master phasor + two exact-rational sub-oscillators with 3-operator phase
modulation, for Bela Gem Multi.

One TUNE-tuned master phase accumulator is the shaft. Two sub phases are computed
as exact rational multiples `p/q` of the master by cycle-counting — rigid integer
gearing, no PLL, no slip. The master sine/cosine are phase-modulated by the two
subs (equal-power depth/balance split); the subs are also exposed raw as sine + ramp.
Ratios are chosen from the Stern-Brocot tree, COMPLEXITY setting the depth ceiling,
so just-intonation relationships hold by construction and a fixed RATIO stays put as
complexity rises.

- **I/O:** 8 analog in, 6 analog out (2 spare), 2 digital in — see `pins.h`.
- **Design contract & gotchas:** `CLAUDE.md`.
- **Shared DSP:** `lib/dsp/cycledivide.h`, `lib/dsp/sternbrocot.h`, `lib/io/cv_input.h`.
- **Tests:** `test/test_drivetrain_dsp.cpp` (run `test/run.sh`).

The critical invariant: phase state advances from TUNE alone; PM is additive at the
output read only. No feedback, ever. See `CLAUDE.md` §3.
