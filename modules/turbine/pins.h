#pragma once

// Bela jack/channel assignments for SARC Turbine.
// Single source of truth for I/O mapping — keep in sync with the I/O list in CLAUDE.md.
//
// CLAUDE.md says "analog conversion deferred" — no real panel exists yet, so these are
// placeholders like servo/spline's were before hardware confirmation. All six outputs ride
// the DC-coupled outs (2+, confirmed DC-coupled on the Gem Multi in empath/servo) — none of
// them use the standard AC-coupled pair (0/1). Bipolar ones (SIN, COS, Ramp x2) get the
// 0.5=0V remap for that write convention (see render.cpp); the unipolar ones (RAMP,
// RAMP-PERP, Ramp x4) write as-is.

// Analog in (CV) — EXP FM / LIN FM patched signal, before attenuverter+bias (see render.cpp:
// these are remapped from analogRead's raw 0..1 to bipolar -1..1, since through-zero FM needs
// a true bipolar signal, unlike every other module's CVs so far).
#define ANALOG_EXP_FM_CV 1
#define ANALOG_LIN_FM_CV 0

// Audio out — all six on DC-coupled outs (see header comment above).
#define AUDIO_OUT_SIN       2
#define AUDIO_OUT_COS       3
#define AUDIO_OUT_RAMP      4
#define AUDIO_OUT_RAMP_PERP 5
#define AUDIO_OUT_RAMP_X4   6
#define AUDIO_OUT_RAMP_X2   7

// Digital
#define DIGITAL_SYNC   0   // in, edge-triggered — snaps phi to 0
#define DIGITAL_FREEZE 1   // in, level-sensitive — suspends integration while high
#define DIGITAL_EVEN   2   // out, gate — quadrant-index parity
#define DIGITAL_HALF   3   // out, gate — rotation-half bit
