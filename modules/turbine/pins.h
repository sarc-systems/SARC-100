#pragma once

// Bela jack/channel assignments for SARC Turbine.
// Single source of truth for I/O mapping — keep in sync with the I/O list in CLAUDE.md.
//
// CLAUDE.md says "analog conversion deferred" — no real panel exists yet, so these are
// placeholders like servo/spline's were before hardware confirmation. SIN/COS are genuinely
// audio-rate and bipolar, so they ride the standard AC-coupled stereo pair (0/1) — written
// directly as sin/cos with no remap, matching empath's audio-out convention. Everything else
// is more CV-like (can hold a static value indefinitely under FREEZE) so it rides the
// DC-coupled outs (2+, confirmed DC-coupled on the Gem Multi in empath/servo).

// Analog in (CV) — EXP FM / LIN FM patched signal, before attenuverter+bias (see render.cpp:
// these are remapped from analogRead's raw 0..1 to bipolar -1..1, since through-zero FM needs
// a true bipolar signal, unlike every other module's CVs so far).
#define ANALOG_EXP_FM_CV 0
#define ANALOG_LIN_FM_CV 1

// Audio out — SIN/COS on the AC-coupled pair; RAMP family on DC-coupled outs.
#define AUDIO_OUT_SIN      0
#define AUDIO_OUT_COS      1
#define AUDIO_OUT_RAMP     2
#define AUDIO_OUT_RAMP_PERP 3
#define AUDIO_OUT_RAMP_X4  4
#define AUDIO_OUT_RAMP_X2  5

// Digital
#define DIGITAL_SYNC   0   // in, edge-triggered — snaps phi to 0
#define DIGITAL_FREEZE 1   // in, level-sensitive — suspends integration while high
#define DIGITAL_EVEN   2   // out, gate — quadrant-index parity
#define DIGITAL_HALF   3   // out, gate — rotation-half bit
