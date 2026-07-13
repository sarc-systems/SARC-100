#pragma once

// Bela jack/channel assignments for SARC Spline.
// Single source of truth for I/O mapping — keep in sync with the I/O list in CLAUDE.md.
//
// No pin numbers were specified when this module was designed, so this uses the full
// standard 0-7 analog range (6 segments + SCAN + INTERPOLATE = exactly 8, no wing-off header
// needed) and follows EFFECTOR's confirmed-DC-coupled audio-out pattern from servo for the
// two outputs. UNCONFIRMED — flag if a different mapping is wanted before wiring.

// Analog in (CV) — 6 spline segments + SCAN + INTERPOLATE
#define ANALOG_SEGMENT1_CV    0
#define ANALOG_SEGMENT2_CV    1
#define ANALOG_SEGMENT3_CV    2
#define ANALOG_SEGMENT4_CV    3
#define ANALOG_SEGMENT5_CV    4
#define ANALOG_SEGMENT6_CV    5
#define ANALOG_SCAN_CV        6
#define ANALOG_INTERPOLATE_CV 7

// Audio out — DC-coupled (Gem Multi audio outs 2-9), carrying scanned/slow CV rather than audio.
#define AUDIO_OUT_OUT           3
#define AUDIO_OUT_SCAN_POSITION 2
