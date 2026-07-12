#pragma once

// Bela jack/channel assignments for SARC Tag.
// Single source of truth for I/O mapping — keep in sync with CLAUDE.md.
//
// CLAUDE.md says "TBD during Bela prototype" for r_max and catch radius, and does not
// assign specific pin numbers. Placeholder assignments below follow the project convention:
// analog outs on DC-coupled audio outs 2+ (matching tine/servo/turbine), analog ins on
// the standard 0-7 range, digital pins sequential from 0.

// Analog in (CV) — per-player SPEED + global DAMPING.
#define ANALOG_P1_SPEED_CV 1
#define ANALOG_P2_SPEED_CV 2
#define ANALOG_P3_SPEED_CV 3
#define ANALOG_DAMPING_CV  0

// Audio out — DC-coupled, r/θ pairs for each player
#define AUDIO_OUT_P1_R 2
#define AUDIO_OUT_P1_T 5
#define AUDIO_OUT_P2_R 3
#define AUDIO_OUT_P2_T 6
#define AUDIO_OUT_P3_R 4
#define AUDIO_OUT_P3_T 7

// Digital in
#define DIGITAL_RESET      8
#define DIGITAL_FREEZE     9
#define DIGITAL_CARTESIAN  4   // switch: high = cartesian (x,y) outputs, low = polar (r,θ)

// Digital out — IT gates (level), tag trigger (single-sample)
#define DIGITAL_IT1      1
#define DIGITAL_IT2      2
#define DIGITAL_IT3      3
#define DIGITAL_TAG      0
