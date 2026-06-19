#pragma once

// Bela jack/channel assignments for SARC Servo.
// Single source of truth for I/O mapping — keep in sync with the I/O list
// in CLAUDE.md (the seed design doc for this module).
//
// NONE of this is confirmed with hardware yet (see CLAUDE.md's "Bela /
// hardware notes" — several inputs likely need the Gem Multi's wing-off /
// 2x12 header DC-coupled channels, and 11 CV ins + 3 CV outs don't fit in
// the board's 8 standard analog channels anyway). Every index below is a
// PLACEHOLDER: every analogRead/analogWrite/audioRead call in render.cpp
// that uses a non-base channel guards against the corresponding
// context->...Channels count, so indices that don't physically exist yet
// are silently skipped rather than misbehaving. GUI sliders carry every
// parameter in the meantime.

// Audio (REF/SIG may be audio-rate or slow CV — either way they need
// DC-coupling, so they're routed through the wing-off header's DC-coupled
// audio channels, not the base AC-coupled stereo pair on 0/1).
#define AUDIO_IN_REF 2   // TODO: unconfirmed placeholder — needs wing-off / 2x12 header
#define AUDIO_IN_SIG 3   // TODO: unconfirmed placeholder — needs wing-off / 2x12 header

// Analog in (CV) — block-rate parameters, GUI slider + CV summed
#define ANALOG_SPLINE1_CV           0
#define ANALOG_SPLINE2_CV           1
#define ANALOG_SPLINE3_CV           2
#define ANALOG_SPLINE4_CV           3
#define ANALOG_SPLINE5_CV           4
#define ANALOG_TENSION_CV           5
#define ANALOG_TIME_CV               6
#define ANALOG_GOVERNOR_TIME_MULT_CV  7
#define ANALOG_ENV_RESPONSE_CV        8   // TODO: unconfirmed placeholder — index >= 8, needs wing-off header (board's standard analog ins are 0-7)

// Analog out (CV) — DC-coupled, slow CV, not audio
#define ANALOG_OUT_EFFECTOR    0
#define ANALOG_OUT_ERROR       1
#define ANALOG_OUT_CONFIDENCE  2

// Digital
#define DIGITAL_FREEZE 0   // gate — hold while high
#define DIGITAL_RESET  1   // trigger — rising edge
