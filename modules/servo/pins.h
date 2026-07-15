#pragma once

// Bela jack/channel assignments for SARC Servo.
// Single source of truth for I/O mapping — keep in sync with the I/O list
// in CLAUDE.md (the seed design doc for this module).
//
// Confirmed hardware mapping, matching the panel layout (A0-A4 = TARGET, REF, SIG, ENV RATE,
// SERVO RATE in that order). ERROR/CONFIDENCE/MAGNI(ref)/MAGNI(sig) are placed on audio outs
// 3-6, following EFFECTOR's already-confirmed-DC-coupled audio out 2 (Gem Multi audio outs
// 2-9 are DC-coupled) — still an ASSUMPTION, not yet confirmed with hardware. Every
// analogRead/audioWrite call in render.cpp guards against the corresponding
// context->...Channels count, so indices that don't physically exist yet are silently skipped.

// Analog in (CV) — confirmed, board's standard analog ins
#define ANALOG_TARGET_CV      0   // TARGET IN
#define ANALOG_REF_CV         1   // REF IN
#define ANALOG_SIG_CV         2   // SIG IN
#define ANALOG_ENV_RATE_CV    3   // MAGNI(ref)/MAGNI(sig) envelope timescale
#define ANALOG_SERVO_RATE_CV  4   // PID loop timescale

// Audio out — all DC-coupled (Gem Multi audio outs 2-9), carrying slow CV rather than audio.
#define AUDIO_OUT_EFFECTOR    3
#define AUDIO_OUT_ERROR       6   // ASSUMPTION — not yet confirmed
#define AUDIO_OUT_CONFIDENCE  2
#define AUDIO_OUT_MAGNI_REF   4   // ASSUMPTION — not yet confirmed
#define AUDIO_OUT_MAGNI_SIG   5   // ASSUMPTION — not yet confirmed

// Digital
#define DIGITAL_FREEZE 0   // gate — hold while high
#define DIGITAL_RESET  1   // trigger — rising edge
