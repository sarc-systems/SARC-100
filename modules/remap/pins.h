#pragma once

// Bela jack/channel assignments for REMAP.
// Single source of truth for I/O mapping — keep in sync with modules/remap/CLAUDE.md.
//
// ⚠ PLACEHOLDER / DUMMY VALUES (penciled 2026-07-19, panel layout in progress).
// These indices are NOT confirmed against hardware. Every analogRead/audioWrite in
// render.cpp is bounds-guarded, so unassigned/wrong indices degrade rather than crash,
// but CONFIRM against the locked panel wiring before deploying.

// ── Analog in (2/8) ──────────────────────────────────────────────────────────
// X and θ each: jack + attenuverter + bias (analog front-end, house style). ADC reads
// unipolar [0,1]; render expands to bipolar volts [-V,+V] (remap::expandBipolar).
// Read paths differ by type (see CLAUDE.md), matching servo:
//   X = signal   -> RAW per-frame analogRead (may be audio-rate; no smoothing).
//   θ = param    -> block-rate CvIn (smoothed) to de-zipper ADC/pot noise.
#define ANALOG_X       0   // X in  — signal, bipolar [-V,+V]
#define ANALOG_THETA   1   // θ in  — translation parameter, bipolar [-V,+V]

// ── Outputs (7) — DC-coupled AUDIO outs, written with audioWrite, NOT analogWrite ────
// On the Gem Multi the DC-coupled outputs are audio outs 2–9 (analogWrite segfaults —
// see drivetrain notes). The DC output stage produces the BIPOLAR swing at the jack, so
// render writes a normalized [0,1] value (remap::mapOutput / mapBipolarOutput).
// Seven outputs -> audio outs 2–8 (one DC-coupled out, ch 9, spare).
#define AUDIO_OUT_ROTATE1  2
#define AUDIO_OUT_ROTATE2  3
#define AUDIO_OUT_MIRROR1  4
#define AUDIO_OUT_MIRROR2  5
#define AUDIO_OUT_FOLD1    6
#define AUDIO_OUT_FOLD2    7
#define AUDIO_OUT_THETA    8   // conditioned/held θ, bipolar [-V,+V]

// ── Digital in (1) ───────────────────────────────────────────────────────────
#define DIGITAL_FREEZE_THETA   0   // gate (level): while high, hold conditioned θ (S&H).
                                   // Transforms keep updating live off X with the held θ.
