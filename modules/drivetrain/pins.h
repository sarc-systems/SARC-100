#pragma once

// Bela jack/channel assignments for DRIVETRAIN.
// Single source of truth for I/O mapping — keep in sync with §1 of the module spec
// (modules/drivetrain/CLAUDE.md). Panel is locked: 8 analog in, 6 analog out
// (2 spare), 2 digital in.

// ── Analog in (8/8) ─────────────────────────────────────────────────────────
#define AN_IN_TUNE        5   // master frequency (exp)
#define AN_IN_RATIO_A     4   // sub A rational ratio (Stern-Brocot value axis)
#define AN_IN_RATIO_B     7   // sub B rational ratio
#define AN_IN_COMPLEXITY  6   // Stern-Brocot max depth (bias+atten do limit-shaping)
#define AN_IN_PM_DEPTH    1   // total PM index (exp)
#define AN_IN_PM_BALANCE  2   // A/B modulator split (bipolar, encoded [0,1])
#define AN_IN_PHASE_A     0   // sub A phase offset (static)
#define AN_IN_PHASE_B     3   // sub B phase offset (static)

// ── Outputs — DC-coupled AUDIO outs on the Gem Multi (written with audioWrite,
//    NOT analogWrite). Audio outs 2–9 are the DC-coupled ones; we use 2–7 for the
//    six voices, matching turbine. Confirm these channels against the panel wiring.
#define AUDIO_OUT_SINE    3   // PM'd master carrier: sin(2pi*phi_m + PM)
#define AUDIO_OUT_COSINE  4   // PM'd master carrier: cos(2pi*phi_m + PM)
#define AUDIO_OUT_SINE_A  2   // sin(2pi*phi_A) — rotated sub A modulator, exposed
#define AUDIO_OUT_SINE_B  5   // sin(2pi*phi_B)
#define AUDIO_OUT_RAMP_A  6   // phi_A — rotated sub ramp (raster source)
#define AUDIO_OUT_RAMP_B  7   // phi_B
// audio outs 0, 1 unused; 8, 9 spare (reserved: self-reporting brightness CV, §11)

// ── Digital in ──────────────────────────────────────────────────────────────
#define DIG_FREEZE        0   // level — high holds the whole geared system
#define DIG_RESET         1   // rising edge — zero master phase + cycle counter
