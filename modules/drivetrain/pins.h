#pragma once

// Bela jack/channel assignments for DRIVETRAIN.
// Single source of truth for I/O mapping — keep in sync with §1 of the module spec
// (modules/drivetrain/CLAUDE.md). Panel is locked: 8 analog in, 6 analog out
// (2 spare), 2 digital in.

// ── Analog in (8/8) ─────────────────────────────────────────────────────────
#define AN_IN_TUNE        0   // master frequency (exp)
#define AN_IN_RATIO_A     1   // sub A rational ratio (Stern-Brocot value axis)
#define AN_IN_RATIO_B     2   // sub B rational ratio
#define AN_IN_COMPLEXITY  3   // Stern-Brocot max depth (bias+atten do limit-shaping)
#define AN_IN_PM_DEPTH    4   // total PM index (exp)
#define AN_IN_PM_BALANCE  5   // A/B modulator split (bipolar, encoded [0,1])
#define AN_IN_PHASE_A     6   // sub A phase offset (static)
#define AN_IN_PHASE_B     7   // sub B phase offset (static)

// ── Analog out (6/8 — 2 spare) ──────────────────────────────────────────────
#define AN_OUT_SINE       0   // PM'd master carrier: sin(2pi*phi_m + PM)
#define AN_OUT_COSINE     1   // PM'd master carrier: cos(2pi*phi_m + PM)
#define AN_OUT_SINE_A     2   // sin(2pi*phi_A) — rotated sub A modulator, exposed
#define AN_OUT_SINE_B     3   // sin(2pi*phi_B)
#define AN_OUT_RAMP_A     4   // phi_A — rotated sub ramp (raster source)
#define AN_OUT_RAMP_B     5   // phi_B
// channels 6, 7 — spare (reserved: self-reporting brightness/density CV, §11)

// ── Digital in ──────────────────────────────────────────────────────────────
#define DIG_FREEZE        0   // level — high holds the whole geared system
#define DIG_RESET         1   // rising edge — zero master phase + cycle counter
