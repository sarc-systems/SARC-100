#pragma once

#include <cmath>

// REMAP core: voltage-domain transforms on the periodic coordinate [0, V).
// Plain C++, no Bela dependency — the module (modules/remap) owns I/O, calibration,
// and the V constant; this header owns only the stateless math so it stays unit-
// testable off-device. See modules/remap/CLAUDE.md for the full spec.
//
// Three transforms (ROTATE = translation, MIRROR = reversal, FOLD = reflection) at
// two winding densities (x1, x2). All formulas use FLOORED modulo (wrapf), so results
// are periodic and NEVER clip for any sign/magnitude of X or θ (the translation param).
namespace remap {

// Default internal reference range (single source of truth for the 0..V convention).
// Functions below are parameterized by v so they stay generic/testable; the module
// passes kDefaultV. Do not scatter the literal 5.0 elsewhere.
constexpr float kDefaultV = 5.0f;

// Floored modulo: result always in [0, m) regardless of the sign of x.
// C's fmodf is truncated (keeps the sign of x), so wrap it.
inline float wrapf(float x, float m) {
	float r = fmodf(x, m);
	return (r < 0.0f) ? r + m : r;
}

// Triangular fold of t into [0, range]: rising then falling, periodic.
inline float foldf(float t, float range) {
	return range - fabsf(wrapf(t, 2.0f * range) - range);
}

enum Transform { ROTATE, MIRROR, FOLD };

// One transform at winding density w (1 or 2), full-scale 0..v.
// x2 performs the same operation on the half-interval (v/w) and rescales (* w) to
// full range — i.e. the same shape at twice the spatial frequency.
inline float transform(Transform t, float x, float theta, float v, int w) {
	float range = v / (float)w;
	switch (t) {
		case ROTATE: return (float)w * wrapf(x + theta, range);   // translation
		case MIRROR: return (float)w * wrapf(theta - x, range);   // reversal
		case FOLD:   return (float)w * foldf(x + theta, range);   // reflection (translate before fold)
	}
	return 0.0f;
}

// The six outputs as a compile-time table. Dropping outputs (open question #3 — DAC
// budget) is a table edit only; the DSP above is untouched.
struct OutputSpec { Transform t; int winding; };
constexpr OutputSpec kOutputs[] = {
	{ROTATE, 1}, {ROTATE, 2},
	{MIRROR, 1}, {MIRROR, 2},
	{FOLD,   1}, {FOLD,   2},
};
constexpr int kNumOutputs = sizeof(kOutputs) / sizeof(kOutputs[0]);

// Compute all live outputs into out[kNumOutputs].
inline void computeAll(float x, float theta, float v, float *out) {
	for (int i = 0; i < kNumOutputs; ++i)
		out[i] = transform(kOutputs[i].t, x, theta, v, kOutputs[i].winding);
}

// --- Input conditioning ---------------------------------------------------
// Bela ADC reads unipolar [0,1]; X and θ are bipolar controls (jack + attenuverter +
// bias, analog front-end, house style — see drivetrain §2). Expand the unipolar read
// to bipolar volts [-v, +v]. Matches drivetrain's `cv*2 - 1` convention. No smoothing/
// slew/deadband — REMAP does no hidden processing, and X may be audio-rate.
inline float expandBipolar(float cvUnipolar, float v) { return (cvUnipolar * 2.0f - 1.0f) * v; }

// --- Output mapping -------------------------------------------------------
// Single mapping point per output kind: internal volts -> normalized [0,1] for
// audioWrite. Outputs go to the Gem Multi's DC-coupled AUDIO outs, whose output stage
// produces a BIPOLAR swing at the jack (house convention) — so normalizing an internal
// range to [0,1] yields a bipolar physical output with the range's center at 0 V.
//   Transforms are unipolar [0,V): center V/2 lands at 0 V at the jack.
inline float mapOutput(float outV, float v) { return outV / v; }
//   θ (already bipolar [-v,+v]): 0 V lands at 0 V at the jack.
inline float mapBipolarOutput(float volts, float v) { return (volts / v + 1.0f) * 0.5f; }

} // namespace remap
