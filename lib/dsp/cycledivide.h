#pragma once

#include <cmath>

// Exact cycle-counted rational phase division (DRIVETRAIN §5). Plain C++.
//
// Given master phase phiMaster in [0,1) and master cycle count C, returns the
// sub-oscillator phase for ratio p/q. Absolute master phase is (C + phiMaster),
// monotonic; the sub phase is frac((p/q)*(C + phiMaster)). Over q master cycles
// that rises by exactly p — p clean sub-cycles — and it is CONTINUOUS across each
// master wrap (no per-cycle phase discontinuity, unlike frac((p/q)*phiMaster)).
//
// (C mod q) keeps the integer term bounded regardless of runtime. Use double for
// the fractional accumulation.
//
// The 6-arg form takes precomputed invQ = 1/q and pOverQ = p/q so the hot loop
// avoids per-sample division (compute them once when (p,q) is latched). The 4-arg
// convenience computes them — use it in tests / cold paths.

static inline double cd_frac(double x) { return x - std::floor(x); }

inline double cycledivide(int p, int q, long long C, double phiMaster,
                          double invQ, double pOverQ) {
	long long m       = C % q;
	long long intpart = ((long long)p * m) % q;   // integer cycle contribution, bounded
	return cd_frac((double)intpart * invQ + pOverQ * phiMaster);
}

inline double cycledivide(int p, int q, long long C, double phiMaster) {
	return cycledivide(p, q, C, phiMaster, 1.0 / (double)q, (double)p / (double)q);
}
