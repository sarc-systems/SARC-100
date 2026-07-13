#pragma once

#include <cmath>

// Equal-power, DC-coupled crossfade / rotation. Plain C++, no Bela dependency.
//
// Pure arithmetic (no filtering), so it works on audio or CV of any sign and preserves DC.
// This is the shared primitive behind FULCRUM and any equal-power A/B blend in the system.
//
// equalPowerRotate rotates the (a,b) pair by theta into an orthogonal (out,quad) pair:
//     out  =  a·cosθ + b·sinθ
//     quad = −a·sinθ + b·cosθ
// Energy is conserved for every θ:  out² + quad² = a² + b²  (since cos²+sin² = 1). Feeding
// a=A, b=B and sweeping θ over [0, π/2] is an equal-power crossfade from A to B; quad is the
// free orthogonal (I/Q) companion. θ=0 → (a, b); θ=π/2 → (b, −a).

struct XfadePair {
	float out;
	float quad;
};

// Precomputed-trig form — pass sinf_neon/cosf_neon results on Bela to avoid recomputing.
inline XfadePair equalPowerRotate(float a, float b, float cosT, float sinT) {
	return { a * cosT + b * sinT, -a * sinT + b * cosT };
}

inline XfadePair equalPowerRotate(float a, float b, float theta) {
	return equalPowerRotate(a, b, cosf(theta), sinf(theta));
}

// Convenience: single-output equal-power crossfade by position p in [0,1].
// p=0 → a, p=1 → b, constant power between (p maps to θ = p·π/2).
inline float equalPowerFade(float a, float b, float p) {
	float t = p * (float)M_PI * 0.5f;
	return a * cosf(t) + b * sinf(t);
}
