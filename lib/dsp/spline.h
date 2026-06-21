#pragma once

#include <algorithm>

// N evenly-spaced knots over x = 0..1. Open curve — does NOT wrap; out-of-range x clamps to
// the nearest end, and the two end knots are duplicated for the smooth-mode tangent math
// (zero curvature at the ends, the same "clamped" boundary condition as a natural spline).
//
// INTERPOLATION is a single continuous 0..1 control, cross-faded rather than switched, so
// sweeping it morphs continuously through three characters rather than hard-switching:
//   0    -> discontinuous: holds the nearest knot's value (a stepped/sample-and-hold read)
//   0.5  -> linear: straight line between the two surrounding knots
//   1    -> smooth: Catmull-Rom-style cardinal cubic (continuous value AND slope at knots)
// Knot values are reproduced exactly at the N knot x-positions regardless of interpolation.
template <int N>
class Spline {
public:
	void setKnot(int i, float value) {
		if(i >= 0 && i < N) knots_[i] = value;
	}

	void setKnots(const float values[N]) {
		for(int i = 0; i < N; i++) knots_[i] = values[i];
	}

	float knot(int i) const {
		return knots_[i >= 0 && i < N ? i : 0];
	}

	void setInterpolation(float interp0to1) {
		interp_ = std::min(std::max(interp0to1, 0.0f), 1.0f);
	}

	float evaluate(float x) const {
		x = std::min(std::max(x, 0.0f), 1.0f);
		float scaled = x * (float)(N - 1);
		int seg = (int)scaled;
		if(seg >= N - 1) seg = N - 2;
		if(seg < 0) seg = 0;
		float t = scaled - (float)seg;

		float p0 = knots_[seg];
		float p1 = knots_[seg + 1];
		float pPrev = knots_[seg > 0 ? seg - 1 : seg];
		float pNext = knots_[seg < N - 2 ? seg + 2 : seg + 1];

		float discontinuous = (t < 0.5f) ? p0 : p1; // nearest knot
		float linear = p0 + t * (p1 - p0);

		float m0 = 0.5f * (p1 - pPrev);
		float m1 = 0.5f * (pNext - p0);
		float t2 = t * t;
		float t3 = t2 * t;
		float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
		float h10 = t3 - 2.0f * t2 + t;
		float h01 = -2.0f * t3 + 3.0f * t2;
		float h11 = t3 - t2;
		float smooth = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;

		if(interp_ <= 0.5f) {
			float k = interp_ / 0.5f;
			return discontinuous + k * (linear - discontinuous);
		}
		float k = (interp_ - 0.5f) / 0.5f;
		return linear + k * (smooth - linear);
	}

private:
	float knots_[N] = {};
	float interp_ = 0.5f;
};
