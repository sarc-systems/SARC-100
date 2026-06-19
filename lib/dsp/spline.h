#pragma once

#include <algorithm>

// 5-point spline over evenly spaced knots at x = 0, .25, .5, .75, 1, with a
// TENSION control. At each segment we blend a Catmull-Rom-style cardinal
// cubic (smooth curvature) with plain linear interpolation between the two
// knots; tension picks the mix. tension=0 -> fully cardinal (smooth).
// tension=1 -> fully linear (angular corners at knots). A zero-tangent
// Hermite cubic is NOT the same as linear (it eases in/out), so the blend is
// against the linear value directly rather than just zeroing the tangents.
// Knot values are reproduced exactly at the 5 sample x-positions regardless
// of tension.
class Spline5 {
public:
	static constexpr int kNumKnots = 5;

	void setKnot(int i, float value) {
		if (i >= 0 && i < kNumKnots) knots_[i] = value;
	}

	void setKnots(const float values[kNumKnots]) {
		for (int i = 0; i < kNumKnots; i++) knots_[i] = values[i];
	}

	float knot(int i) const {
		return knots_[i >= 0 && i < kNumKnots ? i : 0];
	}

	void setTension(float tension) {
		tension_ = std::min(std::max(tension, 0.0f), 1.0f);
	}

	float evaluate(float x) const {
		x = std::min(std::max(x, 0.0f), 1.0f);
		float scaled = x * (float)(kNumKnots - 1);
		int seg = (int)scaled;
		if (seg >= kNumKnots - 1) seg = kNumKnots - 2;
		float t = scaled - (float)seg;

		float p0 = knots_[seg];
		float p1 = knots_[seg + 1];
		float pPrev = knots_[seg > 0 ? seg - 1 : seg];
		float pNext = knots_[seg < kNumKnots - 2 ? seg + 2 : seg + 1];

		float m0 = 0.5f * (p1 - pPrev);
		float m1 = 0.5f * (pNext - p0);

		float t2 = t * t;
		float t3 = t2 * t;
		float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
		float h10 = t3 - 2.0f * t2 + t;
		float h01 = -2.0f * t3 + 3.0f * t2;
		float h11 = t3 - t2;

		float cardinal = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
		float linear = p0 + t * (p1 - p0);
		return (1.0f - tension_) * cardinal + tension_ * linear;
	}

private:
	float knots_[kNumKnots] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float tension_ = 0.0f;
};
