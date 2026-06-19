#pragma once

#include <algorithm>

// Slow leaky integrator: confidence += rate * (success - confidence).
// `rate` is supplied by the caller (derived from TIME * GOVERNOR_TIME_MULT)
// so confidence is provably slower than the PID loop it watches — this
// class only integrates, it has no notion of timescale itself.
class Confidence {
public:
	void reset(float value = 0.0f) {
		confidence_ = std::min(std::max(value, 0.0f), 1.0f);
	}

	// success: 0..1 instantaneous tracking-success signal.
	// rate: 0..1 leak coefficient for this update (caller-derived).
	float update(float success, float rate) {
		confidence_ += rate * (success - confidence_);
		confidence_ = std::min(std::max(confidence_, 0.0f), 1.0f);
		return confidence_;
	}

	float value() const { return confidence_; }

private:
	float confidence_ = 0.0f;
};
