#pragma once

#include <cmath>

// One-pole attack/decay envelope follower on the rectified input. Plain C++,
// no Bela dependency — derive attack/decay coefficients from a time constant
// as coeff = 1.0f - expf(-1.0f / (timeConstantSeconds * sampleRate)).
class EnvelopeFollower {
public:
	void setCoeffs(float attackCoeff, float decayCoeff) {
		attack_ = attackCoeff;
		decay_ = decayCoeff;
	}

	void reset(float value = 0.0f) {
		env_ = value;
	}

	float process(float input) {
		float target = fabsf(input);
		float coeff = (target > env_) ? attack_ : decay_;
		env_ += coeff * (target - env_);
		return env_;
	}

	float value() const { return env_; }

private:
	float env_ = 0.0f;
	float attack_ = 1.0f;
	float decay_ = 1.0f;
};
