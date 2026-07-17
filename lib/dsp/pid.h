#pragma once

#include <cmath>
#include <algorithm>

// PID controller with one-pole-filtered derivative and conditional-
// integration anti-windup: the integral only accumulates when doing so
// wouldn't push the unsaturated output further past the output clamp.
// No back-calculation gain to tune separately.
class PID {
public:
	void setGains(float kp, float ki, float kd) {
		// Bumpless integral-gain change. The integral is stored unscaled (output uses
		// ki_ * integral_), so changing ki would instantly rescale the integral's
		// contribution — an output bump when a gain-scheduled ki is swept (e.g. SERVO
		// RATE scaling ki live). Rescale the stored integral to preserve ki_ * integral_
		// across the change, so only the loop *dynamics* change, not the operating point.
		// kp/kd need no such handling: at a steady point their terms are ~0.
		if (fabsf(ki) > 1e-9f && fabsf(ki_) > 1e-9f && ki != ki_) {
			integral_ *= ki_ / ki;
		}
		kp_ = kp;
		ki_ = ki;
		kd_ = kd;
	}

	// dFilterCoeff: one-pole coefficient (0,1] applied to the raw derivative
	// term before use, to tame measurement noise.
	void setDerivativeFilter(float dFilterCoeff) {
		dFilterCoeff_ = dFilterCoeff;
	}

	void setOutputLimits(float minOutput, float maxOutput) {
		outMin_ = minOutput;
		outMax_ = maxOutput;
	}

	// RESET = 0: clear all state, output returns to neutral on next update().
	void reset() {
		integral_ = 0.0f;
		filteredD_ = 0.0f;
		prevError_ = 0.0f;
		havePrevError_ = false;
		lastOutput_ = 0.0f;
	}

	// RESET = last: re-seed the integral so the very next update() call with
	// the same error reproduces seedOutput exactly (bumpless), then continues
	// integrating normally from there.
	void reset(float seedOutput, float currentError) {
		filteredD_ = 0.0f;
		prevError_ = currentError;
		havePrevError_ = true;
		lastOutput_ = seedOutput;
		integral_ = (fabsf(ki_) > 1e-9f) ? (seedOutput - kp_ * currentError) / ki_ : 0.0f;
	}

	// FREEZE: skip integration/derivative update, hold last output.
	float freeze() const {
		return lastOutput_;
	}

	float update(float error, float dt) {
		float rawD = (havePrevError_ && dt > 0.0f) ? (error - prevError_) / dt : 0.0f;
		prevError_ = error;
		havePrevError_ = true;
		filteredD_ += dFilterCoeff_ * (rawD - filteredD_);

		float pTerm = kp_ * error;
		float dTerm = kd_ * filteredD_;

		// Output uses the integral as accumulated through the *previous* call,
		// so reset(seedOutput, error) can seed it to land on seedOutput exactly
		// on the very next update() — this step's contribution is folded in
		// below, for use starting next call.
		float output = pTerm + ki_ * integral_ + dTerm;
		output = std::min(std::max(output, outMin_), outMax_);

		float candidateIntegral = integral_ + error * dt;
		float candidateOutput = pTerm + ki_ * candidateIntegral + dTerm;
		bool inRange = candidateOutput <= outMax_ && candidateOutput >= outMin_;
		bool recoveringHigh = candidateOutput > outMax_ && error < 0.0f;
		bool recoveringLow = candidateOutput < outMin_ && error > 0.0f;
		if (inRange || recoveringHigh || recoveringLow) {
			integral_ = candidateIntegral;
		}

		lastOutput_ = output;
		return output;
	}

	float lastOutput() const { return lastOutput_; }
	float integralState() const { return integral_; }

private:
	float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
	float dFilterCoeff_ = 1.0f;
	float outMin_ = -1.0f, outMax_ = 1.0f;

	float integral_ = 0.0f;
	float filteredD_ = 0.0f;
	float prevError_ = 0.0f;
	bool havePrevError_ = false;
	float lastOutput_ = 0.0f;
};
