#pragma once

// One-pole IIR smoother for ADC/CV noise. Plain C++, no Bela dependency.
//
// Usage:
//   Smoother s;
//   s.init(callPeriodS, timeConstantS);   // once in setup()
//   float v = s.process(analogRead(...));  // each block or each sample
//
// callPeriodS is the actual time between process() calls:
//   - block-rate CV:  (float)context->audioFrames / context->audioSampleRate
//   - per-sample CV:  1.0f / context->audioSampleRate
//
// The first process() call snapshots the raw value (no filtering), so the output
// starts at the correct level rather than ramping slowly from 0.
class Smoother {
public:
	void init(float callPeriodS, float timeConstantS) {
		coeff_ = callPeriodS / (timeConstantS + callPeriodS);
	}

	float process(float raw) {
		if(!initialised_) { value_ = raw; initialised_ = true; return raw; }
		value_ += coeff_ * (raw - value_);
		return value_;
	}

	// Force-set the current value — use after a RESET or hard-snap to avoid the
	// smoother pulling back toward the old value across the discontinuity.
	void snap(float v) { value_ = v; initialised_ = true; }

	float value() const { return value_; }

private:
	float value_       = 0.0f;
	float coeff_       = 1.0f;
	bool  initialised_ = false;
};
