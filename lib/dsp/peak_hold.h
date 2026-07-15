#pragma once

#include <cmath>

// Peak-hold with exponential decay. Plain C++, no Bela dependency.
//
// process(x) jumps instantly to any new peak, then decays toward 0 between peaks,
// so a brief event stays visible long enough to read on a meter or LED. Feed it
// |signal| for a peak/level meter, or a 0/1 event flag for a retriggerable hold
// (e.g. a saturation/"pinned" indicator that flashes and fades).
//
// Usage:
//   PeakHold p;
//   p.init(callPeriodS, holdTimeConstS);   // once in setup()
//   float v = p.process(x);                 // each block or each sample
//
// callPeriodS is the actual time between process() calls:
//   - per-sample: 1.0f / context->audioSampleRate
//   - block-rate: (float)context->audioFrames / context->audioSampleRate
// holdTimeConstS is the decay time constant (larger = holds longer before fading).
class PeakHold {
public:
	void init(float callPeriodS, float holdTimeConstS) {
		decay_ = expf(-callPeriodS / holdTimeConstS);
	}

	float process(float x) {
		value_ *= decay_;
		if(x > value_) value_ = x;
		return value_;
	}

	void reset(float v = 0.0f) { value_ = v; }

	float value() const { return value_; }

private:
	float value_ = 0.0f;
	float decay_ = 0.0f;
};
