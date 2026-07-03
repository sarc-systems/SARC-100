#pragma once

// Standardized analog CV input for SARC modules.
//
// Wraps Bela's analogRead with one-pole smoothing (Smoother) to reject ADC
// quantization steps and pot/wiper noise — the same conditioning TINE applied
// inline, factored out so every module reads CV the same way. One CvIn instance
// per analog control.
//
// Unlike lib/dsp/, this header depends on Bela (analogRead / BelaContext), so it
// lives in lib/io/. Include it AFTER <Bela.h> (or the IntelliSense stub).
//
// Usage — block-rate, the common case (one read per render() block):
//   CvIn tune;
//   tune.setup(PIN_TUNE, context);   // in setup()
//   ...
//   float v = tune.read(context);    // in render(), once per block
//
// For a control that must track faster than the block (e.g. audio-rate pitch),
// drive lib/dsp/Smoother directly at audio rate instead — see TINE's tune path.

#include "../dsp/smooth.h"

// 10 ms — long enough to kill ADC/wiper noise, short enough to feel immediate.
constexpr float kCvSmoothTimeConstS = 0.01f;

class CvIn {
public:
	void setup(int pin, BelaContext *context, float timeConstantS = kCvSmoothTimeConstS) {
		pin_ = pin;
		float blockPeriodS = (float)context->audioFrames / (float)context->audioSampleRate;
		sm_.init(blockPeriodS, timeConstantS);
	}

	// Block-rate read: analog frame 0 of the current block, smoothed. Call once
	// per render() block. If the context has no analog frames, holds last value.
	float read(BelaContext *context) {
		float raw = (context->analogFrames > 0) ? analogRead(context, 0, pin_) : sm_.value();
		return sm_.process(raw);
	}

	void  snap(float v)   { sm_.snap(v); }
	float value() const   { return sm_.value(); }
	int   pin()   const   { return pin_; }

private:
	Smoother sm_;
	int      pin_ = 0;
};
