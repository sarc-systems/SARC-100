#ifdef __INTELLISENSE__
#include "./stubs/Bela.h"
#include "./stubs/Gui.h"
#include "./stubs/GuiController.h"
#else
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <unistd.h>
#endif
#include <stdlib.h>
#include <math.h>
#ifdef __INTELLISENSE__
#include "./stubs/math_neon.h"
#else
#include <libraries/math_neon/math_neon.h>
#endif

#include "pins.h"

#define TWO_PI (2.0f * (float)M_PI)

// EXP FM steepness — not on the panel (CLAUDE.md's 16-element list has no such control), so
// this is an internal, non-CV constant. exp_rate = sign(cv) * exp(k * |cv|), so at |cv|=1
// (full attenuverter+bias swing) rate = exp(k) rad/s. k=10 -> exp(10) ~= 22026 rad/s ~= 3505Hz,
// a reasonable upper FM rate. Tune after listening — there's no spec'd number for this.
const float kExpFmK = 10.0f;

// LIN FM scale — same situation, no panel control. Picked so lin_cv=1 reaches roughly the
// same max rate as EXP FM above, for the two paths to feel comparable in range.
const float kLinFmK = 22026.0f; // rad/s per unit cv

// Default LIN FM bias gives an audible carrier (220Hz) out of the box instead of silence —
// with the attenuverters gone, bias is the only thing setting the rate when nothing's
// patched in, so a 0 default would just be a held DC phase.
const float kDefaultFreqHz = 220.0f;
const float kLinFmBiasDefault = (TWO_PI * kDefaultFreqHz) / kLinFmK;

Gui gui;
GuiController controller;

unsigned int gExpFmBiasSliderIdx;
unsigned int gLinFmBiasSliderIdx;

// Display-only readouts — written only from the aux meter task (setSliderValue is not
// RT-safe), never read back as control input. Mirrors tine/servo/spline's
// *DisplaySliderIdx convention.
unsigned int gExpFmInDisplaySliderIdx;
unsigned int gLinFmInDisplaySliderIdx;
unsigned int gExpCvDisplaySliderIdx;
unsigned int gLinCvDisplaySliderIdx;
unsigned int gRateHzDisplaySliderIdx;
unsigned int gSinDisplaySliderIdx;
unsigned int gCosDisplaySliderIdx;
unsigned int gRampDisplaySliderIdx;
unsigned int gRampPerpDisplaySliderIdx;
unsigned int gRampX4DisplaySliderIdx;
unsigned int gRampX2DisplaySliderIdx;
unsigned int gEvenDisplaySliderIdx;
unsigned int gHalfDisplaySliderIdx;

float gPhi = 0.0f;
int gSyncPrev = 0;

float gExpFmIn = 0.0f, gLinFmIn = 0.0f;
float gExpCv = 0.0f, gLinCv = 0.0f;
float gRateHz = 0.0f;
float gSinOut = 0.0f, gCosOut = 0.0f;
float gRampOut = 0.0f, gRampPerpOut = 0.0f;
float gRampX4Out = 0.0f, gRampX2Out = 0.0f;
float gEvenOut = 0.0f, gHalfOut = 0.0f;

inline int analogFrameForAudio(BelaContext *context, int audioFrame) {
	if(context->analogFrames == 0) return 0;
	return (audioFrame * (int)context->analogFrames) / (int)context->audioFrames;
}

// Wrap to [0, 2pi) — handles negative input too (the accumulator is bidirectional), unlike a
// plain fmodf which can return a negative result for negative input.
inline float wrapTwoPi(float x) {
	x = fmodf(x, TWO_PI);
	if(x < 0.0f) x += TWO_PI;
	return x;
}

// Lower-priority task: GUI setSliderValue is not RT-safe — never call from render().
void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		controller.setSliderValue(gExpFmInDisplaySliderIdx, gExpFmIn);
		controller.setSliderValue(gLinFmInDisplaySliderIdx, gLinFmIn);
		controller.setSliderValue(gExpCvDisplaySliderIdx,   gExpCv);
		controller.setSliderValue(gLinCvDisplaySliderIdx,   gLinCv);
		controller.setSliderValue(gRateHzDisplaySliderIdx,  gRateHz);
		controller.setSliderValue(gSinDisplaySliderIdx,      gSinOut);
		controller.setSliderValue(gCosDisplaySliderIdx,      gCosOut);
		controller.setSliderValue(gRampDisplaySliderIdx,     gRampOut);
		controller.setSliderValue(gRampPerpDisplaySliderIdx, gRampPerpOut);
		controller.setSliderValue(gRampX4DisplaySliderIdx,   gRampX4Out);
		controller.setSliderValue(gRampX2DisplaySliderIdx,   gRampX2Out);
		controller.setSliderValue(gEvenDisplaySliderIdx,     gEvenOut);
		controller.setSliderValue(gHalfDisplaySliderIdx,     gHalfOut);
#ifdef __INTELLISENSE__
		for(volatile int d = 0; d < 80000; ++d) {}
#else
		usleep(80000);
#endif
	}
}

bool setup(BelaContext *context, void *userData) {
	gui.setup(context->projectName);
	controller.setup(&gui, "Turbine");

	// Bipolar bias (pull-pot), standing in for the deferred physical panel — through-zero FM
	// needs a true bipolar range, unlike most other modules' 0..1 CVs. Attenuverters are not
	// on the GUI (no dev-stage knob for them yet) — a patched signal contributes at unity.
	// Lin FM defaults to kLinFmBiasDefault (220Hz) so the module isn't silent out of the box.
	gExpFmBiasSliderIdx = controller.addSlider("Exp FM Bias", 0.0, -1.0, 1.0, 0.001);
	gLinFmBiasSliderIdx = controller.addSlider("Lin FM Bias", kLinFmBiasDefault, -1.0, 1.0, 0.001);

	gExpFmInDisplaySliderIdx = controller.addSlider("Exp FM IN (A0)", 0.0, -1.0, 1.0, 0.001);
	gLinFmInDisplaySliderIdx = controller.addSlider("Lin FM IN (A1)", 0.0, -1.0, 1.0, 0.001);
	gExpCvDisplaySliderIdx   = controller.addSlider("Exp CV (eff)",   0.0, -1.0, 1.0, 0.001);
	gLinCvDisplaySliderIdx   = controller.addSlider("Lin CV (eff)",   0.0, -1.0, 1.0, 0.001);
	gRateHzDisplaySliderIdx  = controller.addSlider("Rate Hz (eff)",  0.0, -5000.0, 5000.0, 0.1);

	gSinDisplaySliderIdx      = controller.addSlider("Sin (eff)",       0.0, -1.0, 1.0, 0.001);
	gCosDisplaySliderIdx      = controller.addSlider("Cos (eff)",       0.0, -1.0, 1.0, 0.001);
	gRampDisplaySliderIdx     = controller.addSlider("Ramp (eff)",      0.0,  0.0, 1.0, 0.001);
	gRampPerpDisplaySliderIdx = controller.addSlider("Ramp Perp (eff)", 0.0,  0.0, 1.0, 0.001);
	gRampX4DisplaySliderIdx   = controller.addSlider("Ramp x4 (eff)",   0.0,  0.0, 1.0, 0.001);
	gRampX2DisplaySliderIdx   = controller.addSlider("Ramp x2 (eff)",   0.0, -1.0, 1.0, 0.001);
	gEvenDisplaySliderIdx     = controller.addSlider("Even (eff)",      0.0,  0.0, 1.0, 1.0);
	gHalfDisplaySliderIdx     = controller.addSlider("Half (eff)",      0.0,  0.0, 1.0, 1.0);

	pinMode(context, 0, DIGITAL_SYNC, INPUT);
	pinMode(context, 0, DIGITAL_FREEZE, INPUT);
	pinMode(context, 0, DIGITAL_EVEN, OUTPUT);
	pinMode(context, 0, DIGITAL_HALF, OUTPUT);

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);

	return true;
}

void render(BelaContext *context, void *userData) {
	float dt = 1.0f / (float)context->audioSampleRate;

	// Bias is a plain GUI knob (no CV of its own) — block-rate read is enough.
	float expBias = controller.getSliderValue(gExpFmBiasSliderIdx);
	float linBias = controller.getSliderValue(gLinFmBiasSliderIdx);

	for(int frame = 0; frame < context->audioFrames; frame++) {
		int af = (context->analogFrames > 0) ? analogFrameForAudio(context, frame) : 0;

		// EXP FM / LIN FM are read at audio rate — through-zero FM is explicitly meant to
		// work with a hot audio-rate signal, so this can't be a block-rate CV like most other
		// modules' shape parameters. Raw analogRead is 0..1; remapped to -1..1 here since the
		// jack is genuinely bipolar (through-zero), unlike every other CV in this project so
		// far, which has been unipolar by nature.
		float expSignal = 0.0f, linSignal = 0.0f;
		if(context->analogFrames > 0 && ANALOG_EXP_FM_CV < context->analogInChannels)
			expSignal = 2.0f * analogRead(context, af, ANALOG_EXP_FM_CV) - 1.0f;
		if(context->analogFrames > 0 && ANALOG_LIN_FM_CV < context->analogInChannels)
			linSignal = 2.0f * analogRead(context, af, ANALOG_LIN_FM_CV) - 1.0f;
		gExpFmIn = expSignal;
		gLinFmIn = linSignal;

		// Pull-pot bias summed with the patched signal at unity (no attenuverter knob yet).
		// Deliberately unclamped — an open-ended bipolar sum, same as a real bias op-amp
		// stage (which would just rail at its own supply, not something to model here).
		float expCv = expBias + expSignal;
		float linCv = linBias + linSignal;
		gExpCv = expCv;
		gLinCv = linCv;

		// EXP FM: through-zero exponential. sign(0)=0 makes the whole product exactly 0 at
		// expCv==0 — a standard unipolar exponential converter can't do that (0V there is
		// some reference frequency, not silence), so the sign term is load-bearing, not
		// decorative.
		float expSign = (expCv > 0.0f) ? 1.0f : (expCv < 0.0f ? -1.0f : 0.0f);
		float expRate = expSign * expf(kExpFmK * fabsf(expCv));
		float linRate = kLinFmK * linCv;
		float rate = expRate + linRate;
		gRateHz = rate / TWO_PI;

		// SYNC (edge) resolves before FREEZE (level) when both fire the same sample, per
		// CLAUDE.md's same-sample ordering — gives "snap to 0 and hold" as one action.
		int sync = digitalRead(context, frame, DIGITAL_SYNC);
		int freeze = digitalRead(context, frame, DIGITAL_FREEZE);
		if(sync && !gSyncPrev) gPhi = 0.0f;
		gSyncPrev = sync ? 1 : 0;

		// FREEZE is a hard override on integration itself, not a zeroing of the FM inputs —
		// skip the accumulate step entirely rather than feeding rate=0 through it, so a hot
		// audio-rate FM signal truly cannot move phi while frozen.
		if(!freeze) {
			gPhi += rate * dt;
		}
		gPhi = wrapTwoPi(gPhi); // unconditional numerical hygiene, not user-facing

		float sinOut = sinf(gPhi);
		float cosOut = cosf(gPhi);
		float ramp = gPhi / TWO_PI;                       // unipolar [0,1)
		float rampPerp = wrapTwoPi(gPhi + (float)M_PI_2) / TWO_PI; // unipolar [0,1)
		float rampX4 = fmodf(gPhi, (float)M_PI_2) / (float)M_PI_2; // unipolar [0,1), x4 wrap rate
		float rampX2raw = fmodf(gPhi, (float)M_PI) / (float)M_PI; // 0..1, x2 wrap rate
		float rampX2 = 2.0f * rampX2raw - 1.0f;            // bipolar [-1,1)

		int quadrant = (int)floorf(gPhi / (float)M_PI_2);   // 0..3 since gPhi in [0,2pi)
		bool even = (quadrant % 2) == 0;                    // register-state read, not a counter
		bool half = gPhi >= (float)M_PI;                    // same — correct under reversal

		gSinOut = sinOut; gCosOut = cosOut;
		gRampOut = ramp; gRampPerpOut = rampPerp;
		gRampX4Out = rampX4; gRampX2Out = rampX2;
		gEvenOut = even ? 1.0f : 0.0f; gHalfOut = half ? 1.0f : 0.0f;

		// All six outputs ride DC-coupled outs now (no AC-coupled pair used by this module) —
		// bipolar signals (SIN, COS, Ramp x2) need the 0.5=0V remap for that write convention,
		// matching servo's EFF; the unipolar ones (RAMP, RAMP-PERP, Ramp x4) are already [0,1)
		// and write as-is.
		if(AUDIO_OUT_SIN < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_SIN, sinOut * 0.5f + 0.5f);
		if(AUDIO_OUT_COS < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_COS, cosOut * 0.5f + 0.5f);
		if(AUDIO_OUT_RAMP < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_RAMP, ramp);
		if(AUDIO_OUT_RAMP_PERP < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_RAMP_PERP, rampPerp);
		if(AUDIO_OUT_RAMP_X4 < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_RAMP_X4, rampX4);
		if(AUDIO_OUT_RAMP_X2 < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_RAMP_X2, rampX2 * 0.5f + 0.5f);

		digitalWrite(context, frame, DIGITAL_EVEN, even ? 1 : 0);
		digitalWrite(context, frame, DIGITAL_HALF, half ? 1 : 0);
	}
}

void cleanup(BelaContext *context, void *userData) {}
