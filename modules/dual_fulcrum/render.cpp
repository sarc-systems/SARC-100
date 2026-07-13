#ifdef __INTELLISENSE__
#include "./stubs/Bela.h"
#include "./stubs/Gui.h"
#include "./stubs/GuiController.h"
#include "../../lib/dsp/crossfade.h"
#else
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <unistd.h>
#include "lib/dsp/crossfade.h"
#endif
#include <stdio.h>
#include <math.h>
#ifdef __INTELLISENSE__
#include "./stubs/math_neon.h"
#else
#include <libraries/math_neon/math_neon.h>
#endif

#include "pins.h"

// ─────────────────────────────────────────────────────────────────────────────
// DUAL FULCRUM — two independent rotational equal-power crossfaders.
// Per instance, per sample:  OUT/QUAD = r · rotate(a, b, θ), with REVERSE negating θ
// (before the trig) and INVERT negating the output pair. The rotation itself is the
// shared lib primitive (lib/dsp/crossfade.h); this file is just I/O + voltage mapping.
// The two instances share no state.
// ─────────────────────────────────────────────────────────────────────────────

constexpr int NUM_INST = 2;

// Per-instance channel maps (indexed by instance 0=F1, 1=F2).
const int kAinA[NUM_INST]     = {AN_F1_A,       AN_F2_A};
const int kAinB[NUM_INST]     = {AN_F1_B,       AN_F2_B};
const int kAinTheta[NUM_INST] = {AN_F1_THETA,   AN_F2_THETA};
const int kAinR[NUM_INST]     = {AN_F1_R,       AN_F2_R};
const int kOut[NUM_INST]      = {AUDIO_F1_OUT,  AUDIO_F2_OUT};
const int kQuad[NUM_INST]     = {AUDIO_F1_QUAD, AUDIO_F2_QUAD};
const int kInvert[NUM_INST]   = {DIG_F1_INVERT, DIG_F2_INVERT};
const int kReverse[NUM_INST]  = {DIG_F1_REVERSE, DIG_F2_REVERSE};

// Voltage mapping. Bela analog inputs are unipolar [0,1]; the analog front-end centers
// bipolar signals at 0.5. Recover bipolar with 2v-1; map θ's [0,1] onto [-pi/2, pi/2].
// (CLAUDE.md open question: confirm ±V = ±pi/2 against the real front-end scaling — these
// two constants are where you'd retune it.)
inline float bipolar(float v)  { return 2.0f * v - 1.0f; }             // [0,1] -> [-1,1]
inline float thetaMap(float v) { return (v - 0.5f) * (float)M_PI; }    // [0,1] -> [-pi/2, pi/2]
inline float toDcOut(float x)  { float y = 0.5f + 0.5f * x; return y < 0.0f ? 0.0f : (y > 1.0f ? 1.0f : y); }

inline int analogFrameForAudio(BelaContext *ctx, int audioFrame) {
	if(ctx->analogFrames == 0) return 0;
	return (audioFrame * (int)ctx->analogFrames) / (int)ctx->audioFrames;
}

// ── GUI (display-only meters) ────────────────────────────────────────────────
Gui gui;
GuiController controller;
unsigned int gDispTheta[NUM_INST], gDispR[NUM_INST], gDispOutPk[NUM_INST];
float gEffTheta[NUM_INST] = {0.0f};
float gEffR[NUM_INST]     = {0.0f};
float gOutPeak[NUM_INST]  = {0.0f};
const float kPeakDecay = 0.9995f;

void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		for(int m = 0; m < NUM_INST; ++m) {
			controller.setSliderValue(gDispTheta[m], gEffTheta[m]);
			controller.setSliderValue(gDispR[m],     gEffR[m]);
			controller.setSliderValue(gDispOutPk[m], gOutPeak[m]);
		}
#ifdef __INTELLISENSE__
		for(volatile int d = 0; d < 80000; ++d) {}
#else
		usleep(80000);
#endif
	}
}

bool setup(BelaContext *context, void *userData) {
	gui.setup(context->projectName);
	controller.setup(&gui, "Dual Fulcrum");

	const char *names[NUM_INST] = {"F1", "F2"};
	for(int m = 0; m < NUM_INST; ++m) {
		char lbl[40];
		snprintf(lbl, sizeof lbl, "%s theta (rad)", names[m]);
		gDispTheta[m] = controller.addSlider(lbl, 0.0, -1.5708, 1.5708, 0.001);
		snprintf(lbl, sizeof lbl, "%s r", names[m]);
		gDispR[m]     = controller.addSlider(lbl, 0.0, -1.0, 1.0, 0.001);
		snprintf(lbl, sizeof lbl, "%s OUT peak", names[m]);
		gDispOutPk[m] = controller.addSlider(lbl, 0.0, 0.0, 1.5, 0.001);
	}

	for(int m = 0; m < NUM_INST; ++m) {
		pinMode(context, 0, kInvert[m],  INPUT);
		pinMode(context, 0, kReverse[m], INPUT);
	}

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);
	return true;
}

void render(BelaContext *context, void *userData) {
	for(unsigned int frame = 0; frame < context->audioFrames; ++frame) {
		int af = analogFrameForAudio(context, frame);

		for(int m = 0; m < NUM_INST; ++m) {
			// Inputs — front-end already applied att + bias/offset; ADC is unipolar [0,1].
			float a = 0.0f, b = 0.0f, thetaCv = 0.5f, rCv = 0.5f;
			if(context->analogFrames > 0) {
				if((unsigned)kAinA[m]     < context->analogInChannels) a       = bipolar(analogRead(context, af, kAinA[m]));
				if((unsigned)kAinB[m]     < context->analogInChannels) b       = bipolar(analogRead(context, af, kAinB[m]));
				if((unsigned)kAinTheta[m] < context->analogInChannels) thetaCv = analogRead(context, af, kAinTheta[m]);
				if((unsigned)kAinR[m]     < context->analogInChannels) rCv     = analogRead(context, af, kAinR[m]);
			}
			float theta = thetaMap(thetaCv);
			float r     = bipolar(rCv);

			bool reverse = digitalRead(context, frame, kReverse[m]);
			bool invert  = digitalRead(context, frame, kInvert[m]);

			if(reverse) theta = -theta;   // REVERSE: negate θ before the trig

			// Equal-power rotation (shared lib primitive), then magnitude and INVERT.
			XfadePair xf = equalPowerRotate(a, b, cosf_neon(theta), sinf_neon(theta));
			float out  = r * xf.out;
			float quad = r * xf.quad;
			if(invert) { out = -out; quad = -quad; }   // INVERT: flip the whole output

			if((unsigned)kOut[m]  < context->audioOutChannels) audioWrite(context, frame, kOut[m],  toDcOut(out));
			if((unsigned)kQuad[m] < context->audioOutChannels) audioWrite(context, frame, kQuad[m], toDcOut(quad));

			// Meters.
			gEffTheta[m] = theta;
			gEffR[m]     = r;
			float ao = fabsf(out);
			gOutPeak[m]  = ao > gOutPeak[m] ? ao : gOutPeak[m] * kPeakDecay;
		}
	}
}

void cleanup(BelaContext *context, void *userData) {}
