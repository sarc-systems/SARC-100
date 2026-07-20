#ifdef __INTELLISENSE__
#include "./stubs/Bela.h"
#include "./stubs/Gui.h"
#include "./stubs/GuiController.h"
#include "../../lib/dsp/remap.h"
#include "../../lib/io/cv_input.h"
#else
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <unistd.h>
#include "lib/dsp/remap.h"
#include "lib/io/cv_input.h"
#endif

#include "pins.h"

// REMAP — voltage-domain transform module. All DSP lives in lib/dsp/remap.h (Bela-
// agnostic, unit-tested); this file is Bela glue only: analog reads, θ conditioning
// + FREEZE_θ sample-and-hold, per-frame computeAll, DC-coupled audio-out writes, meters.
// See modules/remap/CLAUDE.md. Pin indices in pins.h are DUMMY placeholders (panel TBD).
//
// Output path deviates from the original spec's `analogWrite`/`out-V/2`: on the Gem Multi
// the DC-coupled outputs are AUDIO outs (audioWrite), and their output stage IS the
// bipolar convention. render writes a normalized [0,1] value (remap::mapOutput). Same
// reasoning drivetrain documents.

const float kV = remap::kDefaultV;

Gui gui;
GuiController controller;

// Display-only meters — written from the aux task (setSliderValue is not RT-safe).
unsigned int gXInDisplayIdx;
unsigned int gThetaInDisplayIdx;
unsigned int gFreezeDisplayIdx;
unsigned int gOutDisplayIdx[remap::kNumOutputs];

float gXIn = 0.0f;
float gThetaConditioned = 0.0f;   // θ after expand + FREEZE hold — the value actually used
float gFreezeState = 0.0f;
float gOut[remap::kNumOutputs] = {0.0f};

// θ is a control parameter, so it reads through CvIn (block-rate, one-pole smoothed) to
// de-zipper ADC/pot noise — matching servo's treatment of its control CVs and the "new
// modules should use CvIn" house standard. This overrides the spec's "no smoothing on θ"
// line (FOLD is sensitive to θ quantization steps). X is NOT smoothed: it's the signal
// input and may be audio-rate, so it stays a raw per-frame read (like servo's SIG).
CvIn gCvTheta;

// FREEZE_θ sample-and-hold state: the conditioned θ currently in force. Held while the
// gate is high; tracks live θ while low. Transforms always use this value.
float gHeldTheta = 0.0f;

// audio-out channel per transform output, aligned to remap::kOutputs order. Dropping
// outputs (open question #3) is a table edit here + in remap::kOutputs; DSP is untouched.
const int kOutChannel[remap::kNumOutputs] = {
	AUDIO_OUT_ROTATE1, AUDIO_OUT_ROTATE2,
	AUDIO_OUT_MIRROR1, AUDIO_OUT_MIRROR2,
	AUDIO_OUT_FOLD1,   AUDIO_OUT_FOLD2,
};

inline int analogFrameForAudio(BelaContext *context, int audioFrame) {
	if(context->analogFrames == 0) return 0;
	return (audioFrame * (int)context->analogFrames) / (int)context->audioFrames;
}

void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		controller.setSliderValue(gXInDisplayIdx,     gXIn);
		controller.setSliderValue(gThetaInDisplayIdx, gThetaConditioned);
		controller.setSliderValue(gFreezeDisplayIdx,  gFreezeState);
		for(int i = 0; i < remap::kNumOutputs; i++)
			controller.setSliderValue(gOutDisplayIdx[i], gOut[i]);
#ifdef __INTELLISENSE__
		for(volatile int d = 0; d < 80000; ++d) {}
#else
		usleep(80000);
#endif
	}
}

bool setup(BelaContext *context, void *userData) {
	gui.setup(context->projectName);
	controller.setup(&gui, "Remap");

	// Meters span the bipolar input range and the unipolar transform range.
	gXInDisplayIdx     = controller.addSlider("X in (V)",    0.0, -kV, kV,   0.001);
	gThetaInDisplayIdx = controller.addSlider("theta held (V)", 0.0, -kV, kV, 0.001);
	gFreezeDisplayIdx  = controller.addSlider("FREEZE theta", 0.0,  0.0, 1.0,  1.0);
	const char *names[remap::kNumOutputs] = {
		"ROTATE x1 (V)", "ROTATE x2 (V)", "MIRROR x1 (V)",
		"MIRROR x2 (V)", "FOLD x1 (V)",   "FOLD x2 (V)",
	};
	for(int i = 0; i < remap::kNumOutputs; i++)
		gOutDisplayIdx[i] = controller.addSlider(names[i], 0.0, 0.0, kV, 0.001);

	gCvTheta.setup(ANALOG_THETA, context);

	pinMode(context, 0, DIGITAL_FREEZE_THETA, INPUT);

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "remap-meter", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);

	return true;
}

void render(BelaContext *context, void *userData) {
	// θ is a control parameter: read block-rate through CvIn (smoothed) -> bipolar volts.
	float thetaLive = remap::expandBipolar(gCvTheta.read(context), kV);

	// FREEZE_θ (level): hold conditioned θ while the gate is high; track live θ while low.
	// Block-granular (θ is block-rate) — high anywhere in the block holds, matching servo.
	bool freeze = false;
	for(unsigned int n = 0; n < context->digitalFrames; n++)
		if(digitalRead(context, n, DIGITAL_FREEZE_THETA)) freeze = true;
	if(!freeze) gHeldTheta = thetaLive;

	for(int frame = 0; frame < context->audioFrames; frame++) {
		int af = analogFrameForAudio(context, frame);

		// X: signal input, raw per-frame read (may be audio-rate) -> bipolar volts. No smoothing.
		float xCv = (context->analogFrames > 0 && ANALOG_X < context->analogInChannels)
		            ? analogRead(context, af, ANALOG_X) : 0.5f;   // 0.5 == 0 V unpatched
		float x = remap::expandBipolar(xCv, kV);

		// All six transforms from live X and the (possibly held) θ.
		float out[remap::kNumOutputs];
		remap::computeAll(x, gHeldTheta, kV, out);

		// Transform outputs -> DC-coupled audio outs (normalized; bipolar at the jack).
		for(int i = 0; i < remap::kNumOutputs; i++) {
			if(kOutChannel[i] < (int)context->audioOutChannels)
				audioWrite(context, frame, kOutChannel[i], remap::mapOutput(out[i], kV));
			gOut[i] = out[i];
		}
		// θ output: conditioned/held θ, bipolar -> normalized.
		if(AUDIO_OUT_THETA < (int)context->audioOutChannels)
			audioWrite(context, frame, AUDIO_OUT_THETA, remap::mapBipolarOutput(gHeldTheta, kV));

		gXIn = x;
	}

	gThetaConditioned = gHeldTheta;
	gFreezeState = freeze ? 1.0f : 0.0f;
}

void cleanup(BelaContext *context, void *userData) {}
