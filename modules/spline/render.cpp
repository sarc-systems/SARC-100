#ifdef __INTELLISENSE__
#include "./stubs/Bela.h"
#include "./stubs/Gui.h"
#include "./stubs/GuiController.h"
#include "../../lib/dsp/spline.h"
#else
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <unistd.h>
#include "lib/dsp/spline.h"
#endif
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifdef __INTELLISENSE__
#include "./stubs/math_neon.h"
#else
#include <libraries/math_neon/math_neon.h>
#endif

#include "pins.h"

#define NUM_SEGMENTS 6

Gui gui;
GuiController controller;

unsigned int gSegmentSliderIdx[NUM_SEGMENTS];
unsigned int gScanSliderIdx;
unsigned int gInterpolateSliderIdx;

// Display-only readouts — written only from the aux meter task (setSliderValue is not
// RT-safe), never read back as control input. Mirrors tine/servo's *DisplaySliderIdx
// convention.
unsigned int gOutDisplaySliderIdx;
unsigned int gScanPositionDisplaySliderIdx;

// Raw-CV input meters — every analog input has no GUI base value of its own conceptually
// distinct from "the knob," so these mirror the literal analogRead value at each jack.
unsigned int gSegmentInDisplaySliderIdx[NUM_SEGMENTS];
unsigned int gScanInDisplaySliderIdx;
unsigned int gInterpolateInDisplaySliderIdx;

Spline<NUM_SEGMENTS> gSpline;

float gEffOut = 0.0f;
float gEffScanPosition = 0.0f;
float gSegmentIn[NUM_SEGMENTS] = {0.0f};
float gScanIn = 0.0f;
float gInterpolateIn = 0.0f;

inline float clamp01(float x) {
	return fminf(fmaxf(x, 0.0f), 1.0f);
}

inline int analogFrameForAudio(BelaContext *context, int audioFrame) {
	if(context->analogFrames == 0) return 0;
	return (audioFrame * (int)context->analogFrames) / (int)context->audioFrames;
}

const int kSegmentAnalogIn[NUM_SEGMENTS] = {
	ANALOG_SEGMENT1_CV, ANALOG_SEGMENT2_CV, ANALOG_SEGMENT3_CV,
	ANALOG_SEGMENT4_CV, ANALOG_SEGMENT5_CV, ANALOG_SEGMENT6_CV,
};

// Lower-priority task: GUI setSliderValue is not RT-safe — never call from render().
void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		controller.setSliderValue(gOutDisplaySliderIdx,          gEffOut);
		controller.setSliderValue(gScanPositionDisplaySliderIdx, gEffScanPosition);
		for(int i = 0; i < NUM_SEGMENTS; i++)
			controller.setSliderValue(gSegmentInDisplaySliderIdx[i], gSegmentIn[i]);
		controller.setSliderValue(gScanInDisplaySliderIdx,        gScanIn);
		controller.setSliderValue(gInterpolateInDisplaySliderIdx, gInterpolateIn);
#ifdef __INTELLISENSE__
		for(volatile int d = 0; d < 80000; ++d) {}
#else
		usleep(80000);
#endif
	}
}

bool setup(BelaContext *context, void *userData) {
	gui.setup(context->projectName);
	controller.setup(&gui, "Spline");

	for(int i = 0; i < NUM_SEGMENTS; i++) {
		char label[16];
		snprintf(label, sizeof(label), "Segment %d", i + 1);
		gSegmentSliderIdx[i] = controller.addSlider(label, 0.5, 0.0, 1.0, 0.001);
	}
	gScanSliderIdx        = controller.addSlider("Scan",        0.0, 0.0, 1.0, 0.001);
	gInterpolateSliderIdx = controller.addSlider("Interpolate", 0.5, 0.0, 1.0, 0.001);

	gOutDisplaySliderIdx          = controller.addSlider("Out (eff)",           0.0, 0.0, 1.0, 0.001);
	gScanPositionDisplaySliderIdx = controller.addSlider("Scan Position (eff)", 0.0, 0.0, 1.0, 0.001);

	for(int i = 0; i < NUM_SEGMENTS; i++) {
		char label[24];
		snprintf(label, sizeof(label), "Segment %d IN (A%d)", i + 1, i);
		gSegmentInDisplaySliderIdx[i] = controller.addSlider(label, 0.0, 0.0, 1.0, 0.001);
	}
	gScanInDisplaySliderIdx        = controller.addSlider("Scan IN (A6)",        0.0, 0.0, 1.0, 0.001);
	gInterpolateInDisplaySliderIdx = controller.addSlider("Interpolate IN (A7)", 0.0, 0.0, 1.0, 0.001);

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);

	return true;
}

void render(BelaContext *context, void *userData) {
	// Block-rate CVs (GUI slider + CV summed: knob sets a base value, an unpatched jack
	// contributes nothing). Segments and INTERPOLATE change slowly — no need for audio-rate
	// reads. SCAN is read per-audio-frame below instead, since it's the one input expected
	// to move fast (a scan sweep is the point of this module).
	for(int i = 0; i < NUM_SEGMENTS; i++) {
		float cv = 0.0f;
		if(context->analogFrames > 0 && kSegmentAnalogIn[i] < (int)context->analogInChannels)
			cv = analogRead(context, 0, kSegmentAnalogIn[i]);
		gSegmentIn[i] = cv;
		float segmentValue = clamp01(controller.getSliderValue(gSegmentSliderIdx[i]) + cv);
		gSpline.setKnot(i, segmentValue);
	}

	float cvInterpolate = 0.0f;
	if(context->analogFrames > 0 && ANALOG_INTERPOLATE_CV < context->analogInChannels)
		cvInterpolate = analogRead(context, 0, ANALOG_INTERPOLATE_CV);
	gInterpolateIn = cvInterpolate;
	float interpolate = clamp01(controller.getSliderValue(gInterpolateSliderIdx) + cvInterpolate);
	gSpline.setInterpolation(interpolate);

	for(int frame = 0; frame < context->audioFrames; frame++) {
		int af = (context->analogFrames > 0) ? analogFrameForAudio(context, frame) : 0;

		float cvScan = (context->analogFrames > 0 && ANALOG_SCAN_CV < context->analogInChannels)
			? analogRead(context, af, ANALOG_SCAN_CV) : 0.0f;
		gScanIn = cvScan;
		float scanPosition = clamp01(controller.getSliderValue(gScanSliderIdx) + cvScan); // open curve — clamp, don't wrap

		float out = gSpline.evaluate(scanPosition);
		gEffOut = out;
		gEffScanPosition = scanPosition;

		if(AUDIO_OUT_OUT < context->audioOutChannels) {
			audioWrite(context, frame, AUDIO_OUT_OUT, out);
		}
		if(AUDIO_OUT_SCAN_POSITION < context->audioOutChannels) {
			// The clamped, post-bias/attenuverter position actually used to read the
			// knots — feed this into another Spline's SCAN IN to chain scan positions
			// exactly, regardless of each module's own SCAN attenuverter/bias setting.
			audioWrite(context, frame, AUDIO_OUT_SCAN_POSITION, scanPosition);
		}
	}
}

void cleanup(BelaContext *context, void *userData) {}
