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

// No GUI control sliders — all parameters are CV-driven (the hardware front-end sums each
// input's att + bias/offset into the CV before the ADC). The GUI is display-only.

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

// Bipolar conventions — segment values and OUT are bipolar ±full-scale (0.5 = 0V),
// matching the rest of the system. SCAN and INTERPOLATE stay unipolar [0,1] (they are a
// read position and a morph amount, not bipolar values).
inline float clampBip(float x)  { return fminf(fmaxf(x, -1.0f), 1.0f); }  // clamp to [-1,1]
inline float bipolarIn(float v) { return 2.0f * v - 1.0f; }               // ADC [0,1] -> [-1,1]
inline float toDcOut(float x)   { return clamp01(0.5f + 0.5f * x); }       // [-1,1] -> DAC [0,1]

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

	gOutDisplaySliderIdx          = controller.addSlider("Out (eff)",           0.0, -1.0, 1.0, 0.001);
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
	// Block-rate CVs (CV-only; the hardware front-end applies att + bias/offset before the
	// ADC). Segments and INTERPOLATE change slowly — no need for audio-rate reads. SCAN is
	// read per-audio-frame below, since it's the one input expected to move fast.
	for(int i = 0; i < NUM_SEGMENTS; i++) {
		float cv = 0.0f;
		if(context->analogFrames > 0 && kSegmentAnalogIn[i] < (int)context->analogInChannels)
			cv = analogRead(context, 0, kSegmentAnalogIn[i]);
		gSegmentIn[i] = cv;
		// Bipolar knot from bipolar CV (0V = ADC mid-scale, like the other bipolar modules).
		gSpline.setKnot(i, clampBip(bipolarIn(cv)));
	}

	float cvInterpolate = 0.0f;
	if(context->analogFrames > 0 && ANALOG_INTERPOLATE_CV < context->analogInChannels)
		cvInterpolate = analogRead(context, 0, ANALOG_INTERPOLATE_CV);
	gInterpolateIn = cvInterpolate;
	gSpline.setInterpolation(clamp01(cvInterpolate));   // unipolar [0,1] morph amount

	for(int frame = 0; frame < context->audioFrames; frame++) {
		int af = (context->analogFrames > 0) ? analogFrameForAudio(context, frame) : 0;

		float cvScan = (context->analogFrames > 0 && ANALOG_SCAN_CV < context->analogInChannels)
			? analogRead(context, af, ANALOG_SCAN_CV) : 0.0f;
		gScanIn = cvScan;
		float scanPosition = clamp01(cvScan); // unipolar [0,1] read position; open curve — clamp, don't wrap

		float out = gSpline.evaluate(scanPosition);
		gEffOut = out;
		gEffScanPosition = scanPosition;

		if(AUDIO_OUT_OUT < context->audioOutChannels) {
			// Bipolar value -> DAC (0.5 = 0V). Smooth-mode Catmull-Rom can overshoot the
			// knot range, so toDcOut clamps.
			audioWrite(context, frame, AUDIO_OUT_OUT, toDcOut(out));
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
