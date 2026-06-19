#ifdef __INTELLISENSE__
#include "./stubs/Bela.h"
#include "./stubs/Gui.h"
#include "./stubs/GuiController.h"
#include "../../lib/dsp/envelope.h"
#include "../../lib/dsp/spline.h"
#include "../../lib/dsp/pid.h"
#include "../../lib/dsp/confidence.h"
#else
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <unistd.h>
#include "lib/dsp/envelope.h"
#include "lib/dsp/spline.h"
#include "lib/dsp/pid.h"
#include "lib/dsp/confidence.h"
#endif
#include <stdlib.h>
#include <math.h>
#include <float.h>
#ifdef __INTELLISENSE__
#include "./stubs/math_neon.h"
#else
#include <libraries/math_neon/math_neon.h>
#endif

#include "pins.h"

// Designed to be used in multiples and coupled in feedback networks (see
// CLAUDE.md) — nothing here assumes a single global instance. One channel
// for now; raise this once REF/SIG/CV pin assignment is confirmed for
// additional instances.
#define NUM_SERVO_CHANNELS 1

Gui gui;
GuiController controller;

unsigned int gSpline1SliderIdx, gSpline2SliderIdx, gSpline3SliderIdx, gSpline4SliderIdx, gSpline5SliderIdx;
unsigned int gTensionSliderIdx;
unsigned int gTimeSliderIdx;
unsigned int gGovernorTimeMultSliderIdx;
unsigned int gEnvResponseSliderIdx;
unsigned int gPidKpSliderIdx;
unsigned int gEffectorInvertSliderIdx;
unsigned int gEffectorBipolarSliderIdx;
unsigned int gResetModeSliderIdx;

// Display-only readouts — written only from the aux meter task (setSliderValue
// is not RT-safe), never read back as control input. Mirrors empath's
// *DisplaySliderIdx convention.
unsigned int gEffectorDisplaySliderIdx;
unsigned int gErrorDisplaySliderIdx;
unsigned int gConfidenceDisplaySliderIdx;

struct ServoChannel {
	EnvelopeFollower refEnv;
	EnvelopeFollower sigEnv;
	Spline5 spline;
	PID pid;
	Confidence confidence;
	float lastError = 0.0f;
};

ServoChannel gServo[NUM_SERVO_CHANNELS];

float gEffEffector[NUM_SERVO_CHANNELS]   = {0.0f};
float gEffError[NUM_SERVO_CHANNELS]      = {0.0f};
float gEffConfidence[NUM_SERVO_CHANNELS] = {0.0f};

int gResetPrev = 0;

const int kResetModeZero = 0;
const int kResetModeLast = 1;

inline float clamp01(float x) {
	return fminf(fmaxf(x, 0.0f), 1.0f);
}

inline int analogFrameForAudio(BelaContext *context, int audioFrame) {
	if(context->analogFrames == 0) return 0;
	return (audioFrame * (int)context->analogFrames) / (int)context->audioFrames;
}

// Loop (TIME) timescale: maps the 0..1 TIME control to a time constant,
// log-shaped like a typical CV response curve so most of the slider's
// travel sits in a usable middle range. TIME=0 -> slow loop
// (kLoopTimeConstMax), TIME=1 -> fast loop (kLoopTimeConstMin). ki and the
// derivative filter both scale off the same time constant by fixed
// design-time ratios (kKiPerRate/kKdPerRate), per CLAUDE.md's "Two
// timescales" — kp does not scale with TIME (the seed says TIME governs
// "PID I + D" only).
const float kLoopTimeConstMin = 0.01f;
const float kLoopTimeConstMax = 2.0f;
const float kLoopLogK = 4.0f;
const float kKiPerRate = 0.6f;
const float kKdPerRate = 0.05f;

inline float mapLoopTimeConstant(float timeSlider0to1) {
	float t = clamp01(timeSlider0to1);
	float shaped = (expf(kLoopLogK * (1.0f - t)) - 1.0f) / (expf(kLoopLogK) - 1.0f);
	return kLoopTimeConstMin + (kLoopTimeConstMax - kLoopTimeConstMin) * shaped;
}

// Confidence (GOVERNOR TIME MULT) timescale: confidenceRate is always a
// strict fraction of the loop's own per-sample rate (kGovernorMultMax < 1),
// so confidence is provably slower than the loop by construction, not just
// by typical tuning — per CLAUDE.md's "Confidence must be slower than the
// loop."
const float kGovernorMultMin = 0.01f;
const float kGovernorMultMax = 0.5f;

inline float mapGovernorTimeMult(float governorSlider0to1) {
	float t = clamp01(governorSlider0to1);
	return kGovernorMultMin + (kGovernorMultMax - kGovernorMultMin) * t;
}

// Envelope-follower (ENV RESPONSE) timescale — independent of TIME (see
// "Two timescales": "do NOT gang them to TIME").
const float kEnvResponseTimeConstMin = 0.001f;
const float kEnvResponseTimeConstMax = 0.5f;
const float kEnvResponseLogK = 4.0f;

inline float mapEnvResponseTimeConstant(float envSlider0to1) {
	float t = clamp01(envSlider0to1);
	float shaped = (expf(kEnvResponseLogK * (1.0f - t)) - 1.0f) / (expf(kEnvResponseLogK) - 1.0f);
	return kEnvResponseTimeConstMin + (kEnvResponseTimeConstMax - kEnvResponseTimeConstMin) * shaped;
}

// "Sustained tracking success" for Confidence: 1.0 when error is near zero,
// falling off to 0.0 by kSuccessErrorTolerance away from target. Not part
// of the seed's CV/GUI surface — an internal constant, like empath's energy
// reserve/coupling memory.
const float kSuccessErrorTolerance = 0.3f;

inline float successFromError(float error) {
	return 1.0f - clamp01(fabsf(error) / kSuccessErrorTolerance);
}

// RESET (trigger): always clears integrator + confidence; resetMode only
// changes what the effector does. kResetModeZero -> neutral. kResetModeLast
// -> held at its current value, integrator re-seeded so resumption is
// bumpless (see PID::reset(seedOutput, currentError)).
void resetChannel(ServoChannel &s, int resetMode) {
	if(resetMode == kResetModeLast) {
		float heldOutput = s.pid.lastOutput();
		s.pid.reset(heldOutput, s.lastError);
	} else {
		s.pid.reset();
	}
	s.confidence.reset(0.0f);
}

// Lower-priority task: GUI setSliderValue is not RT-safe — never call from render().
void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		controller.setSliderValue(gEffectorDisplaySliderIdx,   gEffEffector[0]);
		controller.setSliderValue(gErrorDisplaySliderIdx,      gEffError[0]);
		controller.setSliderValue(gConfidenceDisplaySliderIdx, gEffConfidence[0]);
#ifdef __INTELLISENSE__
		for(volatile int d = 0; d < 80000; ++d) {}
#else
		usleep(80000);
#endif
	}
}

bool setup(BelaContext *context, void *userData) {
	gui.setup(context->projectName);
	controller.setup(&gui, "Servo");

	gSpline1SliderIdx = controller.addSlider("Spline 1", 0.0,  0.0, 1.0, 0.001);
	gSpline2SliderIdx = controller.addSlider("Spline 2", 0.25, 0.0, 1.0, 0.001);
	gSpline3SliderIdx = controller.addSlider("Spline 3", 0.5,  0.0, 1.0, 0.001);
	gSpline4SliderIdx = controller.addSlider("Spline 4", 0.75, 0.0, 1.0, 0.001);
	gSpline5SliderIdx = controller.addSlider("Spline 5", 1.0,  0.0, 1.0, 0.001);
	gTensionSliderIdx = controller.addSlider("Tension",  0.0,  0.0, 1.0, 0.001);

	gTimeSliderIdx             = controller.addSlider("Time",                       0.5, 0.0, 1.0, 0.001);
	gGovernorTimeMultSliderIdx = controller.addSlider("Governor Time Mult",         0.5, 0.0, 1.0, 0.001);
	gEnvResponseSliderIdx      = controller.addSlider("Env Response",               0.5, 0.0, 1.0, 0.001);
	gPidKpSliderIdx            = controller.addSlider("PID Kp",                     1.0, 0.0, 5.0, 0.001);
	gEffectorInvertSliderIdx   = controller.addSlider("Effector Invert (0/1)",      0.0, 0.0, 1.0, 1.0);
	gEffectorBipolarSliderIdx  = controller.addSlider("Effector Bipolar (0/1)",     1.0, 0.0, 1.0, 1.0);
	gResetModeSliderIdx        = controller.addSlider("Reset Mode (0=zero/1=last)", 0.0, 0.0, 1.0, 1.0);

	gEffectorDisplaySliderIdx   = controller.addSlider("Effector (eff)",   0.0, -1.0, 1.0, 0.001);
	gErrorDisplaySliderIdx      = controller.addSlider("Error (eff)",      0.0, -1.0, 1.0, 0.001);
	gConfidenceDisplaySliderIdx = controller.addSlider("Confidence (eff)", 0.0,  0.0, 1.0, 0.001);

	pinMode(context, 0, DIGITAL_FREEZE, INPUT);
	pinMode(context, 0, DIGITAL_RESET, INPUT);

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);

	float identityKnots[5] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
	for(int ch = 0; ch < NUM_SERVO_CHANNELS; ch++) {
		gServo[ch].spline.setKnots(identityKnots);
		gServo[ch].pid.setOutputLimits(-1.0f, 1.0f);
	}

	return true;
}

void render(BelaContext *context, void *userData) {
	// Block-rate CVs (GUI slider + CV summed: knob sets a base value, an
	// unpatched/placeholder CV jack contributes nothing). Every channel
	// index here is guarded against analogInChannels since several exceed
	// the board's 8 standard analog ins (see pins.h).
	float cvSpline1 = 0.0f, cvSpline2 = 0.0f, cvSpline3 = 0.0f, cvSpline4 = 0.0f, cvSpline5 = 0.0f;
	float cvTension = 0.0f, cvTime = 0.0f, cvGovernorMult = 0.0f, cvEnvResponse = 0.0f;
	if(context->analogFrames > 0) {
		if(ANALOG_SPLINE1_CV < context->analogInChannels) cvSpline1 = analogRead(context, 0, ANALOG_SPLINE1_CV);
		if(ANALOG_SPLINE2_CV < context->analogInChannels) cvSpline2 = analogRead(context, 0, ANALOG_SPLINE2_CV);
		if(ANALOG_SPLINE3_CV < context->analogInChannels) cvSpline3 = analogRead(context, 0, ANALOG_SPLINE3_CV);
		if(ANALOG_SPLINE4_CV < context->analogInChannels) cvSpline4 = analogRead(context, 0, ANALOG_SPLINE4_CV);
		if(ANALOG_SPLINE5_CV < context->analogInChannels) cvSpline5 = analogRead(context, 0, ANALOG_SPLINE5_CV);
		if(ANALOG_TENSION_CV < context->analogInChannels) cvTension = analogRead(context, 0, ANALOG_TENSION_CV);
		if(ANALOG_TIME_CV < context->analogInChannels) cvTime = analogRead(context, 0, ANALOG_TIME_CV);
		if(ANALOG_GOVERNOR_TIME_MULT_CV < context->analogInChannels) cvGovernorMult = analogRead(context, 0, ANALOG_GOVERNOR_TIME_MULT_CV);
		if(ANALOG_ENV_RESPONSE_CV < context->analogInChannels) cvEnvResponse = analogRead(context, 0, ANALOG_ENV_RESPONSE_CV);
	}

	float spline1 = clamp01(controller.getSliderValue(gSpline1SliderIdx) + cvSpline1);
	float spline2 = clamp01(controller.getSliderValue(gSpline2SliderIdx) + cvSpline2);
	float spline3 = clamp01(controller.getSliderValue(gSpline3SliderIdx) + cvSpline3);
	float spline4 = clamp01(controller.getSliderValue(gSpline4SliderIdx) + cvSpline4);
	float spline5 = clamp01(controller.getSliderValue(gSpline5SliderIdx) + cvSpline5);
	float tension = clamp01(controller.getSliderValue(gTensionSliderIdx) + cvTension);
	float time           = clamp01(controller.getSliderValue(gTimeSliderIdx) + cvTime);
	float governorMult    = clamp01(controller.getSliderValue(gGovernorTimeMultSliderIdx) + cvGovernorMult);
	float envResponse      = clamp01(controller.getSliderValue(gEnvResponseSliderIdx) + cvEnvResponse);
	float kp = controller.getSliderValue(gPidKpSliderIdx);
	bool invert  = controller.getSliderValue(gEffectorInvertSliderIdx) >= 0.5f;
	bool bipolar = controller.getSliderValue(gEffectorBipolarSliderIdx) >= 0.5f;
	int resetMode = (controller.getSliderValue(gResetModeSliderIdx) >= 0.5f) ? kResetModeLast : kResetModeZero;

	float samplePeriod = 1.0f / (float)context->audioSampleRate;

	float loopTimeConst = mapLoopTimeConstant(time);
	float loopRate = 1.0f / loopTimeConst;
	float ki = kKiPerRate * loopRate;
	float kd = kKdPerRate * loopRate;
	float dFilterCoeff = 1.0f - expf(-samplePeriod / loopTimeConst);

	float loopRatePerSample = samplePeriod / loopTimeConst;
	float confidenceRate = loopRatePerSample * mapGovernorTimeMult(governorMult);

	float envTimeConst = mapEnvResponseTimeConstant(envResponse);
	float envCoeff = 1.0f - expf(-samplePeriod / envTimeConst);

	float outMin = bipolar ? -1.0f : 0.0f;
	float outMax = 1.0f;

	float splineKnots[5] = {spline1, spline2, spline3, spline4, spline5};

	for(int ch = 0; ch < NUM_SERVO_CHANNELS; ch++) {
		ServoChannel &s = gServo[ch];
		s.pid.setGains(kp, ki, kd);
		s.pid.setDerivativeFilter(dFilterCoeff);
		s.pid.setOutputLimits(outMin, outMax);
		s.refEnv.setCoeffs(envCoeff, envCoeff);
		s.sigEnv.setCoeffs(envCoeff, envCoeff);
		s.spline.setKnots(splineKnots);
		s.spline.setTension(tension);
	}

	// FREEZE is level-sensitive (hold while high); RESET is edge-triggered.
	// Mirrors empath's gSyncPrev edge-detect block: digital handling stays
	// at block granularity even though digitalFrames may be finer.
	bool freezeActive = false;
	for(unsigned int n = 0; n < context->digitalFrames; n++) {
		int freeze = digitalRead(context, n, DIGITAL_FREEZE);
		int reset  = digitalRead(context, n, DIGITAL_RESET);
		if(freeze) freezeActive = true;
		if(reset && !gResetPrev) {
			for(int ch = 0; ch < NUM_SERVO_CHANNELS; ch++)
				resetChannel(gServo[ch], resetMode);
		}
		gResetPrev = reset ? 1 : 0;
	}

	for(int frame = 0; frame < context->audioFrames; frame++) {
		int af = (context->analogFrames > 0) ? analogFrameForAudio(context, frame) : 0;

		for(int ch = 0; ch < NUM_SERVO_CHANNELS; ch++) {
			ServoChannel &s = gServo[ch];

			float ref = (AUDIO_IN_REF < context->audioInChannels) ? audioRead(context, frame, AUDIO_IN_REF) : 0.0f;
			float sig = (AUDIO_IN_SIG < context->audioInChannels) ? audioRead(context, frame, AUDIO_IN_SIG) : 0.0f;

			float refEnvVal = s.refEnv.process(ref);
			float sigEnvVal = s.sigEnv.process(sig);

			float target = s.spline.evaluate(clamp01(refEnvVal));
			float rawError = target - sigEnvVal;
			float error = invert ? -rawError : rawError;
			s.lastError = error;

			float effector, confidence;
			if(freezeActive) {
				effector = s.pid.freeze();
				confidence = s.confidence.value();
			} else {
				effector = s.pid.update(error, samplePeriod);
				confidence = s.confidence.update(successFromError(error), confidenceRate);
			}

			gEffEffector[ch]   = effector;
			gEffError[ch]      = error;
			gEffConfidence[ch] = confidence;

			if(context->analogFrames > 0 && ANALOG_OUT_EFFECTOR < context->analogOutChannels) {
				float effectorWrite = bipolar ? (effector * 0.5f + 0.5f) : effector;
				analogWrite(context, af, ANALOG_OUT_EFFECTOR, effectorWrite);
			}
			if(context->analogFrames > 0 && ANALOG_OUT_ERROR < context->analogOutChannels) {
				analogWrite(context, af, ANALOG_OUT_ERROR, error * 0.5f + 0.5f);
			}
			if(context->analogFrames > 0 && ANALOG_OUT_CONFIDENCE < context->analogOutChannels) {
				analogWrite(context, af, ANALOG_OUT_CONFIDENCE, confidence);
			}
		}
	}
}

void cleanup(BelaContext *context, void *userData) {}
