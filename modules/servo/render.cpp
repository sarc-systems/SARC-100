#ifdef __INTELLISENSE__
#include "./stubs/Bela.h"
#include "./stubs/Gui.h"
#include "./stubs/GuiController.h"
#include "../../lib/dsp/envelope.h"
#include "../../lib/dsp/pid.h"
#include "../../lib/dsp/confidence.h"
#else
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <unistd.h>
#include "lib/dsp/envelope.h"
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

// Diagnostic toggle — when 1, AUDIO_OUT_EFFECTOR carries a slow test LFO instead of the real
// effector signal, to check whether that channel is actually DC-coupled: a slow CV-like swing
// should hold cleanly, whereas AC coupling would roll it off / pull it back toward center.
// Confirmed DC-coupled (0.2Hz LFO held cleanly) — leave at 0; flip to 1 only to re-test.
#define SERVO_EFFECTOR_LFO_TEST_ENABLED 0
#if SERVO_EFFECTOR_LFO_TEST_ENABLED
const float kEffectorLfoTestHz = 0.2f;
#endif
float gEffectorLfoTestPhase = 0.0f;

// Diagnostic toggle — when 1, AUDIO_OUT_EFFECTOR carries the raw Ref In signal (pre-envelope-
// follower) instead of the real effector signal, as a patch-through test of the Ref In -> ADC
// -> ... -> DAC -> Effector Out path. Mutually exclusive with the LFO test above. Flip back
// to 0 to restore the real effector output.
#define SERVO_REF_TO_EFFECTOR_TEST_ENABLED 0

Gui gui;
GuiController controller;

unsigned int gEnvRateSliderIdx;
unsigned int gServoRateSliderIdx;
unsigned int gPidKpSliderIdx;
unsigned int gEffectorBipolarSliderIdx;
unsigned int gResetModeSliderIdx;

// Display-only readouts — written only from the aux meter task (setSliderValue
// is not RT-safe), never read back as control input. Mirrors empath's
// *DisplaySliderIdx convention.
unsigned int gEffectorDisplaySliderIdx;
unsigned int gErrorDisplaySliderIdx;
unsigned int gConfidenceDisplaySliderIdx;
unsigned int gMagniRefDisplaySliderIdx;
unsigned int gMagniSigDisplaySliderIdx;

// Raw-CV input meters (A2-A4) — these three have no envelope/processing stage, so the meter
// is the literal analogRead value. REF/SIG's processed form (MAGNI) is one of the "(eff)"
// output meters above instead, since MAGNI is now an official CV output, not just a diagnostic.
unsigned int gEnvRateInDisplaySliderIdx;
unsigned int gServoRateInDisplaySliderIdx;
unsigned int gTargetInDisplaySliderIdx;

struct ServoChannel {
	EnvelopeFollower refEnv;
	EnvelopeFollower sigEnv;
	EnvelopeFollower slewEnv;   // smooths |d(error)/dt| for the CONFIDENCE stability metric
	PID pid;
	Confidence confidence;
	float lastError = 0.0f;
};

ServoChannel gServo[NUM_SERVO_CHANNELS];

float gEffEffector[NUM_SERVO_CHANNELS]   = {0.0f};
float gEffError[NUM_SERVO_CHANNELS]      = {0.0f};
float gEffConfidence[NUM_SERVO_CHANNELS] = {0.0f};
float gEffMagniRef[NUM_SERVO_CHANNELS]   = {0.0f};
float gEffMagniSig[NUM_SERVO_CHANNELS]   = {0.0f};

float gEnvRateIn = 0.0f;
float gServoRateIn = 0.0f;
float gTargetIn = 0.0f;

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

// Loop (SERVO RATE) timescale: maps the 0..1 SERVO RATE control to a time constant,
// log-shaped like a typical CV response curve so most of the slider's travel sits in a
// usable middle range. SERVO RATE=0 -> slow loop (kLoopTimeConstMax), =1 -> fast loop
// (kLoopTimeConstMin). ki and the derivative filter both scale off the same time constant
// by fixed design-time ratios (kKiPerRate/kKdPerRate) — kp does not scale with SERVO RATE.
const float kLoopTimeConstMin = 0.01f;
const float kLoopTimeConstMax = 2.0f;
const float kLoopLogK = 4.0f;
const float kKiPerRate = 0.6f;
const float kKdPerRate = 0.05f;

inline float mapLoopTimeConstant(float rateSlider0to1) {
	float t = clamp01(rateSlider0to1);
	float shaped = (expf(kLoopLogK * (1.0f - t)) - 1.0f) / (expf(kLoopLogK) - 1.0f);
	return kLoopTimeConstMin + (kLoopTimeConstMax - kLoopTimeConstMin) * shaped;
}

// Envelope-follower (ENV RATE) timescale for MAGNI(ref)/MAGNI(sig) — independent of SERVO
// RATE (do NOT gang them together). Min time constant sets the fastest the follower can
// track: 0.008s -> ~20Hz cutoff, the low end of audio, so even at its fastest it's reading a
// slow envelope rather than following individual cycles of a low/mid-range tone.
const float kEnvRateTimeConstMin = 0.008f;
const float kEnvRateTimeConstMax = 0.5f;
const float kEnvRateLogK = 4.0f;

inline float mapEnvRateTimeConstant(float envSlider0to1) {
	float t = clamp01(envSlider0to1);
	float shaped = (expf(kEnvRateLogK * (1.0f - t)) - 1.0f) / (expf(kEnvRateLogK) - 1.0f);
	return kEnvRateTimeConstMin + (kEnvRateTimeConstMax - kEnvRateTimeConstMin) * shaped;
}

// CONFIDENCE requires error to be BOTH small AND steady — magnitude alone (the old "tracking
// success" measure) isn't enough, and neither is slew alone: a large but perfectly steady
// error must NOT read as confident. Computed as magnitudeScore * stabilityScore, so either
// factor being low drags the whole thing down. Internal constants, not CV/GUI-exposed:
//  - kConfidenceErrorTolerance: |error| at which magnitudeScore hits 0.
//  - kConfidenceSlewTimeConstS: fixed smoothing on the raw per-sample |d(error)/dt| estimate,
//    so single-sample audio-rate noise doesn't read as "unstable." Not exposed as CV/GUI —
//    there's no Governor-Time-Mult-style input for it in the new spec.
//  - kConfidenceSlewReferenceRate: an error-swing rate (error-units/second) at which
//    stabilityScore hits 0. Picked so a full -1..1 swing (2.0 units) happening in half a
//    second maps to 0; tune after listening on hardware.
//  - kConfidenceRateFraction: CONFIDENCE's own leaky-integrator rate, as a fixed fraction of
//    the loop's per-sample rate — replaces the removed GOVERNOR TIME MULT CV, keeping
//    confidence provably slower than the loop without exposing a separate control for it.
const float kConfidenceErrorTolerance = 0.3f;
const float kConfidenceSlewTimeConstS = 0.02f;
const float kConfidenceSlewReferenceRate = 4.0f;
const float kConfidenceRateFraction = 0.1f;
float gConfidenceSlewCoeff = 1.0f;

// RESET (trigger): clears the PID integrator; resetMode only changes what the effector does.
// kResetModeZero -> neutral. kResetModeLast -> held at its current value, integrator
// re-seeded so resumption is bumpless (see PID::reset(seedOutput, currentError)). CONFIDENCE
// is NOT cleared here — per the new spec it's a standalone, always-live observation of
// ERROR's behavior, decoupled from the PID/EFF stage that RESET targets.
void resetChannel(ServoChannel &s, int resetMode) {
	if(resetMode == kResetModeLast) {
		float heldOutput = s.pid.lastOutput();
		s.pid.reset(heldOutput, s.lastError);
	} else {
		s.pid.reset();
	}
}

// Lower-priority task: GUI setSliderValue is not RT-safe — never call from render().
void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		controller.setSliderValue(gEffectorDisplaySliderIdx,   gEffEffector[0]);
		controller.setSliderValue(gErrorDisplaySliderIdx,      gEffError[0]);
		controller.setSliderValue(gConfidenceDisplaySliderIdx, gEffConfidence[0]);
		controller.setSliderValue(gMagniRefDisplaySliderIdx,   gEffMagniRef[0]);
		controller.setSliderValue(gMagniSigDisplaySliderIdx,   gEffMagniSig[0]);

		controller.setSliderValue(gEnvRateInDisplaySliderIdx,   gEnvRateIn);
		controller.setSliderValue(gServoRateInDisplaySliderIdx, gServoRateIn);
		controller.setSliderValue(gTargetInDisplaySliderIdx,    gTargetIn);
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

	gEnvRateSliderIdx          = controller.addSlider("Env Rate",                     0.5, 0.0, 1.0, 0.001);
	gServoRateSliderIdx        = controller.addSlider("Servo Rate",                   0.5, 0.0, 1.0, 0.001);
	gPidKpSliderIdx            = controller.addSlider("PID Kp",                       1.0, 0.0, 5.0, 0.001);
	gEffectorBipolarSliderIdx  = controller.addSlider("Effector Bipolar (0/1)",        1.0, 0.0, 1.0, 1.0);
	gResetModeSliderIdx        = controller.addSlider("Reset Mode (0=zero/1=last)",    0.0, 0.0, 1.0, 1.0);

	gEffectorDisplaySliderIdx   = controller.addSlider("Effector (eff)",   0.0, -1.0, 1.0, 0.001);
	gErrorDisplaySliderIdx      = controller.addSlider("Error (eff)",      0.0, -1.0, 1.0, 0.001);
	gConfidenceDisplaySliderIdx = controller.addSlider("Confidence (eff)", 0.0,  0.0, 1.0, 0.001);
	gMagniRefDisplaySliderIdx   = controller.addSlider("Magni Ref (eff)",  0.0,  0.0, 1.0, 0.001);
	gMagniSigDisplaySliderIdx   = controller.addSlider("Magni Sig (eff)",  0.0,  0.0, 1.0, 0.001);

	gTargetInDisplaySliderIdx    = controller.addSlider("Target IN (A0)",     0.0, 0.0, 1.0, 0.001);
	gEnvRateInDisplaySliderIdx   = controller.addSlider("Env Rate IN (A3)",   0.0, 0.0, 1.0, 0.001);
	gServoRateInDisplaySliderIdx = controller.addSlider("Servo Rate IN (A4)", 0.0, 0.0, 1.0, 0.001);

	pinMode(context, 0, DIGITAL_FREEZE, INPUT);
	pinMode(context, 0, DIGITAL_RESET, INPUT);

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);

	gConfidenceSlewCoeff = 1.0f - expf(-1.0f / (kConfidenceSlewTimeConstS * context->audioSampleRate));
	for(int ch = 0; ch < NUM_SERVO_CHANNELS; ch++) {
		gServo[ch].pid.setOutputLimits(-1.0f, 1.0f);
		gServo[ch].slewEnv.setCoeffs(gConfidenceSlewCoeff, gConfidenceSlewCoeff);
	}

	return true;
}

void render(BelaContext *context, void *userData) {
	// Block-rate CVs (GUI slider + CV summed: knob sets a base value, an unpatched jack
	// contributes nothing). TARGET has no GUI base value — like REF/SIG, it's a signal, not
	// a rate/shape parameter — so it's pure CV, read per-frame below alongside REF/SIG.
	float cvEnvRate = 0.0f, cvServoRate = 0.0f;
	if(context->analogFrames > 0) {
		if(ANALOG_ENV_RATE_CV < context->analogInChannels) cvEnvRate = analogRead(context, 0, ANALOG_ENV_RATE_CV);
		if(ANALOG_SERVO_RATE_CV < context->analogInChannels) cvServoRate = analogRead(context, 0, ANALOG_SERVO_RATE_CV);
	}
	gEnvRateIn = cvEnvRate;
	gServoRateIn = cvServoRate;

	float envRate   = clamp01(controller.getSliderValue(gEnvRateSliderIdx) + cvEnvRate);
	float servoRate = clamp01(controller.getSliderValue(gServoRateSliderIdx) + cvServoRate);
	float kp = controller.getSliderValue(gPidKpSliderIdx);
	bool bipolar = controller.getSliderValue(gEffectorBipolarSliderIdx) >= 0.5f;
	int resetMode = (controller.getSliderValue(gResetModeSliderIdx) >= 0.5f) ? kResetModeLast : kResetModeZero;

	float samplePeriod = 1.0f / (float)context->audioSampleRate;

	float loopTimeConst = mapLoopTimeConstant(servoRate);
	float loopRate = 1.0f / loopTimeConst;
	float ki = kKiPerRate * loopRate;
	float kd = kKdPerRate * loopRate;
	float dFilterCoeff = 1.0f - expf(-samplePeriod / loopTimeConst);

	float loopRatePerSample = samplePeriod / loopTimeConst;
	float confidenceRate = loopRatePerSample * kConfidenceRateFraction;

	float envTimeConst = mapEnvRateTimeConstant(envRate);
	float envCoeff = 1.0f - expf(-samplePeriod / envTimeConst);

	float outMin = bipolar ? -1.0f : 0.0f;
	float outMax = 1.0f;

	for(int ch = 0; ch < NUM_SERVO_CHANNELS; ch++) {
		ServoChannel &s = gServo[ch];
		s.pid.setGains(kp, ki, kd);
		s.pid.setDerivativeFilter(dFilterCoeff);
		s.pid.setOutputLimits(outMin, outMax);
		s.refEnv.setCoeffs(envCoeff, envCoeff);
		s.sigEnv.setCoeffs(envCoeff, envCoeff);
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

#if SERVO_EFFECTOR_LFO_TEST_ENABLED
		gEffectorLfoTestPhase += 2.0f * M_PI * kEffectorLfoTestHz / (float)context->audioSampleRate;
		if(gEffectorLfoTestPhase >= 2.0f * M_PI)
			gEffectorLfoTestPhase -= 2.0f * M_PI;
		float effectorLfoTest = 0.5f + 0.5f * sinf(gEffectorLfoTestPhase);
#endif

		for(int ch = 0; ch < NUM_SERVO_CHANNELS; ch++) {
			ServoChannel &s = gServo[ch];

			float ref = (context->analogFrames > 0 && ANALOG_REF_CV < context->analogInChannels) ? analogRead(context, af, ANALOG_REF_CV) : 0.0f;
			float sig = (context->analogFrames > 0 && ANALOG_SIG_CV < context->analogInChannels) ? analogRead(context, af, ANALOG_SIG_CV) : 0.0f;
			float target = (context->analogFrames > 0 && ANALOG_TARGET_CV < context->analogInChannels) ? analogRead(context, af, ANALOG_TARGET_CV) : 0.0f;
			gTargetIn = target;

			// MAGNI(ref)/MAGNI(sig): independent outputs, no internal routing to TARGET or
			// ERROR (per spec) — REF's envelope has no other use in this module.
			float magniRef = s.refEnv.process(ref);
			float magniSig = s.sigEnv.process(sig);
			gEffMagniRef[ch] = magniRef;
			gEffMagniSig[ch] = magniSig;

			// ERROR = TARGET - SIG, raw deviation — uses the unprocessed jacks, not MAGNI.
			float error = target - sig;

			// CONFIDENCE: magnitude AND stability, parallel to EFF, not gated by FREEZE and
			// not fed back into ERROR/EFF. Large error -> low confidence even if steady;
			// jittery error -> low confidence even if small.
			float magnitudeScore = 1.0f - clamp01(fabsf(error) / kConfidenceErrorTolerance);
			float rawSlewRate = (error - s.lastError) / samplePeriod;
			float smoothedSlewRate = s.slewEnv.process(rawSlewRate);
			float stabilityScore = clamp01(1.0f - smoothedSlewRate / kConfidenceSlewReferenceRate);
			float confidenceInput = magnitudeScore * stabilityScore;
			float confidence = s.confidence.update(confidenceInput, confidenceRate);
			s.lastError = error;

			// EFF = ERROR through the PID stage. FREEZE only gates this stage's output —
			// ERROR/MAGNI/CONFIDENCE above keep updating live regardless.
			float effector = freezeActive ? s.pid.freeze() : s.pid.update(error, samplePeriod);

			gEffEffector[ch] = effector;
			gEffError[ch]    = error;
			gEffConfidence[ch] = confidence;

			if(AUDIO_OUT_EFFECTOR < context->audioOutChannels) {
#if SERVO_EFFECTOR_LFO_TEST_ENABLED
				audioWrite(context, frame, AUDIO_OUT_EFFECTOR, effectorLfoTest);
#elif SERVO_REF_TO_EFFECTOR_TEST_ENABLED
				audioWrite(context, frame, AUDIO_OUT_EFFECTOR, ref);
#else
				float effectorWrite = bipolar ? (effector * 0.5f + 0.5f) : effector;
				audioWrite(context, frame, AUDIO_OUT_EFFECTOR, effectorWrite);
#endif
			}
			if(AUDIO_OUT_ERROR < context->audioOutChannels) {
				audioWrite(context, frame, AUDIO_OUT_ERROR, error * 0.5f + 0.5f);
			}
			if(AUDIO_OUT_CONFIDENCE < context->audioOutChannels) {
				audioWrite(context, frame, AUDIO_OUT_CONFIDENCE, confidence);
			}
			if(AUDIO_OUT_MAGNI_REF < context->audioOutChannels) {
				audioWrite(context, frame, AUDIO_OUT_MAGNI_REF, magniRef);
			}
			if(AUDIO_OUT_MAGNI_SIG < context->audioOutChannels) {
				audioWrite(context, frame, AUDIO_OUT_MAGNI_SIG, magniSig);
			}
		}
	}
}

void cleanup(BelaContext *context, void *userData) {}
