#ifdef __INTELLISENSE__
#include "./stubs/Bela.h"
#include "./stubs/Gui.h"
#include "./stubs/GuiController.h"
#include "../../lib/io/cv_input.h"
#include "../../lib/dsp/cycledivide.h"
#include "../../lib/dsp/sternbrocot.h"
#else
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <unistd.h>
#include "lib/io/cv_input.h"
#include "lib/dsp/cycledivide.h"
#include "lib/dsp/sternbrocot.h"
#endif
#include <math.h>
#ifdef __INTELLISENSE__
#include "./stubs/math_neon.h"
#else
#include <libraries/math_neon/math_neon.h>
#endif

#include "pins.h"

// ─────────────────────────────────────────────────────────────────────────────
// DRIVETRAIN — master phasor + two exact-rational sub-oscillators, 3-op PM.
//
// THE CRITICAL INVARIANT (spec §3): phase state advances from TUNE alone.
// Everything downstream is a pure function of the clean master phase. PM is
// additive at the OUTPUT READ only — never accumulated into phase state. Strict
// DAG, no feedback. Do not "improve" this by folding PM into the accumulator.
// ─────────────────────────────────────────────────────────────────────────────

static const double TWO_PI = 2.0 * M_PI;

// ── Calibration constants (spec §9) ─────────────────────────────────────────
const float  F_MIN     = 2.0f;    // Hz at TUNE=0                    [CALIBRATE]
const float  OCT_RANGE = 12.0f;   // octaves spanned by TUNE         [CALIBRATE]
const float  MAX_INDEX = 8.0f;    // rad, full PM depth (>> pi for bright sidebands)
const float  EXP_K     = 3.5f;    // PM index curve steepness
const int    D_MAX     = 14;      // tree build depth — deep enough that the odd/prime limits
                                  //   (not depth) gate the lattice; 9/8 sits at tree depth 9
const int    kPrimeLimit = 7;     // p-limit JI: ratios factor into primes <= 7 (2·3·5·7)
const int    kOddLimit   = 9;     // odd-limit JI: max odd part of p,q <= 9 — the 9-odd-limit
                                  //   diamond (admits 9/8). Odd-limit, not tree depth, sets the
                                  //   density: depth over-penalizes near-unison ratios like 9/8.
const float  R_MIN     = 0.25f;   // ratio range (+/- 2 octaves)
const float  R_MAX     = 4.0f;
const float  HYST_RATIO = 0.03f;  // ratio-select dead-zone, natural-log units (~half semitone)

// Stern-Brocot table sized for depth D_MAX within [R_MIN, R_MAX] (build() guards
// against overflow). ~few thousand nodes; lives in static storage, built in setup().
const int MAX_SB_NODES = 8192;
SternBrocotTable<MAX_SB_NODES> gTable;

// ── Master phase engine state (spec §4) ─────────────────────────────────────
double    gPhiMaster = 0.0;   // [0,1)
long long gC         = 0;     // master-cycle counter
int       gResetPrev = 0;

// ── Sub-oscillators (spec §5/§6) ────────────────────────────────────────────
// Subs have NO independent phase state — they are computed windows onto the master
// (spec §10). The only per-sub state is the latched (p,q), its derived constants,
// the hysteresis index, and the static offset.
struct Sub {
	int    p = 1, q = 1;        // latched ratio (used per-sample)
	int    pt = 1, qt = 1;      // target ratio (computed at control rate)
	double invQ = 1.0;          // 1/q   — recomputed on latch
	double pOverQ = 1.0;        // p/q   — recomputed on latch
	int    curIdx = -1;         // selected Stern-Brocot table index (hysteresis)
	double off = 0.0;           // static phase offset [0,1)
	double phi = 0.0;           // last computed sub phase (display only)
};
Sub gSub[2];

inline void latchSub(Sub &s) {
	s.p = s.pt;
	s.q = s.qt;
	s.invQ   = 1.0 / (double)s.q;
	s.pOverQ = (double)s.p / (double)s.q;
}

// ── Control-rate cached values (computed per block, read per sample) ─────────
double gFMaster = F_MIN;
double gDepth   = 0.0;   // PM index (rad)
double gDA = 0.0, gDB = 0.0;
double gFundNull = 0.0;  // fundamental-null amount [0,1] — depth of the tracking notch

// ── CV inputs (standardized, smoothed — lib/io/cv_input.h) ──────────────────
CvIn cvTune, cvRatioA, cvRatioB, cvFundNull, cvPmDepth, cvPmBalance, cvPhaseA, cvPhaseB;

// ── Helpers ──────────────────────────────────────────────────────────────────
inline float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }
inline double frac1(double x) { return x - floor(x); }

inline double tuneMap(float cv)  { return (double)F_MIN * pow(2.0, (double)clamp01(cv) * OCT_RANGE); }

inline double indexMap(float cv) {
	static const double denom = exp((double)EXP_K) - 1.0;
	return (double)MAX_INDEX * (exp((double)EXP_K * clamp01(cv)) - 1.0) / denom;
}

// Direct-Form-I biquad (double state), used as a frequency-tracking notch at the
// master fundamental for the FUND NULL control.
struct Biquad {
	double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
	double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	inline double process(double x) {
		double y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
		x2 = x1; x1 = x; y2 = y1; y1 = y;
		return y;
	}
	// RBJ notch (Audio EQ Cookbook): a true zero at f0; Q sets the width.
	void setNotch(double f0, double fs, double Q) {
		double w0 = 2.0 * M_PI * f0 / fs;
		double cw = cos(w0), alpha = sin(w0) / (2.0 * Q);
		double a0 = 1.0 + alpha;
		b0 = 1.0 / a0;  b1 = -2.0 * cw / a0;  b2 = 1.0 / a0;
		a1 = -2.0 * cw / a0;  a2 = (1.0 - alpha) / a0;
	}
};
// One notch per carrier output (SINE, COSINE) — separate filter state per signal.
Biquad gNotchSine, gNotchCos;
const double kNotchQ = 30.0;   // notch steepness (higher = narrower, spares more sidebands)

// RATIO cv (log-mapped onto [R_MIN, R_MAX]) -> nearest admitted Stern-Brocot node.
void selectRatio(Sub &s, float ratioCv, int depthCeil) {
	static const float logMin   = logf(R_MIN);
	static const float logRange = logf(R_MAX) - logf(R_MIN);
	float targetLog = logMin + clamp01(ratioCv) * logRange;
	s.curIdx = gTable.select(targetLog, depthCeil, s.curIdx, HYST_RATIO);
	const SBNode &nd = gTable.node(s.curIdx);
	s.pt = nd.p;
	s.qt = nd.q;
}

// ── GUI (display-only meters; controls are the analog front end, spec §2) ────
Gui gui;
GuiController controller;
unsigned int gDispFMaster, gDispFundNull, gDispDepth, gDispBalance;
unsigned int gDispRatioValA, gDispRatioValB;

double gEffBalance = 0.0;

void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		controller.setSliderValue(gDispFMaster, (float)gFMaster);
		controller.setSliderValue(gDispFundNull, (float)gFundNull);
		controller.setSliderValue(gDispDepth,   (float)gDepth);
		controller.setSliderValue(gDispBalance, (float)gEffBalance);
		controller.setSliderValue(gDispRatioValA, (float)gSub[0].p / (float)gSub[0].q);
		controller.setSliderValue(gDispRatioValB, (float)gSub[1].p / (float)gSub[1].q);
#ifdef __INTELLISENSE__
		for(volatile int d = 0; d < 80000; ++d) {}
#else
		usleep(80000);
#endif
	}
}

bool setup(BelaContext *context, void *userData) {
	gui.setup(context->projectName);
	controller.setup(&gui, "Drivetrain");

	gDispFMaster = controller.addSlider("f_master Hz", F_MIN, 0.0, 12000.0, 0.01);
	gDispFundNull = controller.addSlider("fund null",  0.0,   0.0, 1.0, 0.001);
	gDispDepth   = controller.addSlider("PM index",    0.0,   0.0, MAX_INDEX, 0.001);
	gDispBalance = controller.addSlider("PM balance", -0.0,  -1.0, 1.0, 0.001);
	gDispRatioValA = controller.addSlider("ratio A (p/q)", 1.0, 0.0, 4.0, 0.001);
	gDispRatioValB = controller.addSlider("ratio B (p/q)", 1.0, 0.0, 4.0, 0.001);

	gTable.build(R_MIN, R_MAX, D_MAX, kPrimeLimit, kOddLimit);

	cvTune.setup(AN_IN_TUNE, context);
	cvRatioA.setup(AN_IN_RATIO_A, context);
	cvRatioB.setup(AN_IN_RATIO_B, context);
	cvFundNull.setup(AN_IN_FUND_NULL, context);
	cvPmDepth.setup(AN_IN_PM_DEPTH, context);
	cvPmBalance.setup(AN_IN_PM_BALANCE, context);
	cvPhaseA.setup(AN_IN_PHASE_A, context);
	cvPhaseB.setup(AN_IN_PHASE_B, context);

	pinMode(context, 0, DIG_FREEZE, INPUT);
	pinMode(context, 0, DIG_RESET,  INPUT);

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);

	return true;
}

void render(BelaContext *context, void *userData) {
	const double dt = 1.0 / (double)context->audioSampleRate;

	// ── Control rate (per block, spec §8) ───────────────────────────────────
	gFMaster  = tuneMap(cvTune.read(context));
	gFundNull = (double)clamp01(cvFundNull.read(context));   // carrier-null amount

	float phaseACv = cvPhaseA.read(context);
	float phaseBCv = cvPhaseB.read(context);
	// Depth ceiling = D_MAX (all built nodes); the odd/prime limits already gated the set.
	selectRatio(gSub[0], cvRatioA.read(context), D_MAX);  // latched at wrap
	selectRatio(gSub[1], cvRatioB.read(context), D_MAX);
	gDepth   = indexMap(cvPmDepth.read(context));

	double balance = (double)clamp01(cvPmBalance.read(context)) * 2.0 - 1.0;   // [-1,1]
	gEffBalance    = balance;
	double th = (balance + 1.0) * M_PI / 4.0;          // [0, pi/2]
	gDA = gDepth * cos(th);                              // equal-power split:
	gDB = gDepth * sin(th);                              //   dA^2 + dB^2 = depth^2

	// Fundamental-null notch tracks the master frequency. Clamp f0 to a stable range
	// (a biquad notch degrades toward DC/Nyquist). Depth applied per-sample via gFundNull.
	double fs = (double)context->audioSampleRate;
	double notchF0 = fmin(fmax(gFMaster, 10.0), 0.45 * fs);
	gNotchSine.setNotch(notchF0, fs, kNotchQ);
	gNotchCos.setNotch(notchF0, fs, kNotchQ);

	gSub[0].off = (double)clamp01(phaseACv);
	gSub[1].off = (double)clamp01(phaseBCv);

	// ── Per sample ──────────────────────────────────────────────────────────
	for(unsigned int frame = 0; frame < context->audioFrames; ++frame) {
		bool freeze = digitalRead(context, frame, DIG_FREEZE);
		int  reset  = digitalRead(context, frame, DIG_RESET);

		// Master phase: advanced by TUNE only, held by FREEZE, zeroed by RESET.
		// (p,q) latch at the master wrap so a ratio change realigns the sub cleanly.
		if(!freeze) {
			gPhiMaster += gFMaster * dt;
			if(gPhiMaster >= 1.0) {
				gPhiMaster -= 1.0;
				gC += 1;
				latchSub(gSub[0]);
				latchSub(gSub[1]);
			}
		}
		if(reset && !gResetPrev) {
			gPhiMaster = 0.0;
			gC = 0;
			latchSub(gSub[0]);
			latchSub(gSub[1]);
		}
		gResetPrev = reset;

		// Subs: exact cycle-counted rational division of the CLEAN master phase,
		// plus static offset. Continuous across master wraps (lib/dsp/cycledivide).
		double phiA = frac1(cycledivide(gSub[0].p, gSub[0].q, gC, gPhiMaster,
		                                gSub[0].invQ, gSub[0].pOverQ) + gSub[0].off);
		double phiB = frac1(cycledivide(gSub[1].p, gSub[1].q, gC, gPhiMaster,
		                                gSub[1].invQ, gSub[1].pOverQ) + gSub[1].off);
		gSub[0].phi = phiA;
		gSub[1].phi = phiB;

		// 3-operator PM: two sine modulators -> one carrier. PM is additive at the
		// output read ONLY — never written back into phase state (spec §3/§7).
		float modA = sinf_neon((float)(TWO_PI * phiA));
		float modB = sinf_neon((float)(TWO_PI * phiB));
		double arg = TWO_PI * gPhiMaster + gDA * (double)modA + gDB * (double)modB;

		// Fundamental null: a steep notch tracking the master frequency, blended in
		// by gFundNull (0 = dry carrier, 1 = full notch). Removes energy at f_master
		// regardless of source, so it handles the subharmonic-ratio case the J0
		// subtraction couldn't. The notch runs every sample (even at gFundNull=0) to
		// keep its state warm so turning it up doesn't click. It also dents sidebands
		// within ~f_master/Q of the fundamental.
		float pmSine = sinf_neon((float)arg);
		float pmCos  = cosf_neon((float)arg);
		double sN = gNotchSine.process((double)pmSine);
		double cN = gNotchCos.process((double)pmCos);
		float sineOut = pmSine + (float)(gFundNull * (sN - (double)pmSine));
		float cosOut  = pmCos  + (float)(gFundNull * (cN - (double)pmCos));
		sineOut = fmaxf(-1.0f, fminf(1.0f, sineOut));   // guard the DC output range
		cosOut  = fmaxf(-1.0f, fminf(1.0f, cosOut));

		// Outputs (spec §2): sines as 0.5+0.5x (bipolar via DC output stage),
		// ramps written directly (full-rail bipolar sawtooth). These go to the
		// Gem Multi's DC-coupled AUDIO outs via audioWrite (audio-rate, frame index
		// always valid) — analogWrite would index a nonexistent analog-out buffer.
		if(AUDIO_OUT_SINE   < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_SINE,   0.5f + 0.5f * sineOut);
		if(AUDIO_OUT_COSINE < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_COSINE, 0.5f + 0.5f * cosOut);
		if(AUDIO_OUT_SINE_A < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_SINE_A, 0.5f + 0.5f * modA);
		if(AUDIO_OUT_SINE_B < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_SINE_B, 0.5f + 0.5f * modB);
		if(AUDIO_OUT_RAMP_A < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_RAMP_A, (float)phiA);
		if(AUDIO_OUT_RAMP_B < context->audioOutChannels) audioWrite(context, frame, AUDIO_OUT_RAMP_B, (float)phiB);
	}
}

void cleanup(BelaContext *context, void *userData) {}
