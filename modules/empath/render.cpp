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
#include <float.h>
#ifdef __INTELLISENSE__
#include "./stubs/math_neon.h"
#else
#include <libraries/math_neon/math_neon.h>
#endif

#include "pins.h"

// Drift LFOs cost ~28 sinf_neon calls/sample (2 per node per channel) for a sub-0.06Hz,
// near-imperceptible modulation — disabled for now to free up headroom at 96/24. Flip to 1
// to restore; advanceDriftLfo/nodeDriftAmount are untouched, just unreached.
#define EMPATH_DRIFT_ENABLED 0

// Cross-ladder frequency pulling (Adler/Kuramoto phase coupling) — muted for now. Flip to 1
// to restore; the Freq Lock Depth slider, its detune contribution, and the (eff) display
// all stay live, only the actual phase pull into theta[][] is skipped.
#define EMPATH_FREQ_LOCK_ENABLED 1

#define NUM_OSCS 7
#define NUM_CHANNELS 2
#define SPEC_A 0
#define SPEC_B 1

Gui gui;
GuiController controller;
unsigned int gGlobalPhaseSliderIdx;
unsigned int gNodeAttackSliderIdx;
unsigned int gNodeDecaySliderIdx;
unsigned int gFreqLockDepthSliderIdx;
unsigned int gPitchExciteDepthSliderIdx;
unsigned int gPitchDecayDepthSliderIdx;
unsigned int gInPeakASliderIdx;
unsigned int gInPeakBSliderIdx;
unsigned int gOutPeakASliderIdx;
unsigned int gOutPeakBSliderIdx;

// Display-only readouts for effective (GUI + CV) values — never read back as control input,
// so writing to these from meterGuiTask cannot feed back into render()'s parameter computation.
unsigned int gF0DisplaySliderIdx;
unsigned int gDetuneDisplaySliderIdx;
unsigned int gNodeCouplingADisplaySliderIdx;
unsigned int gNodeCouplingBDisplaySliderIdx;
unsigned int gXCoupleAmountDisplaySliderIdx;
unsigned int gXCoupleSymmetryDisplaySliderIdx;
unsigned int gFreqLockDepthDisplaySliderIdx;
unsigned int gOutputScanADisplaySliderIdx;
unsigned int gOutputScanBDisplaySliderIdx;

float gInPeakHold[NUM_CHANNELS] = {0.0f, 0.0f};
float gOutPeakHold[NUM_CHANNELS] = {0.0f, 0.0f};
const float kPeakDecay = 0.99985f;

// Effective parameter values (CV-driven, plus any panel offset already summed in hardware
// before the jack) — written by render(), read by meterGuiTask.
float gEffF0Center     = 55.0f;
float gEffDetuneRaw    = 0.0f;
float gEffNodeCouplingA = 0.1f;
float gEffNodeCouplingB = 0.1f;
float gEffXCoupleAmount   = 0.0f;
float gEffXCoupleSymmetry = 0.5f;
float gEffFreqLockDepth = 0.0f;
float gEffOutputScanA  = 0.5f;
float gEffOutputScanB  = 0.5f;

float amplitude = 4.0f;

float omega[NUM_CHANNELS][NUM_OSCS];
float theta[NUM_CHANNELS][NUM_OSCS];
float nodeExcitationGain[NUM_OSCS];
// Ratio-only excitation gain (mult[i]-derived) before absolute-pitch scaling is applied —
// see applyPitchEnergyScale().
float nodeExcitationGainBase[NUM_OSCS];
float loudnessWeight[NUM_OSCS];

float gPrevF0Center = 55.0f;
float gPrevDetune = 0.0f;
int gSyncPrev = 0;

// One-pole smoothing for the block-rate CV reads (F0/detune/node coupling/x-couple) —
// kills ADC/analog noise without adding per-sample cost. gCvSmoothCoeff is derived from
// a fixed time constant and the actual block period in setup(), so it stays correct
// across sample-rate/block-size changes.
float gSmCvF0 = 0.0f;
float gSmCvDetune = 0.0f;
float gSmCvNodeCouplingA = 0.0f;
float gSmCvNodeCouplingB = 0.0f;
float gSmCvXCoupleAmount = 0.0f;
float gSmCvXCoupleSymmetry = 0.0f;
float gCvSmoothCoeff = 1.0f;
bool gCvSmoothInit = false;
const float kCvSmoothTimeConstantS = 0.01f;

// Tune CV also gets a frame-rate (audio-rate) smoothed copy, fed straight into omega —
// the oscillator phase increment — so pitch can be modulated faster than the block rate.
// Bandpass filter coefficients (Q/center-freq, expensive: trig per node) stay on the slow
// gSmCvF0 path above via updateFrequencies()'s existing change-threshold gate; only the
// phase increment itself decouples to audio rate. gCvSmoothCoeffFast uses the same time
// constant as gCvSmoothCoeff, just expressed per-sample instead of per-block.
float gSmCvF0Fast = 0.0f;
float gCvSmoothCoeffFast = 1.0f;

struct Biquad {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    Biquad() : b0(0),b1(0),b2(0),a1(0),a2(0),x1(0),x2(0),y1(0),y2(0) {}
};

Biquad bpFilters[NUM_CHANNELS][NUM_OSCS];
float nodeEnvelope[NUM_CHANNELS][NUM_OSCS];
const float nodeBandwidth = 0.5f;

float couplingWeights[NUM_OSCS][NUM_OSCS];
float nodeBrightness[NUM_OSCS];
float nodeEffectiveDecay[NUM_OSCS];
// Ratio-only decay (mult[i]/nodeBrightness[i]-derived, nodeDecay-slider-scaled) before
// absolute-pitch scaling is applied — see applyPitchEnergyScale().
float nodeEffectiveDecayBase[NUM_OSCS];
float gPrevNodeDecay = 0.9995f;
float gChannelEnergy[NUM_CHANNELS] = {0.0f, 0.0f};
const float gEnergyReserve = 0.45f;

// Slow correlated drift: 3 LFOs per spectrum (sub / fundamental / harmonic families)
float gDriftPhase[NUM_CHANNELS][3];
const float kDriftLfoHz[3] = {0.037f, 0.059f, 0.043f};
const float kDriftFamilyDepth = 0.0016f;
const float kDriftNodeDepth = 0.00035f;

// Intra-ladder: signal sum into bandpass (prev-sample oscOut, one tick delay)
float gPrevOscOut[NUM_CHANNELS][NUM_OSCS];
// Cross-ladder frequency pulling: prev-sample phase of the other ladder's corresponding
// node (one tick delay, same reasoning as gPrevOscOut — avoids an ordering dependency
// between the two channels within a sample).
float gPrevTheta[NUM_CHANNELS][NUM_OSCS];
// Cross-ladder: envelope coupling memory
float gLadderCouplingDrive[NUM_CHANNELS][NUM_OSCS];
const float kCouplingMemCoeff = 0.997f;

const int multNum[NUM_OSCS] = { 1, 1, 3, 2, 5, 3, 7 };
const int multDen[NUM_OSCS] = { 2, 1, 2, 1, 2, 1, 2 };
const float mult[NUM_OSCS] = {
    1.0f/2.0f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f,
};

static const float kInvNumOscs = 1.0f / (float)NUM_OSCS;
static const float kDetuneMax = 0.1f;
static const float kDetuneLogK = 4.0f;

inline float effectiveDetune(float detune) {
    return fmaxf(detune, FLT_MIN);
}

// Detune slider 0..1: most travel is subtle (log-shaped toward kDetuneMax)
inline float mapDetune(float detuneSlider0to1) {
    float t = fminf(fmaxf(detuneSlider0to1, 0.0f), 1.0f);
    float shaped = (expf(kDetuneLogK * t) - 1.0f) / (expf(kDetuneLogK) - 1.0f);
    return effectiveDetune(kDetuneMax * shaped);
}

static const float kF0Max = 1024.0f;

inline float clampScan01(float x) {
    return fminf(fmaxf(x, 0.0f), 1.0f);
}

inline float clampF0Hz(float hz) {
    return fminf(fmaxf(hz, 0.0f), kF0Max);
}

// Absolute-pitch energy reference, independent of each node's harmonic ratio: at/above
// kPitchEnergyRefHz a node gets no low-pitch penalty; at/below kPitchEnergyFloorHz it gets
// the full penalty. Octave-scaled (log2) so the transition feels even across the range,
// matching how pitch is perceived.
const float kPitchEnergyRefHz = 220.0f;
const float kPitchEnergyFloorHz = 24.0f;
const float kPitchEnergyRangeOctaves = log2f(kPitchEnergyRefHz / kPitchEnergyFloorHz);
const float kPitchDecayCeil = 0.9999f; // matches maxDecayCeil in buildNodeEffectiveDecay

inline float pitchLowness(float absFreqHz) {
    float f = fmaxf(absFreqHz, 1.0f);
    float octavesBelowRef = log2f(kPitchEnergyRefHz / f);
    return fminf(fmaxf(octavesBelowRef / kPitchEnergyRangeOctaves, 0.0f), 1.0f);
}

inline int analogFrameForAudio(BelaContext *context, int audioFrame) {
    if(context->analogFrames == 0) return 0;
    return (audioFrame * (int)context->analogFrames) / (int)context->audioFrames;
}

float getFrequency(int i, float f0) {
    return f0 * mult[i];
}

int igcd(int a, int b) {
    while(b) { int t = b; b = a % b; a = t; }
    return a;
}

void buildNodeEffectiveDecay(float nodeDecay) {
    const float maxDecayCeil = 0.9999f;
    const float minDecayFloor = 0.97f;
    const float lowBias = 0.25f;
    const float highDampBias = 0.35f;
    for(int i = 0; i < NUM_OSCS; i++) {
        float b = nodeBrightness[i];
        float slowerDecay = nodeDecay + (maxDecayCeil - nodeDecay) * lowBias * (1.0f - b);
        nodeEffectiveDecayBase[i] = slowerDecay
            - (slowerDecay - minDecayFloor) * highDampBias * b;
    }
}

void buildCouplingWeights() {
    for(int i = 0; i < NUM_OSCS; i++) {
        for(int j = 0; j < NUM_OSCS; j++) {
            if(i == j) { couplingWeights[i][j] = 0.0f; continue; }

            int pj = multNum[j];
            int qj = multDen[j];
            int pi = multNum[i];
            int qi = multDen[i];

            int num = pj * qi;
            int den = qj * pi;
            int g   = igcd(num, den);
            int num_r = num / g;
            int den_r = den / g;

#ifdef EMPATH_LEGACY_COUPLING
            float w = 1.0f / (float)num_r;
#else
            float complexity = (float)num_r + 0.35f * (float)(den_r - 1);
            float base = 1.0f / complexity;
            float asymmetry = (mult[j] < mult[i]) ? 1.15f : 0.8f;
            float w = fminf(base * asymmetry, 0.72f);
#endif
            couplingWeights[i][j] = (w > 0.035f) ? w : 0.0f;
        }
    }
}

void buildExcitationScale() {
    float weightSum = 0.0f;
    for(int i = 0; i < NUM_OSCS; i++) {
        nodeExcitationGainBase[i] = fminf(mult[i], 1.0f) * sqrtf(fmaxf(1.0f / mult[i], 1.0f));
        nodeBrightness[i] = fminf(fmaxf(log2f(mult[i] * 8.0f) / 6.0f, 0.0f), 1.0f);

        float fNorm = fmaxf(mult[i], 1.0f / 8.0f);
        loudnessWeight[i] = powf(fNorm, -0.22f);
        weightSum += loudnessWeight[i];
    }
    if(weightSum > 1e-9f)
        for(int i = 0; i < NUM_OSCS; i++)
            loudnessWeight[i] /= weightSum;
}

// Absolute-pitch energy/decay shaping, independent of buildExcitationScale()'s/
// buildNodeEffectiveDecay()'s ratio-only (mult[i]/nodeBrightness[i]) shaping above. A lower
// absolute F0 means physically larger resonators (tines, bars, strings) — harder to excite
// and slower to die out, like a piano's bass strings vs. its treble. exciteDepth and
// decayDepth are independent GUI sliders (0 = no effect, matches old behavior) so each can
// be dialed in by ear separately.
void applyPitchEnergyScale(float f0, float exciteDepth, float decayDepth) {
    for(int i = 0; i < NUM_OSCS; i++) {
        float lowness = pitchLowness(getFrequency(i, f0));
        nodeExcitationGain[i] = nodeExcitationGainBase[i] * (1.0f - exciteDepth * lowness);
        nodeEffectiveDecay[i] = nodeEffectiveDecayBase[i]
            + (kPitchDecayCeil - nodeEffectiveDecayBase[i]) * decayDepth * lowness;
    }
}

void updateBiquadBP(Biquad &f, float freq, float Q, float sampleRate) {
    float w0    = 2.0f * M_PI * freq / sampleRate;
    float alpha = sinf_neon(w0) / (2.0f * Q);
    float cosw0 = cosf_neon(w0);
    float a0    = 1.0f + alpha;
    f.b0 =  alpha / a0;
    f.b1 =  0.0f;
    f.b2 = -alpha / a0;
    f.a1 = -2.0f * cosw0 / a0;
    f.a2 = (1.0f - alpha) / a0;
}

inline float processBiquad(Biquad &f, float in) {
    float out = f.b0*in + f.b1*f.x1 + f.b2*f.x2
              - f.a1*f.y1 - f.a2*f.y2;
    f.x2 = f.x1; f.x1 = in;
    f.y2 = f.y1; f.y1 = out;
    return out;
}

// Bandpass filter coefficients only — expensive (trig per node), so left on the slow,
// change-threshold-gated path. Oscillator phase rate (omega) is handled separately, every
// frame, by updateOmegaFast() below, so pitch can move faster than the filter can follow.
void updateFrequencies(BelaContext *context, int ch, float f0Spectrum) {
    for(int i = 0; i < NUM_OSCS; i++) {
        float baseFreq     = getFrequency(i, f0Spectrum);
        float neighborFreq = (i < NUM_OSCS - 1) ?
            getFrequency(i + 1, f0Spectrum) : getFrequency(i - 1, f0Spectrum);
        float spacing = fabsf(neighborFreq - baseFreq);
        float freq    = (baseFreq == 0.0f) ? 0.001f : baseFreq;
        float absFreq = fabsf(freq);
        float Q = absFreq / fmaxf(spacing, 1.0f) * nodeBandwidth;
        Q = fmaxf(0.5f, fminf(Q, 50.0f));
        updateBiquadBP(bpFilters[ch][i], absFreq, Q, context->audioSampleRate);
    }
}

// Cheap (no trig) per-frame omega update — lets oscillator pitch follow Tune CV at audio
// rate independent of the filter coefficient recompute above.
inline void updateOmegaFast(BelaContext *context, int ch, float f0Spectrum) {
    for(int i = 0; i < NUM_OSCS; i++) {
        float baseFreq = getFrequency(i, f0Spectrum);
        float freq = (baseFreq == 0.0f) ? 0.001f : baseFreq;
        omega[ch][i] = 2.0f * M_PI * freq / context->audioSampleRate;
    }
}

inline int driftFamilyForNode(int i) {
    if(i < 5) return 0;
    if(i == 5) return 1;
    return 2;
}

float nodeDriftAmount(int ch, int i) {
    int fam = driftFamilyForNode(i);
    float family = sinf_neon(gDriftPhase[ch][fam]) * kDriftFamilyDepth;
    float node = sinf_neon(gDriftPhase[ch][fam] * 1.31f + (float)i * 0.89f) * kDriftNodeDepth;
    return family + node;
}

void advanceDriftLfo(BelaContext *context, int ch) {
    float inc = 2.0f * M_PI / (float)context->audioSampleRate;
    for(int f = 0; f < 3; f++)
        gDriftPhase[ch][f] += inc * kDriftLfoHz[f];
}

void resetSpectrumPhase(int ch) {
    for(int i = 0; i < NUM_OSCS; i++) {
        theta[ch][i] = 0.0f;
        gLadderCouplingDrive[ch][i] = 0.0f;
        gPrevOscOut[ch][i] = 0.0f;
        gPrevTheta[ch][i] = 0.0f;
    }
}

void resetAllSpectra() {
    resetSpectrumPhase(SPEC_A);
    resetSpectrumPhase(SPEC_B);
}

void initOscillators(BelaContext *context, float f0Center, float detuneSlider0to1) {
    float d = mapDetune(detuneSlider0to1);
    float f0A = f0Center * (1.0f - d);
    float f0B = f0Center * (1.0f + d);
    updateFrequencies(context, SPEC_A, f0A);
    updateFrequencies(context, SPEC_B, f0B);
    updateOmegaFast(context, SPEC_A, f0A);
    updateOmegaFast(context, SPEC_B, f0B);
    for(int ch = 0; ch < NUM_CHANNELS; ch++) {
        for(int f = 0; f < 3; f++)
            gDriftPhase[ch][f] = 0.0f;
        for(int i = 0; i < NUM_OSCS; i++) {
            theta[ch][i] = 0.0f;
            nodeEnvelope[ch][i] = 0.0f;
            gLadderCouplingDrive[ch][i] = 0.0f;
            gPrevOscOut[ch][i] = 0.0f;
            gPrevTheta[ch][i] = 0.0f;
        }
    }
}

/*
void computeInputScan(float scanPos, int &node, int &nodeNext, float &gain, float &gainNext) {
    float pos       = scanPos * NUM_OSCS;
    int   n         = (int)pos % NUM_OSCS;
    float frac      = pos - (int)pos;
    node            = n;
    nodeNext        = (n + 1) % NUM_OSCS;
    gain            = cosf_neon(frac * M_PI / 2.0f);
    gainNext        = sinf_neon(frac * M_PI / 2.0f);
}
*/

void computeOutputScan(float scanPos, int &outNode, int &outNodeNext,
                       float &gain, float &gainNext) {
    float pos       = scanPos * (float)NUM_OSCS;
    int   n         = (int)pos % NUM_OSCS;
    float frac      = pos - (int)pos;
    outNode         = n;
    outNodeNext     = (n + 1) % NUM_OSCS;
    gain            = cosf_neon(frac * M_PI / 2.0f);
    gainNext        = sinf_neon(frac * M_PI / 2.0f);
}

// Lower-priority task: GUI setSliderValue is not RT-safe — never call from render().
void meterGuiTask(void *) {
    static const unsigned int kNumMeters = 4;
    float sent[kNumMeters] = {0.0f, 0.0f, 0.0f, 0.0f};
    const unsigned int meterIdx[kNumMeters] = {
        gInPeakASliderIdx, gInPeakBSliderIdx,
        gOutPeakASliderIdx, gOutPeakBSliderIdx
    };
    const float kSendThreshold = 0.05f;

    while(!Bela_stopRequested()) {
        const float *levels[kNumMeters] = {
            &gInPeakHold[0], &gInPeakHold[1],
            &gOutPeakHold[0], &gOutPeakHold[1]
        };
        for(unsigned int m = 0; m < kNumMeters; m++) {
            float level = *levels[m];
            if(fabsf(level - sent[m]) < kSendThreshold && level < 0.95f)
                continue;
            controller.setSliderValue(meterIdx[m], level);
            sent[m] = level;
        }

        // CV parameter readouts — separate display-only sliders, never read back as
        // control input (would otherwise feed back into render()'s parameter computation
        // and accumulate every tick).
        controller.setSliderValue(gF0DisplaySliderIdx,            gEffF0Center);
        controller.setSliderValue(gDetuneDisplaySliderIdx,        gEffDetuneRaw);
        controller.setSliderValue(gNodeCouplingADisplaySliderIdx, gEffNodeCouplingA);
        controller.setSliderValue(gNodeCouplingBDisplaySliderIdx, gEffNodeCouplingB);
        controller.setSliderValue(gXCoupleAmountDisplaySliderIdx,   gEffXCoupleAmount);
        controller.setSliderValue(gXCoupleSymmetryDisplaySliderIdx, gEffXCoupleSymmetry);
        controller.setSliderValue(gFreqLockDepthDisplaySliderIdx,   gEffFreqLockDepth);
        controller.setSliderValue(gOutputScanADisplaySliderIdx,   gEffOutputScanA);
        controller.setSliderValue(gOutputScanBDisplaySliderIdx,   gEffOutputScanB);

#ifdef __INTELLISENSE__
        for(volatile int d = 0; d < 80000; ++d) {}
#else
        usleep(80000);
#endif
    }
}

bool setup(BelaContext *context, void *userData) {
    buildCouplingWeights();
    buildExcitationScale();
    buildNodeEffectiveDecay(0.9995f);
    applyPitchEnergyScale(55.0f, 0.0f, 0.0f);

    gui.setup(context->projectName);
    controller.setup(&gui, "Harmonic Resonator");

    gGlobalPhaseSliderIdx   = controller.addSlider("Osc Phase",           0.0,    0.0,  6.2832, 0.001);
    gNodeAttackSliderIdx    = controller.addSlider("Node Env Attack",  0.9041,    0.9, 0.9999, 0.0001);
    gNodeDecaySliderIdx     = controller.addSlider("Node Env Decay",     0.91,    0.9, 0.9999, 0.0001);
    gFreqLockDepthSliderIdx = controller.addSlider("Freq Lock Depth",    0.0,     0.0, 0.5,    0.001);
    gPitchExciteDepthSliderIdx = controller.addSlider("Pitch Excite Depth", 0.193, 0.0, 1.0, 0.001);
    gPitchDecayDepthSliderIdx  = controller.addSlider("Pitch Decay Depth",  0.239, 0.0, 1.0, 0.001);
    gInPeakASliderIdx       = controller.addSlider("In A Peak (1=clip)",  0.0,    0.0, 1.2, 0.001);
    gInPeakBSliderIdx       = controller.addSlider("In B Peak (1=clip)",  0.0,    0.0, 1.2, 0.001);
    gOutPeakASliderIdx      = controller.addSlider("Out A Peak (1=clip)", 0.0,    0.0, 1.2, 0.001);
    gOutPeakBSliderIdx      = controller.addSlider("Out B Peak (1=clip)", 0.0,    0.0, 1.2, 0.001);

    gF0DisplaySliderIdx           = controller.addSlider("F0 (eff)",            55.0,  0.0, 1024.0, 0.1);
    gDetuneDisplaySliderIdx       = controller.addSlider("Detune (eff)",         0.18, 0.0,    kDetuneMax, 0.000001);
    gNodeCouplingADisplaySliderIdx = controller.addSlider("Node Coupling A (eff)", 0.1, 0.0,    2.0, 0.001);
    gNodeCouplingBDisplaySliderIdx = controller.addSlider("Node Coupling B (eff)", 0.1, 0.0,    2.0, 0.001);
    gXCoupleAmountDisplaySliderIdx   = controller.addSlider("XCouple Amount (eff)",   0.0, 0.0, 1.0, 0.001);
    gXCoupleSymmetryDisplaySliderIdx = controller.addSlider("XCouple Symmetry (eff)", 0.5, 0.0, 1.0, 0.001);
    gFreqLockDepthDisplaySliderIdx   = controller.addSlider("Freq Lock Depth (eff)",  0.0, 0.0, 1.0, 0.001);
    gOutputScanADisplaySliderIdx   = controller.addSlider("Output A Scan (eff)",   0.5, 0.0,    1.0, 0.001);
    gOutputScanBDisplaySliderIdx   = controller.addSlider("Output B Scan (eff)",   0.5, 0.0,    1.0, 0.001);

    // Digital I/O setup
    pinMode(context, 0, SYNC_DIGITAL, INPUT);

    AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
    Bela_scheduleAuxiliaryTask(meterTask);

    initOscillators(context, 55.0f, 0.0f);
    gPrevF0Center = 55.0f;
    gPrevDetune = mapDetune(0.0f);

    float blockPeriodS = (float)context->audioFrames / (float)context->audioSampleRate;
    gCvSmoothCoeff = blockPeriodS / (kCvSmoothTimeConstantS + blockPeriodS);

    float samplePeriodS = 1.0f / (float)context->audioSampleRate;
    gCvSmoothCoeffFast = samplePeriodS / (kCvSmoothTimeConstantS + samplePeriodS);

    return true;
}

void render(BelaContext *context, void *userData) {
    // All control CVs read at block rate — parameter values are constants
    // throughout the frame loop; filter/coupling coefficient updates are expensive.
    float cvF0           = 0.0f;
    float cvDetune       = 0.0f;
    float cvNodeCouplingA = 0.0f;
    float cvNodeCouplingB = 0.0f;
    float cvXCoupleAmount   = 0.0f;
    float cvXCoupleSymmetry = 0.0f;
    if(context->analogFrames > 0) {
        cvF0            = analogRead(context, 0, ANALOG_TUNE_CV);
        cvDetune        = analogRead(context, 0, ANALOG_DETUNE_CV);
        cvNodeCouplingA = analogRead(context, 0, ANALOG_NODE_COUPLING_A_CV);
        cvNodeCouplingB = analogRead(context, 0, ANALOG_NODE_COUPLING_B_CV);
        cvXCoupleAmount   = analogRead(context, 0, ANALOG_XCOUPLE_AMOUNT_CV);
        cvXCoupleSymmetry = analogRead(context, 0, ANALOG_XCOUPLE_SYMMETRY_CV);
    }
    if(!gCvSmoothInit) {
        gSmCvF0 = cvF0; gSmCvF0Fast = cvF0; gSmCvDetune = cvDetune;
        gSmCvNodeCouplingA = cvNodeCouplingA; gSmCvNodeCouplingB = cvNodeCouplingB;
        gSmCvXCoupleAmount = cvXCoupleAmount; gSmCvXCoupleSymmetry = cvXCoupleSymmetry;
        gCvSmoothInit = true;
    } else {
        gSmCvF0              += gCvSmoothCoeff * (cvF0              - gSmCvF0);
        gSmCvDetune          += gCvSmoothCoeff * (cvDetune          - gSmCvDetune);
        gSmCvNodeCouplingA   += gCvSmoothCoeff * (cvNodeCouplingA   - gSmCvNodeCouplingA);
        gSmCvNodeCouplingB   += gCvSmoothCoeff * (cvNodeCouplingB   - gSmCvNodeCouplingB);
        gSmCvXCoupleAmount   += gCvSmoothCoeff * (cvXCoupleAmount   - gSmCvXCoupleAmount);
        gSmCvXCoupleSymmetry += gCvSmoothCoeff * (cvXCoupleSymmetry - gSmCvXCoupleSymmetry);
    }
    float f0Center   = clampF0Hz(gSmCvF0 * kF0Max);
    float detuneRaw  = clampScan01(gSmCvDetune);
    float detune     = mapDetune(detuneRaw);
    float globalPhase    = controller.getSliderValue(gGlobalPhaseSliderIdx);
    float nodeCouplingA  = fminf(fmaxf(gSmCvNodeCouplingA * 2.0f, 0.0f), 2.0f);
    float nodeCouplingB  = fminf(fmaxf(gSmCvNodeCouplingB * 2.0f, 0.0f), 2.0f);
    float xCoupleAmount   = clampScan01(gSmCvXCoupleAmount);
    float xCoupleSymmetry = clampScan01(gSmCvXCoupleSymmetry);
    // Symmetry sweeps the Amount budget between directions (0 = all A->B, 1 = all B->A,
    // 0.5 = split evenly) — constant-sum so total injected cross-energy never exceeds
    // Amount regardless of where Symmetry sits, given how feedback-sensitive xCoupling is.
    float xCouplingAB = xCoupleAmount * (1.0f - xCoupleSymmetry);
    float xCouplingBA = xCoupleAmount * xCoupleSymmetry;
    float nodeAttack    = controller.getSliderValue(gNodeAttackSliderIdx);
    float nodeDecay     = controller.getSliderValue(gNodeDecaySliderIdx);
    float pitchExciteDepth = controller.getSliderValue(gPitchExciteDepthSliderIdx);
    float pitchDecayDepth  = controller.getSliderValue(gPitchDecayDepthSliderIdx);
    // Detune adds its own contribution to lock depth on top of the GUI slider — as the
    // ladders are pushed further apart, the corresponding-node pull is dialed up too, so
    // the system rides nearer the edge between sync and drift instead of just drifting
    // apart unopposed. kDetuneLockDepthScale maps detune's full range (0..kDetuneMax) onto
    // the same 0..0.5 span as the slider itself; flip the sign here to make it work the
    // other way (more detune = less lock) if that reads better by ear.
    const float kDetuneLockDepthScale = 5.0f;
    float freqLockDepth = controller.getSliderValue(gFreqLockDepthSliderIdx)
                         + detune * kDetuneLockDepthScale;
    const float couplingSpendA = nodeCouplingA + 0.5f * xCouplingBA;
    const float couplingSpendB = nodeCouplingB + 0.5f * xCouplingAB;

    gEffF0Center      = f0Center;
    gEffDetuneRaw     = detune;
    gEffNodeCouplingA = nodeCouplingA;
    gEffNodeCouplingB = nodeCouplingB;
    gEffXCoupleAmount   = xCoupleAmount;
    gEffXCoupleSymmetry = xCoupleSymmetry;
    gEffFreqLockDepth   = freqLockDepth;
    const float energyReserve = gEnergyReserve;
    const float reserveCurve = energyReserve * energyReserve;
    const float activeBudgetScale = 0.2f + 1.8f * reserveCurve;
    const float replenishScale = 0.0003f + 0.02f * reserveCurve;
    const float leakScale = 0.004f * (1.0f - reserveCurve) + 0.00008f;
    const float budgetBase = 0.002f + 0.01f * reserveCurve;

    float f0A = f0Center * (1.0f - detune);
    float f0B = f0Center * (1.0f + detune);

    if(fabsf(f0Center - gPrevF0Center) > 1e-6f || fabsf(detune - gPrevDetune) > 1e-6f) {
        updateFrequencies(context, SPEC_A, f0A);
        updateFrequencies(context, SPEC_B, f0B);
        gPrevF0Center = f0Center;
        gPrevDetune = detune;
    }
    if(fabsf(nodeDecay - gPrevNodeDecay) > 1e-6f) {
        buildNodeEffectiveDecay(nodeDecay);
        gPrevNodeDecay = nodeDecay;
    }
    // Cheap (no trig, NUM_OSCS iterations) — recomputed every block rather than
    // change-gated like the trig-heavy paths above.
    applyPitchEnergyScale(f0Center, pitchExciteDepth, pitchDecayDepth);

    for(unsigned int n = 0; n < context->digitalFrames; n++) {
        int sync = digitalRead(context, n, SYNC_DIGITAL);
        if(sync && !gSyncPrev)
            resetAllSpectra();
        gSyncPrev = sync ? 1 : 0;
    }

    for(int frame = 0; frame < context->audioFrames; frame++) {

        float preEnv[NUM_CHANNELS][NUM_OSCS];
        float nodeSin[NUM_CHANNELS][NUM_OSCS];
        float oscOut[NUM_CHANNELS][NUM_OSCS];
        float inputAbs[NUM_CHANNELS];

        // Tune CV read and smoothed at audio rate, feeding omega directly — pitch can
        // track modulation faster than the block-rate gSmCvF0/updateFrequencies() path
        // above, which still governs the (expensive) bandpass filter coefficients.
        if(context->analogFrames > 0) {
            int af = analogFrameForAudio(context, frame);
            float cvF0Frame = analogRead(context, af, ANALOG_TUNE_CV);
            gSmCvF0Fast += gCvSmoothCoeffFast * (cvF0Frame - gSmCvF0Fast);
        }
        float f0CenterFast = clampF0Hz(gSmCvF0Fast * kF0Max);
        updateOmegaFast(context, SPEC_A, f0CenterFast * (1.0f - detune));
        updateOmegaFast(context, SPEC_B, f0CenterFast * (1.0f + detune));

        for(int ch = 0; ch < NUM_CHANNELS; ch++) {
            // ch indexes both SPEC_A/SPEC_B and AUDIO_IN_A/AUDIO_IN_B identically — see pins.h
            float allIn = audioRead(context, frame, ch);
            inputAbs[ch] = fabsf(allIn);
            gInPeakHold[ch] = fmaxf(inputAbs[ch], gInPeakHold[ch] * kPeakDecay);

#if EMPATH_DRIFT_ENABLED
            advanceDriftLfo(context, ch);
#endif

            float nodeCouplingCh = (ch == SPEC_A) ? nodeCouplingA : nodeCouplingB;
            for(int i = 0; i < NUM_OSCS; i++) {
                float intraSum = 0.0f;
                for(int j = 0; j < NUM_OSCS; j++) {
                    float w = couplingWeights[i][j];
                    if(w == 0.0f) continue;
                    intraSum += w * gPrevOscOut[ch][j];
                }
                float nodeIn = allIn + nodeCouplingCh * intraSum;
                float bandSig = processBiquad(bpFilters[ch][i],
                    nodeIn * nodeExcitationGain[i]);
                float target  = fabsf(bandSig);

                if(target > nodeEnvelope[ch][i])
                    nodeEnvelope[ch][i] = nodeAttack * nodeEnvelope[ch][i]
                                        + (1.0f - nodeAttack) * target;
                else
                    nodeEnvelope[ch][i] = nodeEffectiveDecay[i] * nodeEnvelope[ch][i]
                                        + (1.0f - nodeEffectiveDecay[i]) * target;

#if EMPATH_DRIFT_ENABLED
                float drift = nodeDriftAmount(ch, i);
#else
                float drift = 0.0f;
#endif
                // Cross-ladder injection locking (Adler/Kuramoto): pulls this node's phase
                // toward the corresponding node in the other ladder, scaled by its own
                // omega so the pull is a bounded fraction of its own frequency rather than
                // an absolute Hz amount. Only valid between corresponding (same-index) node
                // pairs since they're nearly equal frequency by construction (differ only by
                // detune) — extending this to non-corresponding or intra-ladder pairs would
                // need a generalized harmonic locking term (sin(a*theta_j - b*theta_i)), not
                // implemented.
#if EMPATH_FREQ_LOCK_ENABLED
                int otherCh = 1 - ch;
                float phasePull = freqLockDepth * omega[ch][i]
                                 * sinf_neon(gPrevTheta[otherCh][i] - theta[ch][i]);
#else
                float phasePull = 0.0f;
#endif
                theta[ch][i] += omega[ch][i] * (1.0f + drift) + phasePull;
                if(theta[ch][i] >  M_PI) theta[ch][i] -= 2.0f * M_PI;
                if(theta[ch][i] < -M_PI) theta[ch][i] += 2.0f * M_PI;

                nodeSin[ch][i] = sinf_neon(theta[ch][i] + globalPhase);
                preEnv[ch][i] = nodeEnvelope[ch][i];
            }
        }

        for(int ch = 0; ch < NUM_CHANNELS; ch++) {
            const int other = 1 - ch;
            float xCoupling = (ch == SPEC_A) ? xCouplingBA : xCouplingAB;
            for(int i = 0; i < NUM_OSCS; i++) {
                float ladderSum = preEnv[other][i];
                for(int j = 0; j < NUM_OSCS; j++) {
                    float w = couplingWeights[i][j];
                    if(w == 0.0f) continue;
                    ladderSum += preEnv[other][j] * w;
                }
                gLadderCouplingDrive[ch][i] = kCouplingMemCoeff * gLadderCouplingDrive[ch][i]
                                            + (1.0f - kCouplingMemCoeff) * ladderSum;
                // Attack-only pull toward the cross-ladder target, not an unconditional add:
                // an unconditional add ratchets every sample with nothing but the hard clamp
                // to stop it, so at slow nodeDecay even tiny xCoupling rails to ceiling almost
                // instantly. Bounding it to the same attack dynamics as local excitation lets
                // it decay back down on the next sample's local-target pass once the source
                // ladder's energy fades, instead of being a permanent floor.
                float crossTarget = fminf(gLadderCouplingDrive[ch][i] * xCoupling, 1.05f);
                if(crossTarget > nodeEnvelope[ch][i])
                    nodeEnvelope[ch][i] = nodeAttack * nodeEnvelope[ch][i]
                                        + (1.0f - nodeAttack) * crossTarget;
            }

            gChannelEnergy[ch] = fminf(gChannelEnergy[ch] + replenishScale * inputAbs[ch], 2.2f);

            float sumSq = 0.0f;
            for(int i = 0; i < NUM_OSCS; i++)
                sumSq += nodeEnvelope[ch][i] * nodeEnvelope[ch][i];
            float meanSq = sumSq * kInvNumOscs;

            float budgetMeanSq = budgetBase + activeBudgetScale * 0.22f * gChannelEnergy[ch];
            if(meanSq > budgetMeanSq && meanSq > 1e-9f) {
                float scale = sqrtf(budgetMeanSq / meanSq);
                for(int i = 0; i < NUM_OSCS; i++)
                    nodeEnvelope[ch][i] *= scale;
            }

            float couplingSpend = (ch == SPEC_A) ? couplingSpendA : couplingSpendB;
            float spend = meanSq * (0.0012f + 0.0055f * couplingSpend);
            gChannelEnergy[ch] = fmaxf(gChannelEnergy[ch] - spend - leakScale, 0.0f);

            for(int i = 0; i < NUM_OSCS; i++)
                oscOut[ch][i] = nodeSin[ch][i] * nodeEnvelope[ch][i];
        }

        for(int ch = 0; ch < NUM_CHANNELS; ch++)
            for(int i = 0; i < NUM_OSCS; i++) {
                gPrevOscOut[ch][i] = oscOut[ch][i];
                gPrevTheta[ch][i] = theta[ch][i];
            }

        float cvScanA = 0.0f;
        float cvScanB = 0.0f;
        if(context->analogFrames > 0) {
            int af = analogFrameForAudio(context, frame);
            cvScanA = analogRead(context, af, ANALOG_SCAN_A_CV);
            cvScanB = analogRead(context, af, ANALOG_SCAN_B_CV);
        }
        float outputScanA = clampScan01(cvScanA);
        float outputScanB = clampScan01(cvScanB);
        gEffOutputScanA = outputScanA;
        gEffOutputScanB = outputScanB;

        int outNodeA, outNodeA_next, outNodeB, outNodeB_next;
        float gainNodeA, gainNextA, gainNodeB, gainNextB;
        computeOutputScan(outputScanA, outNodeA, outNodeA_next, gainNodeA, gainNextA);
        computeOutputScan(outputScanB, outNodeB, outNodeB_next, gainNodeB, gainNextB);

        float scanOutA = oscOut[SPEC_A][outNodeA] * gainNodeA
                       + oscOut[SPEC_A][outNodeA_next] * gainNextA;
        float scanOutB = oscOut[SPEC_B][outNodeB] * gainNodeB
                       + oscOut[SPEC_B][outNodeB_next] * gainNextB;

        float outA = scanOutA * amplitude;
        float outB = scanOutB * amplitude;
        gOutPeakHold[0] = fmaxf(fabsf(outA), gOutPeakHold[0] * kPeakDecay);
        gOutPeakHold[1] = fmaxf(fabsf(outB), gOutPeakHold[1] * kPeakDecay);

        audioWrite(context, frame, AUDIO_OUT_A, outA);
        audioWrite(context, frame, AUDIO_OUT_B, outB);

    }
}

void cleanup(BelaContext *context, void *userData) {}
