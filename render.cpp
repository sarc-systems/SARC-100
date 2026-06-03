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

#define NUM_OSCS 11
#define NUM_CHANNELS 2
#define SPEC_A 0
#define SPEC_B 1

// Digital SYNC (gate/trigger jacks)
#define SYNC_DIGITAL_A 0
#define SYNC_DIGITAL_B 1

// Main audio I/O (ch 0/1): all-in / all-out for now.
// Dedicated scan jacks + analog CV outs (ch 2+) when scan path is wired up.

Gui gui;
GuiController controller;
unsigned int gF0SliderIdx;
unsigned int gDetuneSliderIdx;
unsigned int gGlobalPhaseSliderIdx;
unsigned int gKSpreadSliderIdx;
unsigned int gInputScanASliderIdx;
unsigned int gInputScanBSliderIdx;
unsigned int gOutputScanASliderIdx;
unsigned int gOutputScanBSliderIdx;
unsigned int gSpecAToBSliderIdx;
unsigned int gSpecBToASliderIdx;
unsigned int gNodeAttackSliderIdx;
unsigned int gNodeDecaySliderIdx;
unsigned int gInPeakASliderIdx;
unsigned int gInPeakBSliderIdx;
unsigned int gOutPeakASliderIdx;
unsigned int gOutPeakBSliderIdx;

float gInPeakHold[NUM_CHANNELS] = {0.0f, 0.0f};
float gOutPeakHold[NUM_CHANNELS] = {0.0f, 0.0f};
const float kPeakDecay = 0.99985f;

float amplitude = 4.0f;

float omega[NUM_CHANNELS][NUM_OSCS];
float theta[NUM_CHANNELS][NUM_OSCS];
float excitationScale[NUM_OSCS];
float injectionBoost[NUM_OSCS];
float loudnessWeight[NUM_OSCS];

float gPrevF0Center = 55.0f;
float gPrevDetune = 0.0f;
float gFeedbackBus[NUM_CHANNELS] = {0.0f, 0.0f};
float gFeedbackActivity[NUM_CHANNELS] = {0.0f, 0.0f};
int gSyncPrev[NUM_CHANNELS] = {0, 0};

// Cross-spectrum feedback: envelope bus + soft gate (single multiply, not gate²)
const float kFeedbackBusGain = 3.25f;
const float kFeedbackRawBleed = 0.38f;
const float kFeedbackMinOpen = 0.42f;
const float kFeedbackAttack = 0.9f;
const float kFeedbackRelease = 0.996f;
const float kFeedbackActAttack = 0.78f;
const float kFeedbackActRelease = 0.997f;

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
float gPrevNodeDecay = 0.9995f;
float gChannelEnergy[NUM_CHANNELS] = {0.0f, 0.0f};
const float gEnergyReserve = 0.45f;

// Slow correlated drift: 3 LFOs per spectrum (sub / fundamental / harmonic families)
float gDriftPhase[NUM_CHANNELS][3];
const float kDriftLfoHz[3] = {0.037f, 0.059f, 0.043f};
const float kDriftFamilyDepth = 0.0016f;
const float kDriftNodeDepth = 0.00035f;

// Coupling memory: smoothed cross-node drive (one-pole per node)
float gCouplingDrive[NUM_CHANNELS][NUM_OSCS];
const float kCouplingMemCoeff = 0.997f;

// 1/8 … 1, 3/2 … 7/2 (just intonation–friendly set)
const float mult[NUM_OSCS] = {
    1.0f/8.0f, 1.0f/6.0f, 1.0f/4.0f, 1.0f/3.0f, 1.0f/2.0f, 1.0f,
    1.5f, 2.0f, 2.5f, 3.0f, 3.5f,
};
const int multNum[NUM_OSCS] = { 1, 1, 1, 1, 1, 1, 3, 2, 5, 3, 7 };
const int multDen[NUM_OSCS] = { 8, 6, 4, 3, 2, 1, 2, 1, 2, 1, 2 };

static const float kInvNumOscs = 1.0f / (float)NUM_OSCS;

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
        nodeEffectiveDecay[i] = slowerDecay
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

#ifdef KOSC_LEGACY_COUPLING
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
        excitationScale[i] = fminf(mult[i], 1.0f);
        float boost = sqrtf(fmaxf(1.0f / mult[i], 1.0f));
        injectionBoost[i] = boost;
        nodeBrightness[i] = fminf(fmaxf(log2f(mult[i] * 8.0f) / 6.0f, 0.0f), 1.0f);

        float fNorm = fmaxf(mult[i], 1.0f / 8.0f);
        loudnessWeight[i] = powf(fNorm, -0.22f);
        weightSum += loudnessWeight[i];
    }
    if(weightSum > 1e-9f)
        for(int i = 0; i < NUM_OSCS; i++)
            loudnessWeight[i] /= weightSum;
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

void updateFrequencies(BelaContext *context, int ch, float f0Spectrum) {
    for(int i = 0; i < NUM_OSCS; i++) {
        float baseFreq     = getFrequency(i, f0Spectrum);
        float neighborFreq = (i < NUM_OSCS - 1) ?
            getFrequency(i + 1, f0Spectrum) : getFrequency(i - 1, f0Spectrum);
        float spacing = fabsf(neighborFreq - baseFreq);
        float freq    = (baseFreq == 0.0f) ? 0.001f : baseFreq;
        float absFreq = fabsf(freq);
        omega[ch][i] = 2.0f * M_PI * freq / context->audioSampleRate;
        float Q = absFreq / fmaxf(spacing, 1.0f) * nodeBandwidth;
        Q = fmaxf(0.5f, fminf(Q, 50.0f));
        updateBiquadBP(bpFilters[ch][i], absFreq, Q, context->audioSampleRate);
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
        gCouplingDrive[ch][i] = 0.0f;
    }
    gFeedbackBus[ch] = 0.0f;
    gFeedbackActivity[ch] = 0.0f;
}

void initOscillators(BelaContext *context, float f0Center, float detune) {
    float f0A = f0Center * (1.0f - detune);
    float f0B = f0Center * (1.0f + detune);
    updateFrequencies(context, SPEC_A, f0A);
    updateFrequencies(context, SPEC_B, f0B);
    for(int ch = 0; ch < NUM_CHANNELS; ch++) {
        for(int f = 0; f < 3; f++)
            gDriftPhase[ch][f] = 0.0f;
        for(int i = 0; i < NUM_OSCS; i++) {
            theta[ch][i] = 0.0f;
            nodeEnvelope[ch][i] = 0.0f;
            gCouplingDrive[ch][i] = 0.0f;
        }
    }
}

void computeInputScan(float scanPos, int &node, int &nodeNext, float &gain, float &gainNext) {
    float pos       = scanPos * NUM_OSCS;
    int   n         = (int)pos % NUM_OSCS;
    float frac      = pos - (int)pos;
    node            = n;
    nodeNext        = (n + 1) % NUM_OSCS;
    gain            = cosf_neon(frac * M_PI / 2.0f);
    gainNext        = sinf_neon(frac * M_PI / 2.0f);
}

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

    gui.setup(context->projectName);
    controller.setup(&gui, "Harmonic Resonator");

    gF0SliderIdx            = controller.addSlider("F0 (Hz)",            55.0,    0.0, 1024.0, 0.1);
    gDetuneSliderIdx        = controller.addSlider("Detune",            0.003,    0.0,    0.05, 0.0001);
    gGlobalPhaseSliderIdx   = controller.addSlider("Osc Phase",           0.0,    0.0,  6.2832, 0.001);
    gKSpreadSliderIdx       = controller.addSlider("Spread",              0.1,    0.0,    1.0, 0.001);
    gInputScanASliderIdx    = controller.addSlider("Input A Scan",        0.5,    0.0,    1.0, 0.001);
    gInputScanBSliderIdx    = controller.addSlider("Input B Scan",        0.5,    0.0,    1.0, 0.001);
    gOutputScanASliderIdx   = controller.addSlider("Output A Scan",       0.5,    0.0,    1.0, 0.001);
    gOutputScanBSliderIdx   = controller.addSlider("Output B Scan",       0.5,    0.0,    1.0, 0.001);
    gSpecAToBSliderIdx      = controller.addSlider("A bus -> B in",       0.0,    0.0,    2.0, 0.001);
    gSpecBToASliderIdx      = controller.addSlider("B bus -> A in",       0.0,    0.0,    2.0, 0.001);
    gNodeAttackSliderIdx    = controller.addSlider("Node Env Attack",     0.91,   0.9, 0.9999, 0.0001);
    gNodeDecaySliderIdx     = controller.addSlider("Node Env Decay",      0.92,   0.9, 0.9999, 0.0001);
    gInPeakASliderIdx       = controller.addSlider("In A Peak (1=clip)",  0.0,    0.0, 1.2, 0.001);
    gInPeakBSliderIdx       = controller.addSlider("In B Peak (1=clip)",  0.0,    0.0, 1.2, 0.001);
    gOutPeakASliderIdx      = controller.addSlider("Out A Peak (1=clip)", 0.0,    0.0, 1.2, 0.001);
    gOutPeakBSliderIdx      = controller.addSlider("Out B Peak (1=clip)", 0.0,    0.0, 1.2, 0.001);

    // Digital I/O setup
    pinMode(context, 0, SYNC_DIGITAL_A, INPUT);
    pinMode(context, 0, SYNC_DIGITAL_B, INPUT);

    AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
    Bela_scheduleAuxiliaryTask(meterTask);

    initOscillators(context, 55.0f, 0.0f);
    gPrevF0Center = 55.0f;
    gPrevDetune = 0.0f;
    return true;
}

void render(BelaContext *context, void *userData) {
    float f0Center      = controller.getSliderValue(gF0SliderIdx);
    float detune        = controller.getSliderValue(gDetuneSliderIdx);
    float globalPhase   = controller.getSliderValue(gGlobalPhaseSliderIdx);
    float KSpread       = controller.getSliderValue(gKSpreadSliderIdx);
    float inputScanA    = controller.getSliderValue(gInputScanASliderIdx);
    float inputScanB    = controller.getSliderValue(gInputScanBSliderIdx);
    float outputScanA   = controller.getSliderValue(gOutputScanASliderIdx);
    float outputScanB   = controller.getSliderValue(gOutputScanBSliderIdx);
    float specAToB      = controller.getSliderValue(gSpecAToBSliderIdx);
    float specBToA      = controller.getSliderValue(gSpecBToASliderIdx);
    float nodeAttack    = controller.getSliderValue(gNodeAttackSliderIdx);
    float nodeDecay     = controller.getSliderValue(gNodeDecaySliderIdx);
    const float energyReserve = gEnergyReserve;
    const float reserveCurve = energyReserve * energyReserve;
    const float activeBudgetScale = 0.2f + 1.8f * reserveCurve;
    const float replenishScale = 0.0003f + 0.02f * reserveCurve;
    const float leakScale = 0.004f * (1.0f - reserveCurve) + 0.00008f;
    const float budgetBase = 0.002f + 0.01f * reserveCurve;

    float f0A = f0Center * (1.0f - detune);
    float f0B = f0Center * (1.0f + detune);
    const bool phaseLocked = (detune < 1e-5f);

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

    (void)inputScanA;
    (void)inputScanB;

    int outNodeA, outNodeA_next, outNodeB, outNodeB_next;
    float gainNodeA, gainNextA, gainNodeB, gainNextB;
    computeOutputScan(outputScanA, outNodeA, outNodeA_next, gainNodeA, gainNextA);
    computeOutputScan(outputScanB, outNodeB, outNodeB_next, gainNodeB, gainNextB);

    for(unsigned int n = 0; n < context->digitalFrames; n++) {
        int syncA = digitalRead(context, n, SYNC_DIGITAL_A);
        int syncB = digitalRead(context, n, SYNC_DIGITAL_B);
        if(syncA && !gSyncPrev[SPEC_A]) resetSpectrumPhase(SPEC_A);
        if(syncB && !gSyncPrev[SPEC_B]) resetSpectrumPhase(SPEC_B);
        gSyncPrev[SPEC_A] = syncA ? 1 : 0;
        gSyncPrev[SPEC_B] = syncB ? 1 : 0;
    }

    for(int frame = 0; frame < context->audioFrames; frame++) {

        float oscOut[NUM_CHANNELS][NUM_OSCS];

        for(int ch = 0; ch < NUM_CHANNELS; ch++) {
            float allIn = audioRead(context, frame, ch);
            float inAbs = fabsf(allIn);
            gInPeakHold[ch] = fmaxf(inAbs, gInPeakHold[ch] * kPeakDecay);
            if(ch == SPEC_A)
                allIn += gFeedbackBus[SPEC_B] * specBToA;
            else
                allIn += gFeedbackBus[SPEC_A] * specAToB;

            advanceDriftLfo(context, ch);

            float nodeSin[NUM_OSCS];
            for(int i = 0; i < NUM_OSCS; i++) {
                float bandSig = processBiquad(bpFilters[ch][i],
                    allIn * excitationScale[i] * injectionBoost[i]);
                float target  = fabsf(bandSig);

                if(target > nodeEnvelope[ch][i])
                    nodeEnvelope[ch][i] = nodeAttack * nodeEnvelope[ch][i]
                                        + (1.0f - nodeAttack) * target;
                else
                    nodeEnvelope[ch][i] = nodeEffectiveDecay[i] * nodeEnvelope[ch][i]
                                        + (1.0f - nodeEffectiveDecay[i]) * target;

                float drift = nodeDriftAmount(ch, i);
                if(!phaseLocked || ch == SPEC_A) {
                    theta[ch][i] += omega[ch][i] * (1.0f + drift);
                    if(theta[ch][i] >  M_PI) theta[ch][i] -= 2.0f * M_PI;
                    if(theta[ch][i] < -M_PI) theta[ch][i] += 2.0f * M_PI;
                } else {
                    theta[ch][i] = theta[SPEC_A][i];
                }

                nodeSin[i] = sinf_neon(theta[ch][i] + globalPhase);
            }

            float prevEnv[NUM_OSCS];
            for(int i = 0; i < NUM_OSCS; i++) prevEnv[i] = nodeEnvelope[ch][i];

            for(int i = 0; i < NUM_OSCS; i++) {
                float maxContrib = 0.0f;
                for(int j = 0; j < NUM_OSCS; j++) {
                    float w = couplingWeights[i][j];
                    if(w == 0.0f) continue;
                    float contrib = prevEnv[j] * w;
                    if(contrib > maxContrib) maxContrib = contrib;
                }
                gCouplingDrive[ch][i] = kCouplingMemCoeff * gCouplingDrive[ch][i]
                                      + (1.0f - kCouplingMemCoeff) * maxContrib;
                nodeEnvelope[ch][i] = fminf(
                    fmaxf(nodeEnvelope[ch][i], gCouplingDrive[ch][i] * KSpread),
                    1.05f
                );
            }

            float inputAbs = fabsf(allIn);

            gChannelEnergy[ch] = fminf(gChannelEnergy[ch] + replenishScale * inputAbs, 2.2f);

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

            float spend = meanSq * (0.0012f + 0.0055f * KSpread);
            gChannelEnergy[ch] = fmaxf(gChannelEnergy[ch] - spend - leakScale, 0.0f);

            float busEnv = 0.0f;
            float busRaw = 0.0f;
            for(int i = 0; i < NUM_OSCS; i++) {
                oscOut[ch][i] = nodeSin[i] * nodeEnvelope[ch][i];
                busEnv += oscOut[ch][i] * loudnessWeight[i];
                busRaw += nodeSin[i] * loudnessWeight[i];
            }

            float actTarget = 0.0f;
            for(int i = 0; i < NUM_OSCS; i++)
                actTarget = fmaxf(actTarget, nodeEnvelope[ch][i]);
            actTarget = fmaxf(actTarget, fabsf(busEnv));

            float act = gFeedbackActivity[ch];
            float actCoeff = (actTarget > act) ? kFeedbackActAttack : kFeedbackActRelease;
            gFeedbackActivity[ch] = actCoeff * act + (1.0f - actCoeff) * actTarget;

            float gate = kFeedbackMinOpen
                       + (1.0f - kFeedbackMinOpen) * gFeedbackActivity[ch];
            float busTarget = kFeedbackBusGain * gate
                            * (busEnv + kFeedbackRawBleed * busRaw);
            float bus = gFeedbackBus[ch];
            float busCoeff = (fabsf(busTarget) > fabsf(bus)) ? kFeedbackAttack
                                                             : kFeedbackRelease;
            gFeedbackBus[ch] = busCoeff * bus + (1.0f - busCoeff) * busTarget;
        }

        float scanOutA = oscOut[SPEC_A][outNodeA] * gainNodeA
                       + oscOut[SPEC_A][outNodeA_next] * gainNextA;
        float scanOutB = oscOut[SPEC_B][outNodeB] * gainNodeB
                       + oscOut[SPEC_B][outNodeB_next] * gainNextB;

        float outA = scanOutA * amplitude;
        float outB = scanOutB * amplitude;
        gOutPeakHold[0] = fmaxf(fabsf(outA), gOutPeakHold[0] * kPeakDecay);
        gOutPeakHold[1] = fmaxf(fabsf(outB), gOutPeakHold[1] * kPeakDecay);

        audioWrite(context, frame, 0, outA);
        audioWrite(context, frame, 1, outB);

    }
}

void cleanup(BelaContext *context, void *userData) {}
