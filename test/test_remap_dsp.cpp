// Desktop validation for lib/dsp/remap.h — the REMAP voltage-transform core.
// No Bela, no framework — plain checks that exit(1) on failure. Run via test/run.sh.
// Encodes every verification case from modules/remap/CLAUDE.md so the "not yet
// confirmed" FOLD formulas are confirmed against all stated behavioral requirements.

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "../lib/dsp/remap.h"

using namespace remap;

static int gChecks = 0;

#define CHECK(cond) do { \
	gChecks++; \
	if (!(cond)) { \
		fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		exit(1); \
	} \
} while (0)

static bool nearlyEqual(float a, float b, float tol) {
	return fabsf(a - b) <= tol;
}

static const float V = 5.0f;
static const float TOL = 1e-4f;

// Convenience wrappers matching the spec's table names.
static float rot1(float x, float theta) { return transform(ROTATE, x, theta, V, 1); }
static float rot2(float x, float theta) { return transform(ROTATE, x, theta, V, 2); }
static float mir1(float x, float theta) { return transform(MIRROR, x, theta, V, 1); }
static float mir2(float x, float theta) { return transform(MIRROR, x, theta, V, 2); }
static float fold1(float x, float theta) { return transform(FOLD, x, theta, V, 1); }
static float fold2(float x, float theta) { return transform(FOLD, x, theta, V, 2); }

// ---------------------------------------------------------------------------
// wrapf / foldf primitives
// ---------------------------------------------------------------------------
static void testFlooredModulo() {
	// Floored modulo: always in [0, m), including negatives (unlike C fmod).
	CHECK(nearlyEqual(wrapf(7.0f, 5.0f), 2.0f, TOL));
	CHECK(nearlyEqual(wrapf(-1.0f, 5.0f), 4.0f, TOL));
	CHECK(nearlyEqual(wrapf(-6.0f, 5.0f), 4.0f, TOL));
	CHECK(nearlyEqual(wrapf(5.0f, 5.0f), 0.0f, TOL));   // exact multiple wraps to 0
	// foldf into [0, range].
	CHECK(nearlyEqual(foldf(6.0f, 5.0f), 4.0f, TOL));
	CHECK(nearlyEqual(foldf(3.0f, 2.5f), 2.0f, TOL));
	printf("ok: wrapf floored-modulo and foldf into range\n");
}

// ---------------------------------------------------------------------------
// Spot values from the spec (V = 5)
// ---------------------------------------------------------------------------
static void testSpotValues() {
	CHECK(nearlyEqual(rot1(4.0f, 3.0f), 2.0f, TOL));   // wrapf(7,5)=2
	CHECK(nearlyEqual(mir1(1.0f, 0.0f), 4.0f, TOL));   // wrapf(-1,5)=4
	CHECK(nearlyEqual(fold1(6.0f, 0.0f), 4.0f, TOL));  // foldf(6,5)=4
	CHECK(nearlyEqual(fold2(3.0f, 0.0f), 4.0f, TOL));  // 2*foldf(3,2.5)=4
	printf("ok: spec spot values (rot/mir/fold x1/x2)\n");
}

// ---------------------------------------------------------------------------
// FOLD x1 = |X| for X in [-V, V] with theta=0 (full-wave rectifier)
// ---------------------------------------------------------------------------
static void testFoldRectifier() {
	CHECK(nearlyEqual(fold1(-2.5f, 0.0f), 2.5f, TOL));  // spec's named case
	for (float x = -V; x <= V; x += 0.1f)
		CHECK(nearlyEqual(fold1(x, 0.0f), fabsf(x), TOL));
	printf("ok: FOLD x1 is full-wave rectifier |X| on [-V,V]\n");
}

// ---------------------------------------------------------------------------
// Rising ramp X: 0->V, theta=0
// ---------------------------------------------------------------------------
static void testRampBehaviors() {
	for (float x = 0.0f; x < V; x += 0.05f) {
		CHECK(nearlyEqual(rot1(x, 0.0f), x, TOL));        // ROTATE x1 = identity
		CHECK(nearlyEqual(fold1(x, 0.0f), x, TOL));       // FOLD x1 = rising ramp = X
		if (x > TOL)
			CHECK(nearlyEqual(mir1(x, 0.0f), V - x, TOL)); // MIRROR x1 = falling ramp
	}
	// x2 outputs: same shape at 2x frequency -> value repeats after half the input span.
	for (float x = 0.0f; x < V / 2.0f; x += 0.05f) {
		CHECK(nearlyEqual(rot2(x, 0.0f), rot2(x + V / 2.0f, 0.0f), TOL));
		CHECK(nearlyEqual(mir2(x, 0.0f), mir2(x + V / 2.0f, 0.0f), TOL));
	}
	printf("ok: ramp identity/rectify/mirror + x2 double-frequency periodicity\n");
}

// ---------------------------------------------------------------------------
// FOLD x1 sweep theta: rising ramp -> triangle -> falling ramp, continuously
// ---------------------------------------------------------------------------
static void testFoldSweepTheta() {
	// theta=0: rising ramp (X at endpoints).
	CHECK(nearlyEqual(fold1(0.0f, 0.0f), 0.0f, TOL));
	CHECK(nearlyEqual(fold1(V, 0.0f), V, TOL));
	// theta=V/2: triangle, peak at X=V/2.
	CHECK(nearlyEqual(fold1(0.0f, V / 2.0f), V / 2.0f, TOL));
	CHECK(nearlyEqual(fold1(V / 2.0f, V / 2.0f), V, TOL));    // apex
	CHECK(nearlyEqual(fold1(V, V / 2.0f), V / 2.0f, TOL));
	// theta=V: falling ramp.
	CHECK(nearlyEqual(fold1(0.0f, V), V, TOL));
	CHECK(nearlyEqual(fold1(V, V), 0.0f, TOL));
	// Continuity in theta: no jumps as theta sweeps (fold is continuous in its argument).
	float prev = fold1(1.3f, -V);
	for (float theta = -V; theta <= V; theta += 0.01f) {
		float cur = fold1(1.3f, theta);
		CHECK(fabsf(cur - prev) < 0.05f);   // small theta step -> small output step
		prev = cur;
	}
	printf("ok: FOLD x1 sweeps rising->triangle->falling continuously in theta\n");
}

// ---------------------------------------------------------------------------
// Algebraic equivalences from the spec table
// ---------------------------------------------------------------------------
static void testEquivalences() {
	for (float x = -12.3f; x <= 12.3f; x += 0.37f) {
		for (float theta = -7.1f; theta <= 7.1f; theta += 0.53f) {
			CHECK(nearlyEqual(rot2(x, theta), wrapf(2.0f * (x + theta), V), TOL));
			CHECK(nearlyEqual(mir2(x, theta), wrapf(2.0f * (theta - x), V), TOL));
			CHECK(nearlyEqual(fold1(x, theta), V - fabsf(wrapf(x + theta, 2.0f * V) - V), TOL));
			CHECK(nearlyEqual(fold2(x, theta), V - fabsf(2.0f * wrapf(x + theta, V) - V), TOL));
		}
	}
	printf("ok: x2/fold algebraic equivalences hold\n");
}

// ---------------------------------------------------------------------------
// Never clip: every output stays in [0, V] for any X, theta (incl. huge/negative)
// ---------------------------------------------------------------------------
static void testNeverClip() {
	float out[kNumOutputs];
	for (float x = -100.0f; x <= 100.0f; x += 1.7f) {
		for (float theta = -100.0f; theta <= 100.0f; theta += 3.3f) {
			computeAll(x, theta, V, out);
			for (int i = 0; i < kNumOutputs; ++i)
				CHECK(out[i] >= -TOL && out[i] <= V + TOL);
		}
	}
	printf("ok: all %d outputs bounded to [0,V] for extreme X,theta (never clip)\n", kNumOutputs);
}

// ---------------------------------------------------------------------------
// computeAll table matches individual transform() calls; input/output mapping helpers
// ---------------------------------------------------------------------------
static void testTableAndConditioning() {
	float out[kNumOutputs];
	computeAll(2.2f, -1.1f, V, out);
	CHECK(nearlyEqual(out[0], rot1(2.2f, -1.1f), TOL));
	CHECK(nearlyEqual(out[1], rot2(2.2f, -1.1f), TOL));
	CHECK(nearlyEqual(out[2], mir1(2.2f, -1.1f), TOL));
	CHECK(nearlyEqual(out[3], mir2(2.2f, -1.1f), TOL));
	CHECK(nearlyEqual(out[4], fold1(2.2f, -1.1f), TOL));
	CHECK(nearlyEqual(out[5], fold2(2.2f, -1.1f), TOL));
	CHECK(kNumOutputs == 6);

	// Input expansion: unipolar ADC [0,1] -> bipolar volts [-V,+V] (X and theta alike).
	CHECK(nearlyEqual(expandBipolar(0.0f, V), -V, TOL));
	CHECK(nearlyEqual(expandBipolar(0.5f, V), 0.0f, TOL));
	CHECK(nearlyEqual(expandBipolar(1.0f, V), V, TOL));

	// Transform output mapping: [0,V] -> normalized [0,1] (center V/2 -> 0.5 -> 0 V jack).
	CHECK(nearlyEqual(mapOutput(0.0f, V), 0.0f, TOL));
	CHECK(nearlyEqual(mapOutput(V / 2.0f, V), 0.5f, TOL));
	CHECK(nearlyEqual(mapOutput(V, V), 1.0f, TOL));

	// theta output mapping: bipolar [-V,+V] -> normalized [0,1] (0 V -> 0.5 -> 0 V jack).
	CHECK(nearlyEqual(mapBipolarOutput(-V, V), 0.0f, TOL));
	CHECK(nearlyEqual(mapBipolarOutput(0.0f, V), 0.5f, TOL));
	CHECK(nearlyEqual(mapBipolarOutput(V, V), 1.0f, TOL));
	printf("ok: computeAll table, input expansion, output mapping\n");
}

int main() {
	testFlooredModulo();
	testSpotValues();
	testFoldRectifier();
	testRampBehaviors();
	testFoldSweepTheta();
	testEquivalences();
	testNeverClip();
	testTableAndConditioning();
	printf("PASS: %d checks\n", gChecks);
	return 0;
}
