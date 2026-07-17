// Desktop validation for the servo module's lib/dsp/ units. No Bela, no
// framework — plain checks that exit(1) on failure. Run via test/run.sh.
// Spline is no longer part of servo's lib/dsp/ usage — see test_spline_dsp.cpp.

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "../lib/dsp/pid.h"
#include "../lib/dsp/confidence.h"
#include "../lib/dsp/peak_hold.h"

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

// ---------------------------------------------------------------------------
// PID + simulated plant
// ---------------------------------------------------------------------------

// One-pole plant whose gain itself saturates, so some setpoints are
// physically unreachable no matter how hard the effector pushes.
struct SaturatingPlant {
	float gain = 2.0f;
	float satLimit = 1.0f;
	float alpha = 0.3f;
	float y = 0.0f;

	float step(float effector) {
		float target = std::min(std::max(gain * effector, -satLimit), satLimit);
		y += alpha * (target - y);
		return y;
	}
};

static void testPidConvergesOnReachableSetpoint() {
	PID pid;
	pid.setGains(0.8f, 0.5f, 0.0f);
	pid.setDerivativeFilter(1.0f);
	pid.setOutputLimits(-1.0f, 1.0f);

	SaturatingPlant plant;
	float setpoint = 0.5f; // reachable: plant maxes out at 1.0
	float dt = 0.01f;
	float error = 0.0f;
	for (int i = 0; i < 2000; i++) {
		error = setpoint - plant.y;
		float effector = pid.update(error, dt);
		plant.step(effector);
	}
	CHECK(nearlyEqual(plant.y, setpoint, 0.02f));
	printf("ok: PID converges on reachable setpoint (y=%.4f, target=%.4f)\n", plant.y, setpoint);
}

static void testPidAntiWindup() {
	PID pid;
	pid.setGains(0.8f, 0.5f, 0.0f);
	pid.setDerivativeFilter(1.0f);
	pid.setOutputLimits(-1.0f, 1.0f);

	SaturatingPlant plant;
	float dt = 0.01f;

	// Unreachable setpoint: plant can never produce more than 1.0 * alpha-settled,
	// so error never converges to zero and a naive integrator would wind up.
	float unreachable = 5.0f;
	for (int i = 0; i < 3000; i++) {
		float error = unreachable - plant.y;
		float effector = pid.update(error, dt);
		plant.step(effector);
	}
	// Anti-windup contract: the internal integral state stays bounded near
	// what's needed to hold the output at its limit, not run away unbounded.
	float boundedIntegralGuess = fabsf(1.0f / 0.5f) * 4.0f; // generous multiple of outMax/ki
	CHECK(fabsf(pid.integralState()) < boundedIntegralGuess);

	// Now make the setpoint reachable again and confirm prompt recovery —
	// a wound-up integrator would overshoot/lag badly here.
	SaturatingPlant plant2 = plant; // continue from the saturated state
	float reachable = 0.3f;
	int stepsToConverge = -1;
	for (int i = 0; i < 1000; i++) {
		float error = reachable - plant2.y;
		float effector = pid.update(error, dt);
		plant2.step(effector);
		if (stepsToConverge < 0 && nearlyEqual(plant2.y, reachable, 0.02f)) {
			stepsToConverge = i;
		}
	}
	CHECK(stepsToConverge >= 0);
	CHECK(stepsToConverge < 500); // prompt recovery, not a long unwind
	printf("ok: PID anti-windup bounded (integral=%.3f), recovered in %d steps\n",
		pid.integralState(), stepsToConverge);
}

static void testPidBumplessReset() {
	PID pid;
	pid.setGains(0.8f, 0.5f, 0.2f);
	pid.setDerivativeFilter(0.5f);
	pid.setOutputLimits(-1.0f, 1.0f);

	// Run it a bit so internal state isn't trivially zero.
	float dt = 0.01f;
	float error = 0.0f;
	for (int i = 0; i < 50; i++) {
		error = 0.4f - 0.1f * i * dt;
		pid.update(error, dt);
	}

	float seedOutput = 0.3f;
	pid.reset(seedOutput, error);
	float next = pid.update(error, dt);
	CHECK(nearlyEqual(next, seedOutput, 1e-3f));
	printf("ok: PID RESET=last is bumpless (next=%.5f, seed=%.5f)\n", next, seedOutput);
}

static void testPidBumplessGainChange() {
	// Sweeping a gain-scheduled ki (as SERVO RATE does) must not bump the output: the
	// integral is stored unscaled, so setGains rescales it to preserve ki*integral.
	PID pid;
	pid.setOutputLimits(-1.0f, 1.0f);
	pid.setGains(1.0f, 2.0f, 0.0f);   // kp, ki, kd

	// Build a steady holding output against a constant error.
	float dt = 0.001f;
	float out = 0.0f;
	for (int i = 0; i < 500; i++) out = pid.update(0.2f, dt);

	// Change ki sharply (2 -> 5). Without bumpless rescaling, ki*integral would jump.
	pid.setGains(1.0f, 5.0f, 0.0f);
	float outAfter = pid.update(0.2f, dt);
	CHECK(nearlyEqual(outAfter, out, 5e-3f));
	printf("ok: PID gain change is bumpless (%.4f -> %.4f on ki 2->5)\n", out, outAfter);
}

// ---------------------------------------------------------------------------
// Confidence
// ---------------------------------------------------------------------------

static int stepsTo63Percent(Confidence &c, float success, float rate, int maxSteps) {
	for (int i = 0; i < maxSteps; i++) {
		float v = c.update(success, rate);
		if (v >= 0.63f) return i;
	}
	return -1;
}

static void testConfidenceRisesAndDecays() {
	Confidence c;
	float rate = 0.05f;
	float prev = c.value();
	bool monotonic = true;
	for (int i = 0; i < 200; i++) {
		float v = c.update(1.0f, rate);
		if (v < prev - 1e-6f) monotonic = false;
		prev = v;
	}
	CHECK(monotonic);
	CHECK(c.value() > 0.9f);

	float peak = c.value();
	for (int i = 0; i < 200; i++) {
		c.update(0.0f, rate);
	}
	CHECK(c.value() < peak);
	printf("ok: confidence rises under success, decays under failure\n");
}

static void testConfidenceSlowerThanLoop() {
	// The "loop's own integral time constant" is modeled here as the same
	// leaky-integrator step using ki directly (rate=ki), matching how the
	// PID's integral term itself evolves toward a steady error contribution.
	// Confidence's rate must be slower (GOVERNOR_TIME_MULT < 1 applied to it).
	float loopKi = 0.5f;
	float governorTimeMult = 0.1f;
	float confidenceRate = loopKi * governorTimeMult;
	CHECK(confidenceRate < loopKi);

	Confidence loopLike;
	Confidence confidence;
	int loopSteps = stepsTo63Percent(loopLike, 1.0f, loopKi, 100000);
	int confidenceSteps = stepsTo63Percent(confidence, 1.0f, confidenceRate, 100000);
	CHECK(loopSteps >= 0 && confidenceSteps >= 0);
	CHECK(confidenceSteps > loopSteps);
	printf("ok: confidence time-to-63%% (%d steps) slower than loop's (%d steps)\n",
		confidenceSteps, loopSteps);
}

// ---------------------------------------------------------------------------
// PeakHold (EFF pin / saturation detector)
// ---------------------------------------------------------------------------

static void testPeakHoldSnapsAndFades() {
	// callPeriod 1 ms, hold time constant 0.25 s.
	PeakHold p;
	p.init(0.001f, 0.25f);

	// A single event snaps the output straight to the peak.
	CHECK(nearlyEqual(p.process(1.0f), 1.0f, 1e-6f));

	// With no further events it decays monotonically toward 0, staying below the peak.
	float prev = p.value();
	bool monotonicDown = true;
	for (int i = 0; i < 250; i++) {
		float v = p.process(0.0f);
		if (v > prev + 1e-6f) monotonicDown = false;
		prev = v;
	}
	CHECK(monotonicDown);
	CHECK(p.value() < 1.0f);
	// ~one time constant of decay (250 * 1 ms = 0.25 s) lands near 1/e.
	CHECK(nearlyEqual(p.value(), expf(-1.0f), 0.02f));
	printf("ok: peak-hold snaps to peak and decays ~1/e per time constant\n");
}

static void testPeakHoldRetriggerHolds() {
	// Sustained events hold the output pinned at the peak (no sag between hits).
	PeakHold p;
	p.init(0.001f, 0.25f);
	for (int i = 0; i < 500; i++) p.process(1.0f);
	CHECK(nearlyEqual(p.value(), 1.0f, 1e-6f));

	// A higher peak overrides a lower held value immediately.
	PeakHold q;
	q.init(0.001f, 0.25f);
	q.process(0.3f);
	CHECK(nearlyEqual(q.process(0.9f), 0.9f, 1e-6f));
	printf("ok: peak-hold stays pinned while retriggered and rises to new peaks\n");
}

int main() {
	testPidConvergesOnReachableSetpoint();
	testPidAntiWindup();
	testPidBumplessReset();
	testPidBumplessGainChange();
	testConfidenceRisesAndDecays();
	testConfidenceSlowerThanLoop();
	testPeakHoldSnapsAndFades();
	testPeakHoldRetriggerHolds();
	printf("PASS: %d checks\n", gChecks);
	return 0;
}
