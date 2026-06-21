// Desktop validation for the SPLINE module's lib/dsp/ unit. No Bela, no
// framework — plain checks that exit(1) on failure. Run via test/run.sh.

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <initializer_list>

#include "../lib/dsp/spline.h"

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

static void setSixKnots(Spline<6> &s, const float values[6]) {
	s.setKnots(values);
}

static void testKnotExactnessAcrossInterpolation() {
	Spline<6> s;
	float knots[6] = {0.1f, 0.9f, -0.4f, 0.6f, 0.0f, 0.3f};
	setSixKnots(s, knots);
	for (float interp : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) {
		s.setInterpolation(interp);
		for (int i = 0; i < 6; i++) {
			float x = (float)i / 5.0f;
			CHECK(nearlyEqual(s.evaluate(x), knots[i], 1e-4f));
		}
	}
	printf("ok: knot exactness at all 6 x-positions, across interpolation settings\n");
}

static void testDiscontinuousIsNearestNeighbor() {
	Spline<6> s;
	float knots[6] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
	setSixKnots(s, knots);
	s.setInterpolation(0.0f);

	for (int seg = 0; seg < 5; seg++) {
		float x0 = seg / 5.0f;
		float x1 = (seg + 1) / 5.0f;
		// First half of the segment holds the left knot, second half the right knot.
		CHECK(nearlyEqual(s.evaluate(x0 + 0.01f * (x1 - x0)), knots[seg], 1e-4f));
		CHECK(nearlyEqual(s.evaluate(x0 + 0.49f * (x1 - x0)), knots[seg], 1e-4f));
		CHECK(nearlyEqual(s.evaluate(x0 + 0.51f * (x1 - x0)), knots[seg + 1], 1e-4f));
		CHECK(nearlyEqual(s.evaluate(x0 + 0.99f * (x1 - x0)), knots[seg + 1], 1e-4f));
	}
	// The very last knot must be reachable, not just the second-to-last (an open curve with
	// nearest-neighbor discontinuous mode should hit every knot, including the endpoints).
	CHECK(nearlyEqual(s.evaluate(1.0f), knots[5], 1e-4f));
	printf("ok: discontinuous mode holds the nearest knot, reaches every knot including the ends\n");
}

static void testLinearInterpolatesBetweenKnots() {
	Spline<6> s;
	float knots[6] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
	setSixKnots(s, knots);
	s.setInterpolation(0.5f);

	for (int seg = 0; seg < 5; seg++) {
		float p0 = knots[seg];
		float p1 = knots[seg + 1];
		for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
			float x = (seg + t) / 5.0f;
			float expected = p0 + t * (p1 - p0);
			CHECK(nearlyEqual(s.evaluate(x), expected, 1e-3f));
		}
	}
	printf("ok: linear mode (interp=0.5) interpolates exactly between adjacent knots\n");
}

static void testSmoothIsContinuousNoNaN() {
	Spline<6> s;
	float knots[6] = {0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f}; // zigzag, stresses curvature
	setSixKnots(s, knots);
	s.setInterpolation(1.0f);

	for (float x = -0.5f; x <= 1.5f; x += 0.01f) {
		CHECK(std::isfinite(s.evaluate(x)));
	}

	// C1 continuity at interior knots: left/right finite-difference slopes should agree
	// closely (Catmull-Rom-style cardinal cubic is C1). Every knot here is a local
	// extremum (the zigzag's true tangent is exactly 0), so finite-difference slopes pick
	// up curvature, not just the tangent — eps must be small enough that this higher-order
	// term stays well under the tolerance (confirmed it shrinks proportionally with eps,
	// not a real discontinuity).
	const float eps = 1e-4f;
	for (float knotX : {0.2f, 0.4f, 0.6f, 0.8f}) {
		float slopeLeft = (s.evaluate(knotX) - s.evaluate(knotX - eps)) / eps;
		float slopeRight = (s.evaluate(knotX + eps) - s.evaluate(knotX)) / eps;
		CHECK(nearlyEqual(slopeLeft, slopeRight, 0.05f));
	}
	printf("ok: smooth mode (interp=1) is continuous (C1 at knots), no NaN/inf\n");
}

static void testOpenCurveClampsAtEnds() {
	Spline<6> s;
	float knots[6] = {0.1f, 0.9f, -0.4f, 0.6f, 0.0f, 0.3f};
	setSixKnots(s, knots);

	for (float interp : {0.0f, 0.5f, 1.0f}) {
		s.setInterpolation(interp);
		// Open curve — out-of-range x clamps to the nearest end, it does NOT wrap around.
		CHECK(nearlyEqual(s.evaluate(-0.5f), s.evaluate(0.0f), 1e-4f));
		CHECK(nearlyEqual(s.evaluate(1.5f), s.evaluate(1.0f), 1e-4f));
		// In particular, clamping at x=0 must NOT equal the last knot (that would be wrapping).
		CHECK(!nearlyEqual(s.evaluate(-0.5f), knots[5], 1e-3f) || nearlyEqual(knots[0], knots[5], 1e-3f));
	}
	printf("ok: open curve clamps at the ends instead of wrapping\n");
}

static void testInterpolationMorphsContinuously() {
	Spline<6> s;
	float knots[6] = {0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f}; // zigzag, stresses the morph
	setSixKnots(s, knots);

	float x = (0.0f + 0.25f) / 5.0f; // quarter-way through segment 0
	float discontinuous = knots[0]; // frac=0.25 < 0.5 -> nearest is the left knot
	float linear = knots[0] + 0.25f * (knots[1] - knots[0]);

	s.setInterpolation(0.0f);
	CHECK(nearlyEqual(s.evaluate(x), discontinuous, 1e-4f));
	s.setInterpolation(0.5f);
	CHECK(nearlyEqual(s.evaluate(x), linear, 1e-4f));

	// At interp=0.25 (halfway between discontinuous and linear), should be the midpoint.
	s.setInterpolation(0.25f);
	float expectedQuarter = discontinuous + 0.5f * (linear - discontinuous);
	CHECK(nearlyEqual(s.evaluate(x), expectedQuarter, 1e-4f));

	printf("ok: interpolation control morphs continuously, not a hard switch\n");
}

int main() {
	testKnotExactnessAcrossInterpolation();
	testDiscontinuousIsNearestNeighbor();
	testLinearInterpolatesBetweenKnots();
	testSmoothIsContinuousNoNaN();
	testOpenCurveClampsAtEnds();
	testInterpolationMorphsContinuously();
	printf("PASS: %d checks\n", gChecks);
	return 0;
}
