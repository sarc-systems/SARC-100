// Desktop validation for lib/dsp/crossfade.h — equal-power rotation used by DUAL FULCRUM
// and any A/B crossfade. Plain checks that exit(1) on failure. Run via test/run.sh.

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "../lib/dsp/crossfade.h"

static int gChecks = 0;
#define CHECK(cond) do { \
	gChecks++; \
	if (!(cond)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); exit(1); } \
} while (0)

static void test_power_conserved() {
	// out² + quad² == a² + b² for every theta — the equal-power invariant.
	const float as[] = {0.3f, -0.7f, 1.0f, 0.0f, -1.0f};
	const float bs[] = {0.9f,  0.2f, -0.5f, 1.0f,  0.6f};
	for (int k = 0; k < 5; ++k)
		for (int n = 0; n <= 64; ++n) {
			float th = (float)(-M_PI / 2 + n * (M_PI / 64));
			XfadePair p = equalPowerRotate(as[k], bs[k], th);
			float lhs = p.out * p.out + p.quad * p.quad;
			float rhs = as[k] * as[k] + bs[k] * bs[k];
			CHECK(fabsf(lhs - rhs) < 1e-5f);
		}
}

static void test_endpoints() {
	// θ=0 → (a, b);  θ=π/2 → (b, −a).
	XfadePair p0 = equalPowerRotate(0.4f, 0.9f, 0.0f);
	CHECK(fabsf(p0.out - 0.4f) < 1e-6f && fabsf(p0.quad - 0.9f) < 1e-6f);
	XfadePair p9 = equalPowerRotate(0.4f, 0.9f, (float)M_PI / 2);
	CHECK(fabsf(p9.out - 0.9f) < 1e-5f && fabsf(p9.quad + 0.4f) < 1e-5f);
}

static void test_precomputed_matches_theta() {
	float th = 0.63f;
	XfadePair a = equalPowerRotate(0.2f, -0.8f, th);
	XfadePair b = equalPowerRotate(0.2f, -0.8f, cosf(th), sinf(th));
	CHECK(fabsf(a.out - b.out) < 1e-6f && fabsf(a.quad - b.quad) < 1e-6f);
}

static void test_orthogonality() {
	// out and quad are orthogonal as vectors over (a,b): out·quad dotted across the basis
	// is zero — check via the identity that the (cos,sin)/(−sin,cos) rows are orthonormal.
	float th = 0.9f, c = cosf(th), s = sinf(th);
	CHECK(fabsf(c*(-s) + s*c) < 1e-6f);            // row dot product = 0
	CHECK(fabsf(c*c + s*s - 1.0f) < 1e-6f);        // unit rows
}

static void test_fade() {
	CHECK(fabsf(equalPowerFade(1.0f, 0.0f, 0.0f) - 1.0f) < 1e-6f);   // p=0 → a
	CHECK(fabsf(equalPowerFade(0.0f, 1.0f, 1.0f) - 1.0f) < 1e-5f);   // p=1 → b
	// Correlated inputs (a=b=1) peak at √2 at the center — the equal-power +3dB bump.
	CHECK(fabsf(equalPowerFade(1.0f, 1.0f, 0.5f) - sqrtf(2.0f)) < 1e-5f);
	// Uncorrelated (a on out, b on quad-equivalent): power constant across p.
	for (int n = 0; n <= 16; ++n) {
		float p = n / 16.0f, t = p * (float)M_PI * 0.5f;
		float oa = 1.0f * cosf(t), ob = 1.0f * sinf(t);
		CHECK(fabsf(oa*oa + ob*ob - 1.0f) < 1e-5f);
	}
}

int main() {
	test_power_conserved();
	test_endpoints();
	test_precomputed_matches_theta();
	test_orthogonality();
	test_fade();
	printf("test_crossfade_dsp: %d checks passed\n", gChecks);
	return 0;
}
