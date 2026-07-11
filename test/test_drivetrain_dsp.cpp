// Desktop validation for DRIVETRAIN's lib/dsp/ units (no Bela, no framework).
// Exercises the two correctness-critical pieces: cycle-counted rational phase
// division (continuity across master wraps + exact period) and Stern-Brocot ratio
// selection (value-sorted, nested, persistent across depth). Run via test/run.sh.

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "../lib/dsp/cycledivide.h"
#include "../lib/dsp/sternbrocot.h"

static int gChecks = 0;

#define CHECK(cond) do { \
	gChecks++; \
	if (!(cond)) { \
		fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		exit(1); \
	} \
} while (0)

// Smallest signed wrap of a phase difference into (-0.5, 0.5].
static double wrapDelta(double d) {
	d -= floor(d);
	if (d > 0.5) d -= 1.0;
	return d;
}

static void test_cycledivide_continuity() {
	// For several ratios, step the master phase finely across many master cycles
	// and confirm the sub phase advances by a CONSTANT increment every sample —
	// including at the samples that straddle a master wrap (C increments). A
	// per-cycle discontinuity (the bug we designed out) would show up as one
	// outlier delta per master cycle.
	const int pqs[][2] = {{1,1},{3,2},{2,3},{7,4},{5,1},{1,4},{11,8}};
	const double step = 1.0 / 257.0;   // master phase increment per "sample" (coprime-ish)

	for (auto &pq : pqs) {
		int p = pq[0], q = pq[1];
		double expected = (double)p / (double)q * step;   // exact sub-phase increment

		double phiM = 0.0;
		long long C = 0;
		double prevSub = cycledivide(p, q, C, phiM);
		int samples = 257 * q * 6;   // a few full macro-periods

		for (int n = 0; n < samples; ++n) {
			phiM += step;
			if (phiM >= 1.0) { phiM -= 1.0; C += 1; }
			double sub = cycledivide(p, q, C, phiM);
			double delta = wrapDelta(sub - prevSub);
			CHECK(fabs(delta - wrapDelta(expected)) < 1e-9);
			prevSub = sub;
		}
	}
}

static void test_cycledivide_exact_period() {
	// Over q master cycles the sub completes exactly p cycles: at C=q, phi=0 the
	// sub phase returns to 0, and the precomputed-constant overload agrees with
	// the convenience overload.
	const int pqs[][2] = {{3,2},{7,4},{2,3},{11,8}};
	for (auto &pq : pqs) {
		int p = pq[0], q = pq[1];
		CHECK(fabs(cycledivide(p, q, 0, 0.0)) < 1e-12);
		CHECK(fabs(cycledivide(p, q, (long long)q, 0.0)) < 1e-12);

		double a = cycledivide(p, q, 5, 0.371);
		double b = cycledivide(p, q, 5, 0.371, 1.0/(double)q, (double)p/(double)q);
		CHECK(fabs(a - b) < 1e-12);
	}
}

static void test_cycledivide_bounded() {
	// (C mod q) keeps the integer term bounded — a huge C must still give a sane,
	// finite phase identical to the same C reduced mod q (with phiM=0).
	int p = 7, q = 4;
	long long bigC = 1000000000LL;
	double hi = cycledivide(p, q, bigC, 0.0);
	double lo = cycledivide(p, q, bigC % q, 0.0);
	CHECK(fabs(hi - lo) < 1e-12);
	CHECK(hi >= 0.0 && hi < 1.0);
}

static SternBrocotTable<8192> gT;

static int findRatio(int p, int q) {
	for (int i = 0; i < gT.count(); ++i)
		if (gT.node(i).p == p && gT.node(i).q == q) return i;
	return -1;
}

static void test_sternbrocot_structure() {
	gT.build(0.25f, 4.0f, 12);
	CHECK(gT.count() > 0);

	// Value-sorted ascending, all within range, all in lowest terms.
	for (int i = 0; i < gT.count(); ++i) {
		const SBNode &n = gT.node(i);
		CHECK(n.value >= 0.25f && n.value <= 4.0f);
		CHECK(n.q > 0 && n.p > 0);
		// gcd(p,q) == 1
		int a = n.p, b = n.q;
		while (b) { int t = a % b; a = b; b = t; }
		CHECK(a == 1);
		if (i > 0) CHECK(gT.node(i).value >= gT.node(i-1).value);
	}

	// Canonical low-depth ratios are present at their expected shallow depths.
	CHECK(gT.node(findRatio(1,1)).depth == 1);   // root
	CHECK(findRatio(2,1) >= 0);                  // octave
	CHECK(findRatio(1,2) >= 0);
	CHECK(findRatio(3,2) >= 0);                  // fifth
	CHECK(findRatio(7,4) >= 0);                  // septimal
}

static void test_sternbrocot_nesting() {
	// Membership is nested: every node admitted at depth D is still admitted at
	// D+1 (count is monotone non-decreasing in the ceiling).
	SternBrocotTable<8192> t;
	int prev = 0;
	for (int D = 1; D <= 12; ++D) {
		t.build(0.25f, 4.0f, D);
		CHECK(t.count() >= prev);
		prev = t.count();
	}
}

static void test_sternbrocot_persistence() {
	// THE persistence guarantee (spec §6): a fixed RATIO target selects the SAME
	// ratio as the depth ceiling rises. Target exactly on 3/2 stays 3/2 at every
	// depth that admits it; never reflows to a neighbour.
	gT.build(0.25f, 4.0f, 12);
	float target = logf(1.5f);   // exactly 3/2
	int idx32 = findRatio(3, 2);
	int depth32 = gT.node(idx32).depth;

	int cur = -1;
	for (int D = depth32; D <= 12; ++D) {
		cur = gT.select(target, D, cur, 0.03f);
		CHECK(gT.node(cur).p == 3 && gT.node(cur).q == 2);
	}

	// Simple (3/2) and complex (7/4) coexist in a deep admitted set and are
	// independently reachable by their own targets.
	int sel32 = gT.select(logf(1.5f),  12, -1, 0.03f);
	int sel74 = gT.select(logf(1.75f), 12, -1, 0.03f);
	CHECK(gT.node(sel32).p == 3 && gT.node(sel32).q == 2);
	CHECK(gT.node(sel74).p == 7 && gT.node(sel74).q == 4);
}

static bool smooth7(int n) {
	for (int p = 2; p <= 7; ++p) while (n % p == 0) n /= p;
	return n == 1;
}

static int findRatioIn(const SternBrocotTable<8192> &t, int p, int q) {
	for (int i = 0; i < t.count(); ++i)
		if (t.node(i).p == p && t.node(i).q == q) return i;
	return -1;
}

static void test_sternbrocot_primelimit() {
	// 7-limit: every admitted ratio's p and q factor into primes <= 7; nothing with an
	// 11/13/... factor is admitted, but 7-limit ratios still are, and 1/1 is always in.
	SternBrocotTable<8192> t;
	t.build(0.25f, 4.0f, 12, 7);
	CHECK(t.count() > 0);
	bool has11 = false, has13 = false, has74 = false, has75 = false, has11_ratio = false;
	for (int i = 0; i < t.count(); ++i) {
		const SBNode &n = t.node(i);
		CHECK(smooth7(n.p) && smooth7(n.q));   // core invariant
		if (n.p == 7 && n.q == 4) has74 = true;
		if (n.p == 7 && n.q == 5) has75 = true;
		if (n.p == 11 || n.q == 11) has11 = true;
		if (n.p == 13 || n.q == 13) has13 = true;
		if (n.p == 11 && n.q == 8)  has11_ratio = true;
	}
	CHECK(has74 && has75);                 // 7-limit ratios present
	CHECK(!has11 && !has13 && !has11_ratio); // 11/13-limit excluded (11/8 would be depth 4 unfiltered)

	// Filtering never admits more than the unfiltered tree, and still includes 1/1.
	SternBrocotTable<8192> tAll;
	tAll.build(0.25f, 4.0f, 12);
	CHECK(t.count() < tAll.count());
	CHECK(findRatioIn(t, 1, 1) >= 0);
}

static int oddPart(int n) { while ((n & 1) == 0) n >>= 1; return n; }

static void test_sternbrocot_oddlimit() {
	// 9-odd-limit, 7-prime-limit (the drivetrain lattice): every ratio's odd parts are
	// <= 9, 9/8 is admitted (it's tree-depth 9, so a depth ceiling would miss it), and
	// higher odd-limit ratios like 15/8 are excluded.
	SternBrocotTable<8192> t;
	t.build(0.25f, 4.0f, 14, 7, 9);
	CHECK(t.count() > 0);
	for (int i = 0; i < t.count(); ++i) {
		const SBNode &n = t.node(i);
		CHECK(oddPart(n.p) <= 9 && oddPart(n.q) <= 9);
		CHECK(smooth7(n.p) && smooth7(n.q));
	}
	CHECK(findRatioIn(t, 9, 8) >= 0);    // the one we came for
	CHECK(findRatioIn(t, 10, 9) >= 0);   // 9-odd-limit neighbours
	CHECK(findRatioIn(t, 8, 7) >= 0);
	CHECK(findRatioIn(t, 15, 8) < 0);    // odd-limit 15 — excluded
	CHECK(findRatioIn(t, 1, 1) >= 0);
}

int main() {
	test_cycledivide_continuity();
	test_cycledivide_exact_period();
	test_cycledivide_bounded();
	test_sternbrocot_structure();
	test_sternbrocot_nesting();
	test_sternbrocot_persistence();
	test_sternbrocot_primelimit();
	test_sternbrocot_oddlimit();
	printf("test_drivetrain_dsp: %d checks passed\n", gChecks);
	return 0;
}
