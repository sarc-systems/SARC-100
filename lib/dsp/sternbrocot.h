#pragma once

#include <cmath>

// Stern-Brocot rational table for just-intonation ratio selection (DRIVETRAIN §6).
// Plain C++, no Bela dependency.
//
// build() enumerates every Stern-Brocot tree node of depth <= maxDepth whose value
// lies in [rMin, rMax], in value-sorted order (in-order traversal). The tree is
// rooted at the mediant 1/1 (depth 1); every positive rational appears exactly once
// in lowest terms.
//
// Membership is NESTED: a node admitted at depth D stays admitted at every deeper
// level, and deeper levels only insert new nodes in the GAPS between existing ones.
// select() therefore addresses by table index — a FIXED tree node (value-coordinate)
// — never by position in a depth-filtered sub-list. That gives the persistence
// guarantee: a fixed RATIO target maps to the SAME ratio as the depth ceiling rises.

struct SBNode {
	float value;
	float logValue;
	int   p;
	int   q;
	int   depth;
};

template <int MAXN>
class SternBrocotTable {
public:
	void build(float rMin, float rMax, int maxDepth) {
		n_ = 0;
		rMin_ = rMin;
		rMax_ = rMax;
		// Root interval (0/1, 1/0); mediant 1/1 sits at depth 1.
		recurse(0, 1, 1, 0, 1, maxDepth);
	}

	int           count()       const { return n_; }
	const SBNode &node(int i)    const { return nodes_[i]; }

	// Nearest admitted (depth <= maxD) node to targetLog (natural log of the desired
	// ratio), with hysteresis: keep curIdx unless another node beats it by more than
	// hystLog in log-ratio. Pass curIdx = -1 to force a fresh selection.
	int select(float targetLog, int maxD, int curIdx, float hystLog) const {
		int   best  = -1;
		float bestD = 1e30f;
		for(int i = 0; i < n_; ++i) {
			if(nodes_[i].depth > maxD) continue;
			float d = fabsf(nodes_[i].logValue - targetLog);
			if(d < bestD) { bestD = d; best = i; }
		}
		// Stick with the current node unless a clearly closer one exists, and only
		// if the current node is still admitted (it may not be if maxD dropped).
		if(curIdx >= 0 && curIdx < n_ && nodes_[curIdx].depth <= maxD) {
			float curD = fabsf(nodes_[curIdx].logValue - targetLog);
			if(curD <= bestD + hystLog) return curIdx;
		}
		return best;
	}

private:
	// Enumerate the subtree spanning open interval (a/b, c/d). Mediant (a+c)/(b+d)
	// is this node. In-order: left subtree, self, right subtree -> value-sorted.
	void recurse(int a, int b, int c, int d, int depth, int maxDepth) {
		if(depth > maxDepth) return;
		long p = (long)a + c;
		long q = (long)b + d;
		float val = (float)((double)p / (double)q);

		// Left subtree covers values < val; skip it entirely if none reach rMin.
		if(val > rMin_) recurse(a, b, (int)p, (int)q, depth + 1, maxDepth);

		if(val >= rMin_ && val <= rMax_ && n_ < MAXN) {
			nodes_[n_].value    = val;
			nodes_[n_].logValue = logf(val);
			nodes_[n_].p        = (int)p;
			nodes_[n_].q        = (int)q;
			nodes_[n_].depth    = depth;
			++n_;
		}

		// Right subtree covers values > val; skip if none reach rMax.
		if(val < rMax_) recurse((int)p, (int)q, c, d, depth + 1, maxDepth);
	}

	SBNode nodes_[MAXN];
	int    n_    = 0;
	float  rMin_ = 0.0f;
	float  rMax_ = 0.0f;
};
