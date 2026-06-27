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
#include <stdio.h>
#include <math.h>
#ifdef __INTELLISENSE__
#include "./stubs/math_neon.h"
#else
#include <libraries/math_neon/math_neon.h>
#endif

#include "pins.h"

#define NUM_PLAYERS 3

// -- Hardcoded physics constants (CLAUDE.md: "TBD during Bela prototype tuning") ----------
// Normalized space: center at origin, r_max defines the boundary circle.
const float kRMax        = 1.0f;    // boundary radius
const float kRStart      = 0.5f;    // initial spawn radius (equally spaced at 120° on this circle)
const float kCatchRadius = 0.15f;   // distance threshold for tag transfer
const float kWallSpring  = 50.0f;   // restoring force coefficient past r_max (units/s² per unit past wall)

// SPEED maps 0..1 bias+cv sum to 0..kMaxSpeed acceleration magnitude.
// DAMPING maps 0..1 to 0..kMaxDamping viscous drag coefficient (s⁻¹).
// At default (0.5 bias, no CV): speed ≈ 2.5 units/s², damping ≈ 2.5 s⁻¹ → terminal velocity ≈ 1 unit/s,
// crossing r_max in roughly 1 second — a usable musical timescale.
const float kMaxSpeed   = 5.0f;
const float kMaxDamping = 5.0f;
// -----------------------------------------------------------------------------------------

Gui gui;
GuiController controller;

// Bias sliders (pull-pots, standing in for panel knobs — attenuverters are on the hardware
// panel, not modeled in the GUI per the established TURBINE-era convention).
unsigned int gP1SpeedBiasSliderIdx;
unsigned int gP2SpeedBiasSliderIdx;
unsigned int gP3SpeedBiasSliderIdx;
unsigned int gDampingBiasSliderIdx;

// Display-only readouts — aux meter task only, never read back as control input.
unsigned int gP1SpeedInDisplayIdx, gP2SpeedInDisplayIdx, gP3SpeedInDisplayIdx;
unsigned int gDampingInDisplayIdx;
unsigned int gP1RDisplayIdx, gP1TDisplayIdx;
unsigned int gP2RDisplayIdx, gP2TDisplayIdx;
unsigned int gP3RDisplayIdx, gP3TDisplayIdx;
unsigned int gITDisplayIdx;

// -- Game / physics state ------------------------------------------------------------------

struct Player {
	float x, y;    // position (normalized, origin at center)
	float vx, vy;  // velocity
};

Player gPlayers[NUM_PLAYERS];
int gIT = 0;             // which player is currently IT (0..2)
int gNoTagBack = -1;     // player index that gIT cannot target (-1 = no restriction)

int gResetPrev = 0;

// CV meter display globals — written by render(), read by meterGuiTask
float gSpeedIn[NUM_PLAYERS] = {0.0f};
float gDampingIn = 0.0f;
float gROut[NUM_PLAYERS]    = {0.0f};
float gThetaOut[NUM_PLAYERS] = {0.0f};

// -----------------------------------------------------------------------------------------

inline int analogFrameForAudio(BelaContext *ctx, int audioFrame) {
	if(ctx->analogFrames == 0) return 0;
	return (audioFrame * (int)ctx->analogFrames) / (int)ctx->audioFrames;
}

inline float dist2(const Player &a, const Player &b) {
	float dx = a.x - b.x, dy = a.y - b.y;
	return sqrtf(dx*dx + dy*dy);
}

void resetPositions() {
	// Equally spaced 120° apart on a circle at kRStart; IT state preserved per CLAUDE.md.
	for(int i = 0; i < NUM_PLAYERS; i++) {
		float angle = (float)i * (2.0f * (float)M_PI) / (float)NUM_PLAYERS;
		gPlayers[i].x = kRStart * cosf(angle);
		gPlayers[i].y = kRStart * sinf(angle);
		gPlayers[i].vx = 0.0f;
		gPlayers[i].vy = 0.0f;
	}
	gNoTagBack = -1;
}

void meterGuiTask(void *) {
	while(!Bela_stopRequested()) {
		for(int i = 0; i < NUM_PLAYERS; i++) {
			const unsigned int speedIdx[NUM_PLAYERS] = {gP1SpeedInDisplayIdx, gP2SpeedInDisplayIdx, gP3SpeedInDisplayIdx};
			const unsigned int rIdx[NUM_PLAYERS]     = {gP1RDisplayIdx, gP2RDisplayIdx, gP3RDisplayIdx};
			const unsigned int tIdx[NUM_PLAYERS]     = {gP1TDisplayIdx, gP2TDisplayIdx, gP3TDisplayIdx};
			controller.setSliderValue(speedIdx[i], gSpeedIn[i]);
			controller.setSliderValue(rIdx[i],     gROut[i]);
			controller.setSliderValue(tIdx[i],     gThetaOut[i]);
		}
		controller.setSliderValue(gDampingInDisplayIdx, gDampingIn);
		controller.setSliderValue(gITDisplayIdx, (float)(gIT + 1)); // display 1..3
#ifdef __INTELLISENSE__
		for(volatile int d = 0; d < 80000; ++d) {}
#else
		usleep(80000);
#endif
	}
}

bool setup(BelaContext *context, void *userData) {
	gui.setup(context->projectName);
	controller.setup(&gui, "Tag");

	gP1SpeedBiasSliderIdx = controller.addSlider("P1 Speed Bias",  0.5, 0.0, 1.0, 0.001);
	gP2SpeedBiasSliderIdx = controller.addSlider("P2 Speed Bias",  0.5, 0.0, 1.0, 0.001);
	gP3SpeedBiasSliderIdx = controller.addSlider("P3 Speed Bias",  0.5, 0.0, 1.0, 0.001);
	gDampingBiasSliderIdx = controller.addSlider("Damping Bias",   0.5, 0.0, 1.0, 0.001);

	gP1SpeedInDisplayIdx = controller.addSlider("P1 Speed IN (A0)", 0.0, 0.0, 1.0, 0.001);
	gP2SpeedInDisplayIdx = controller.addSlider("P2 Speed IN (A1)", 0.0, 0.0, 1.0, 0.001);
	gP3SpeedInDisplayIdx = controller.addSlider("P3 Speed IN (A2)", 0.0, 0.0, 1.0, 0.001);
	gDampingInDisplayIdx = controller.addSlider("Damping IN (A3)",  0.0, 0.0, 1.0, 0.001);

	gP1RDisplayIdx = controller.addSlider("P1 r (eff)", 0.0, 0.0, 1.0, 0.001);
	gP1TDisplayIdx = controller.addSlider("P1 θ (eff)", 0.0, 0.0, 1.0, 0.001);
	gP2RDisplayIdx = controller.addSlider("P2 r (eff)", 0.0, 0.0, 1.0, 0.001);
	gP2TDisplayIdx = controller.addSlider("P2 θ (eff)", 0.0, 0.0, 1.0, 0.001);
	gP3RDisplayIdx = controller.addSlider("P3 r (eff)", 0.0, 0.0, 1.0, 0.001);
	gP3TDisplayIdx = controller.addSlider("P3 θ (eff)", 0.0, 0.0, 1.0, 0.001);

	gITDisplayIdx = controller.addSlider("IT player (1/2/3)", 1.0, 1.0, 3.0, 1.0);

	pinMode(context, 0, DIGITAL_RESET, INPUT);
	pinMode(context, 0, DIGITAL_FREEZE, INPUT);
	for(int pin : {DIGITAL_IT1, DIGITAL_IT2, DIGITAL_IT3, DIGITAL_TAG,
	               DIGITAL_WALL1, DIGITAL_WALL2, DIGITAL_WALL3, DIGITAL_WALLANY}) {
		pinMode(context, 0, pin, OUTPUT);
	}

	AuxiliaryTask meterTask = Bela_createAuxiliaryTask(meterGuiTask, 0, "meter-gui", NULL);
	Bela_scheduleAuxiliaryTask(meterTask);

	resetPositions();
	return true;
}

void render(BelaContext *context, void *userData) {
	float dt = 1.0f / (float)context->audioSampleRate;

	// Speed and damping parameters — block-rate read (bias knobs, slow-varying CVs).
	float speedCv[NUM_PLAYERS] = {0.0f};
	float dampingCv = 0.0f;
	if(context->analogFrames > 0) {
		if(ANALOG_P1_SPEED_CV < context->analogInChannels) speedCv[0] = analogRead(context, 0, ANALOG_P1_SPEED_CV);
		if(ANALOG_P2_SPEED_CV < context->analogInChannels) speedCv[1] = analogRead(context, 0, ANALOG_P2_SPEED_CV);
		if(ANALOG_P3_SPEED_CV < context->analogInChannels) speedCv[2] = analogRead(context, 0, ANALOG_P3_SPEED_CV);
		if(ANALOG_DAMPING_CV  < context->analogInChannels) dampingCv  = analogRead(context, 0, ANALOG_DAMPING_CV);
	}
	for(int i = 0; i < NUM_PLAYERS; i++) gSpeedIn[i] = speedCv[i];
	gDampingIn = dampingCv;

	const float speedBias[NUM_PLAYERS] = {
		controller.getSliderValue(gP1SpeedBiasSliderIdx),
		controller.getSliderValue(gP2SpeedBiasSliderIdx),
		controller.getSliderValue(gP3SpeedBiasSliderIdx),
	};
	float dampingBias = controller.getSliderValue(gDampingBiasSliderIdx);

	// Effective parameters — bias + cv, clamped to [0,1], then scaled to physics units.
	float speed[NUM_PLAYERS];
	for(int i = 0; i < NUM_PLAYERS; i++) {
		float s = speedBias[i] + speedCv[i];
		if(s < 0.0f) s = 0.0f; if(s > 1.0f) s = 1.0f;
		speed[i] = s * kMaxSpeed;
	}
	float dampingBiasEff = dampingBias + dampingCv;
	if(dampingBiasEff < 0.0f) dampingBiasEff = 0.0f;
	if(dampingBiasEff > 1.0f) dampingBiasEff = 1.0f;
	float damping = dampingBiasEff * kMaxDamping;

	for(int frame = 0; frame < context->audioFrames; frame++) {

		// Digital reads — sample-accurate (same-frame SYNC/FREEZE ordering per CLAUDE.md).
		int resetPin  = digitalRead(context, frame, DIGITAL_RESET);
		int freezePin = digitalRead(context, frame, DIGITAL_FREEZE);

		// RESET (rising edge) — reinitializes positions, preserves IT state.
		if(resetPin && !gResetPrev) resetPositions();
		gResetPrev = resetPin ? 1 : 0;

		// FREEZE (level) — zero all velocities, skip integration; positions held.
		if(freezePin) {
			for(int i = 0; i < NUM_PLAYERS; i++) {
				gPlayers[i].vx = 0.0f;
				gPlayers[i].vy = 0.0f;
			}
			// Still write outputs so CV outs reflect the frozen positions.
		} else {

			// -- No-tag-backs clearing -------------------------------------------------
			// Once the recently-tagged player (gNoTagBack) has retreated far enough,
			// clear the restriction so IT can target them again.
			if(gNoTagBack >= 0) {
				// "third" = the non-IT player who is neither IT nor the no-tag-back target
				int third = -1;
				for(int i = 0; i < NUM_PLAYERS; i++) {
					if(i != gIT && i != gNoTagBack) { third = i; break; }
				}
				if(third >= 0 && dist2(gPlayers[gIT], gPlayers[gNoTagBack]) >= dist2(gPlayers[gIT], gPlayers[third]))
					gNoTagBack = -1;
			}

			// -- Find IT's eligible chase target (nearest non-IT, skip no-tag-back) ---
			int target = -1;
			float minDist = 1e9f;
			for(int i = 0; i < NUM_PLAYERS; i++) {
				if(i == gIT) continue;
				if(i == gNoTagBack) continue;
				float d = dist2(gPlayers[gIT], gPlayers[i]);
				if(d < minDist) { minDist = d; target = i; }
			}
			// Fallback: if both non-IT are excluded (shouldn't happen with 3 players and
			// only one restriction at a time), chase the nearest regardless.
			if(target == -1) {
				for(int i = 0; i < NUM_PLAYERS; i++) {
					if(i == gIT) continue;
					float d = dist2(gPlayers[gIT], gPlayers[i]);
					if(d < minDist) { minDist = d; target = i; }
				}
			}

			// -- Per-player physics update --------------------------------------------
			for(int i = 0; i < NUM_PLAYERS; i++) {
				float ax = 0.0f, ay = 0.0f;

				if(i == gIT && target >= 0) {
					float dx = gPlayers[target].x - gPlayers[i].x;
					float dy = gPlayers[target].y - gPlayers[i].y;
					float d  = sqrtf(dx*dx + dy*dy);
					if(d > 1e-4f) { ax = speed[i] * dx/d; ay = speed[i] * dy/d; }
				} else if(i != gIT) {
					// Flee directly away from IT's current position.
					float dx = gPlayers[i].x - gPlayers[gIT].x;
					float dy = gPlayers[i].y - gPlayers[gIT].y;
					float d  = sqrtf(dx*dx + dy*dy);
					if(d > 1e-4f) { ax = speed[i] * dx/d; ay = speed[i] * dy/d; }
				}

				// Viscous damping.
				ax -= damping * gPlayers[i].vx;
				ay -= damping * gPlayers[i].vy;

				// Soft-wall restoring force (spring past r_max).
				float r = sqrtf(gPlayers[i].x*gPlayers[i].x + gPlayers[i].y*gPlayers[i].y);
				if(r > kRMax && r > 1e-4f) {
					float wallF = kWallSpring * (r - kRMax);
					ax -= wallF * gPlayers[i].x / r;
					ay -= wallF * gPlayers[i].y / r;
				}

				gPlayers[i].vx += ax * dt;
				gPlayers[i].vy += ay * dt;
				gPlayers[i].x  += gPlayers[i].vx * dt;
				gPlayers[i].y  += gPlayers[i].vy * dt;
			}

			// -- Tag detection (post-physics positions) --------------------------------
			bool tagEventThisFrame = false;
			if(target >= 0) {
				float d = dist2(gPlayers[gIT], gPlayers[target]);
				if(d < kCatchRadius) {
					int oldIT = gIT;
					gIT = target;
					gNoTagBack = oldIT; // new IT can't immediately re-tag old IT
					tagEventThisFrame = true;
				}
			}

			// -- Wall gate: high while r >= r_max (position tracked freely beyond boundary,
			// r output is clamped to 1.0, gate is the raw level comparison) --------------
			bool wallHit[NUM_PLAYERS] = {false, false, false};
			for(int i = 0; i < NUM_PLAYERS; i++) {
				float r = sqrtf(gPlayers[i].x*gPlayers[i].x + gPlayers[i].y*gPlayers[i].y);
				wallHit[i] = (r >= kRMax);
			}
			bool anyWall = wallHit[0] || wallHit[1] || wallHit[2];

			// -- Write digital outs ---------------------------------------------------
			digitalWrite(context, frame, DIGITAL_IT1, gIT == 0 ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_IT2, gIT == 1 ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_IT3, gIT == 2 ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_TAG,     tagEventThisFrame ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_WALL1,   wallHit[0] ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_WALL2,   wallHit[1] ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_WALL3,   wallHit[2] ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_WALLANY, anyWall ? 1 : 0);
		}

		// -- Compute and write r/θ outputs (both frozen and live paths) ---------------
		const unsigned int audioOutR[NUM_PLAYERS] = {AUDIO_OUT_P1_R, AUDIO_OUT_P2_R, AUDIO_OUT_P3_R};
		const unsigned int audioOutT[NUM_PLAYERS] = {AUDIO_OUT_P1_T, AUDIO_OUT_P2_T, AUDIO_OUT_P3_T};
		for(int i = 0; i < NUM_PLAYERS; i++) {
			float r = sqrtf(gPlayers[i].x*gPlayers[i].x + gPlayers[i].y*gPlayers[i].y);
			float rNorm = r / kRMax; // 0..1 (may momentarily exceed 1 due to soft wall)
			if(rNorm > 1.0f) rNorm = 1.0f;
			float theta = atan2f(gPlayers[i].y, gPlayers[i].x);
			float thetaNorm = (theta + (float)M_PI) / (2.0f * (float)M_PI); // -π..+π → 0..1
			gROut[i]    = rNorm;
			gThetaOut[i] = thetaNorm;
			if(audioOutR[i] < context->audioOutChannels) audioWrite(context, frame, audioOutR[i], rNorm);
			if(audioOutT[i] < context->audioOutChannels) audioWrite(context, frame, audioOutT[i], thetaNorm);
		}

		// Write IT gates while frozen too (state is unchanged, but pins still need driving).
		if(freezePin) {
			digitalWrite(context, frame, DIGITAL_IT1, gIT == 0 ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_IT2, gIT == 1 ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_IT3, gIT == 2 ? 1 : 0);
			digitalWrite(context, frame, DIGITAL_TAG,     0);
			digitalWrite(context, frame, DIGITAL_WALL1,   0);
			digitalWrite(context, frame, DIGITAL_WALL2,   0);
			digitalWrite(context, frame, DIGITAL_WALL3,   0);
			digitalWrite(context, frame, DIGITAL_WALLANY, 0);
		}
	}
}

void cleanup(BelaContext *context, void *userData) {}
