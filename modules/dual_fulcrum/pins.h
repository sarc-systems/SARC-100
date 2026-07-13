#pragma once

// Bela jack/channel assignments for DUAL FULCRUM — two independent FULCRUM instances.
// 8 analog in (4 per instance), 4 DC-coupled audio outs (2 per instance), 4 digital in.
//
// All attenuverters / bias / offset pots are ANALOG front-end conditioning — the ADC
// already sees the post-pot signal, so code reads ONE value per input (no separate pot ADC).

// ── Analog in — FULCRUM 1 ────────────────────────────────────────────────────
#define AN_F1_A      4   // A input (audio/CV, bipolar)
#define AN_F1_B      5   // B input (audio/CV, bipolar)
#define AN_F1_THETA  0   // rotation/crossfade CV -> [-pi/2, pi/2]
#define AN_F1_R      1   // magnitude CV (bipolar; negative inverts)
// ── Analog in — FULCRUM 2 ────────────────────────────────────────────────────
#define AN_F2_A      6
#define AN_F2_B      72
#define AN_F2_THETA  2
#define AN_F2_R      3

// ── Outputs — DC-coupled AUDIO outs on the Gem Multi (audioWrite, NOT analogWrite;
//    analogWrite would index a nonexistent analog-out buffer — see DRIVETRAIN notes).
#define AUDIO_F1_OUT   2   // r·(a·cosθ + b·sinθ)
#define AUDIO_F1_QUAD  3   // r·(-a·sinθ + b·cosθ)  — orthogonal companion
#define AUDIO_F2_OUT   4
#define AUDIO_F2_QUAD  5
// audio outs 0,1 and 6,7 unused on this module

// ── Digital in — gates ───────────────────────────────────────────────────────
#define DIG_F1_INVERT   0   // high = flip output polarity (negate r_eff)
#define DIG_F1_REVERSE  1   // high = negate theta (swap which input is favored)
#define DIG_F2_INVERT   2
#define DIG_F2_REVERSE  3
