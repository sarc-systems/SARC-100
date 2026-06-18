#pragma once

// Bela jack/channel assignments for SARC Empath.
// Single source of truth for I/O mapping — keep in sync with the
// "I/O mapping (current)" table in CLAUDE.md.

// Audio
#define AUDIO_IN_A   0   // All-in A
#define AUDIO_IN_B   1   // All-in B
#define AUDIO_OUT_A  0   // Scan out A
#define AUDIO_OUT_B  1   // Scan out B

// Analog in (CV)
#define ANALOG_SCAN_A_CV          0   // Output scan A CV
#define ANALOG_SCAN_B_CV          1   // Output scan B CV
#define ANALOG_TUNE_CV            2   // F0 tune CV
#define ANALOG_DETUNE_CV          3   // Detune CV
#define ANALOG_NODE_COUPLING_A_CV 4   // Node Coupling A CV
#define ANALOG_NODE_COUPLING_B_CV 5   // Node Coupling B CV
#define ANALOG_XCOUPLE_AMOUNT_CV   6   // Cross-couple amount CV (total budget)
#define ANALOG_XCOUPLE_SYMMETRY_CV 7   // Cross-couple symmetry CV (0=A->B .. 1=B->A)

// Digital
#define SYNC_DIGITAL 0   // SYNC in — rising edge resets both ladders
