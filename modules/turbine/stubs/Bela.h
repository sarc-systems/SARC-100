#pragma once

// Local IntelliSense fallback for machines without Bela headers installed.
// Real Bela builds use <Bela.h> on target and ignore this file.

#include <cstdint>

struct BelaContext {
	unsigned int audioFrames = 0;
	unsigned int audioSampleRate = 48000;
	unsigned int audioInChannels = 2;   // base AC-coupled stereo pair only, without wing-off header
	unsigned int audioOutChannels = 2;
	unsigned int digitalFrames = 0;
	unsigned int analogFrames = 0;
	unsigned int analogInChannels = 8;
	unsigned int analogOutChannels = 8;
	const char* projectName = "turbine";
};

#ifndef INPUT
#define INPUT 0
#define OUTPUT 1
#endif

inline float audioRead(BelaContext*, unsigned int, unsigned int) { return 0.0f; }
inline void audioWrite(BelaContext*, unsigned int, unsigned int, float) {}
inline int digitalRead(BelaContext*, unsigned int, unsigned int) { return 0; }
inline void digitalWrite(BelaContext*, unsigned int, unsigned int, int) {}
inline float analogRead(BelaContext*, unsigned int, unsigned int) { return 0.0f; }
inline void analogWrite(BelaContext*, unsigned int, unsigned int, float) {}
inline void pinMode(BelaContext*, unsigned int, unsigned int, int) {}
inline int Bela_stopRequested() { return 0; }
typedef void* AuxiliaryTask;
inline AuxiliaryTask Bela_createAuxiliaryTask(void (*)(void*), int, const char*, void*) { return nullptr; }
inline int Bela_scheduleAuxiliaryTask(AuxiliaryTask) { return 0; }
inline int rt_printf(const char*, ...) { return 0; }
