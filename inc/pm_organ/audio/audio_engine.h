#ifndef PM_ORGAN_AUDIO_AUDIO_ENGINE_H
#define PM_ORGAN_AUDIO_AUDIO_ENGINE_H

#include "pm_organ/core/memory_arena.h"
#include "pm_organ/core/types.h"

typedef struct AudioEngineDesc
{
    u32 sample_rate;
    u32 channel_count;
    u32 block_frame_count;
} AudioEngineDesc;

typedef struct AudioEngine
{
    AudioEngineDesc config;
    f64 test_sine_phase;
} AudioEngine;

bool AudioEngine_Initialize (AudioEngine *engine, MemoryArena *arena, const AudioEngineDesc *desc);
void AudioEngine_Shutdown (AudioEngine *engine);
void AudioEngine_RenderBlock (AudioEngine *engine, f32 *output);

#endif // PM_ORGAN_AUDIO_AUDIO_ENGINE_H
