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

typedef void AudioEngineRenderSourceCallback (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
);

typedef struct AudioEngine
{
    AudioEngineDesc config;
    AudioEngineRenderSourceCallback *render_source_callback;
    void *render_source_user_data;
    f32 *mix_buffer;
    f32 *scratch_buffer;
} AudioEngine;

bool AudioEngine_Initialize (AudioEngine *engine, MemoryArena *arena, const AudioEngineDesc *desc);
void AudioEngine_Shutdown (AudioEngine *engine);
void AudioEngine_RenderBlock (AudioEngine *engine, f32 *output);
void AudioEngine_SetRenderSource (
    AudioEngine *engine,
    AudioEngineRenderSourceCallback *render_callback,
    void *user_data
);

#endif // PM_ORGAN_AUDIO_AUDIO_ENGINE_H
