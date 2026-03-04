#include <string.h>

#include "pm_organ/audio/audio_engine.h"
#include "pm_organ/core/assert.h"

static void RenderSilenceSource (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    usize output_byte_count;

    ASSERT(output != NULL);
    ASSERT(scratch_buffer != NULL);
    ASSERT(block_frame_count > 0);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    (void) user_data;

    output_byte_count = sizeof(f32) * (usize) block_frame_count * (usize) channel_count;
    memset(output, 0, output_byte_count);
    memset(scratch_buffer, 0, output_byte_count);
}

bool AudioEngine_Initialize (AudioEngine *engine, MemoryArena *arena, const AudioEngineDesc *desc)
{
    usize mix_sample_count;

    ASSERT(engine != NULL);
    ASSERT(arena != NULL);
    ASSERT(desc != NULL);
    ASSERT(desc->sample_rate > 0);
    ASSERT(desc->channel_count > 0);
    ASSERT(desc->block_frame_count > 0);

    engine->config = *desc;
    mix_sample_count = (usize) desc->block_frame_count * (usize) desc->channel_count;

    engine->mix_buffer = MEMORY_ARENA_PUSH_ARRAY(arena, mix_sample_count, f32);
    engine->scratch_buffer = MEMORY_ARENA_PUSH_ARRAY(
        arena,
        (usize) desc->block_frame_count * (usize) desc->channel_count,
        f32
    );
    if ((engine->mix_buffer == NULL) || (engine->scratch_buffer == NULL))
    {
        return false;
    }

    engine->render_source_callback = RenderSilenceSource;
    engine->render_source_user_data = NULL;

    return true;
}

void AudioEngine_Shutdown (AudioEngine *engine)
{
    ASSERT(engine != NULL);

    engine->config.sample_rate = 0;
    engine->config.channel_count = 0;
    engine->config.block_frame_count = 0;
    engine->render_source_callback = NULL;
    engine->render_source_user_data = NULL;
    engine->mix_buffer = NULL;
    engine->scratch_buffer = NULL;
}

void AudioEngine_RenderBlock (AudioEngine *engine, f32 *output)
{
    usize mix_byte_count;
    usize scratch_byte_count;

    ASSERT(engine != NULL);
    ASSERT(output != NULL);
    ASSERT(engine->config.sample_rate > 0);
    ASSERT(engine->config.channel_count > 0);
    ASSERT(engine->config.block_frame_count > 0);
    ASSERT(engine->mix_buffer != NULL);
    ASSERT(engine->scratch_buffer != NULL);
    ASSERT(engine->render_source_callback != NULL);

    mix_byte_count = sizeof(f32) * (usize) engine->config.block_frame_count * (usize) engine->config.channel_count;
    scratch_byte_count = sizeof(f32) * (usize) engine->config.block_frame_count * (usize) engine->config.channel_count;

    memset(engine->mix_buffer, 0, mix_byte_count);
    memset(engine->scratch_buffer, 0, scratch_byte_count);

    engine->render_source_callback(
        engine->render_source_user_data,
        engine->mix_buffer,
        engine->scratch_buffer,
        engine->config.block_frame_count,
        engine->config.channel_count,
        engine->config.sample_rate
    );

    memcpy(output, engine->mix_buffer, mix_byte_count);
}

void AudioEngine_SetRenderSource (
    AudioEngine *engine,
    AudioEngineRenderSourceCallback *render_callback,
    void *user_data
)
{
    ASSERT(engine != NULL);
    ASSERT(render_callback != NULL);

    engine->render_source_callback = render_callback;
    engine->render_source_user_data = user_data;
}
