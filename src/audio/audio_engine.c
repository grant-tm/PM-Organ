#include <math.h>

#include "pm_organ/audio/audio_engine.h"
#include "pm_organ/core/assert.h"

static const f64 PI64 = 3.14159265358979323846;

bool AudioEngine_Initialize (AudioEngine *engine, MemoryArena *arena, const AudioEngineDesc *desc)
{
    ASSERT(engine != NULL);
    ASSERT(arena != NULL);
    ASSERT(desc != NULL);
    ASSERT(desc->sample_rate > 0);
    ASSERT(desc->channel_count > 0);
    ASSERT(desc->block_frame_count > 0);

    (void) arena;

    engine->config = *desc;
    engine->test_sine_phase = 0.0;

    return true;
}

void AudioEngine_Shutdown (AudioEngine *engine)
{
    ASSERT(engine != NULL);

    engine->config.sample_rate = 0;
    engine->config.channel_count = 0;
    engine->config.block_frame_count = 0;
    engine->test_sine_phase = 0.0;
}

void AudioEngine_RenderBlock (AudioEngine *engine, f32 *output)
{
    u32 frame_index;

    ASSERT(engine != NULL);
    ASSERT(output != NULL);
    ASSERT(engine->config.sample_rate > 0);
    ASSERT(engine->config.channel_count > 0);
    ASSERT(engine->config.block_frame_count > 0);

    for (frame_index = 0; frame_index < engine->config.block_frame_count; frame_index += 1)
    {
        f32 sample_value;
        u32 channel_index;

        sample_value = (f32) (0.05 * sin(engine->test_sine_phase));
        engine->test_sine_phase += (2.0 * PI64 * 220.0) / (f64) engine->config.sample_rate;

        if (engine->test_sine_phase >= 2.0 * PI64)
        {
            engine->test_sine_phase -= 2.0 * PI64;
        }

        for (channel_index = 0; channel_index < engine->config.channel_count; channel_index += 1)
        {
            output[frame_index * engine->config.channel_count + channel_index] = sample_value;
        }
    }
}
