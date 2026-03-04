#include <math.h>
#include <string.h>

#include "pm_organ/audio/test_tone_source.h"
#include "pm_organ/core/assert.h"

static const f64 PI64 = 3.14159265358979323846;

bool TestToneSource_Initialize (TestToneSource *source, const TestToneSourceDesc *desc)
{
    ASSERT(source != NULL);
    ASSERT(desc != NULL);
    ASSERT(desc->frequency_hz > 0.0);
    ASSERT(desc->amplitude >= 0.0f);

    source->frequency_hz = desc->frequency_hz;
    source->phase = 0.0;
    source->amplitude = desc->amplitude;

    return true;
}

void TestToneSource_Render (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    TestToneSource *source;
    usize output_byte_count;
    u32 frame_index;

    ASSERT(user_data != NULL);
    ASSERT(output != NULL);
    ASSERT(scratch_buffer != NULL);
    ASSERT(block_frame_count > 0);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    source = (TestToneSource *) user_data;
    output_byte_count = sizeof(f32) * (usize) block_frame_count * (usize) channel_count;

    memset(output, 0, output_byte_count);
    memset(scratch_buffer, 0, output_byte_count);

    for (frame_index = 0; frame_index < block_frame_count; frame_index += 1)
    {
        f32 sample_value;
        u32 channel_index;

        sample_value = source->amplitude * (f32) sin(source->phase);
        source->phase += (2.0 * PI64 * source->frequency_hz) / (f64) sample_rate;

        if (source->phase >= 2.0 * PI64)
        {
            source->phase -= 2.0 * PI64;
        }

        for (channel_index = 0; channel_index < channel_count; channel_index += 1)
        {
            output[frame_index * channel_count + channel_index] = sample_value;
        }
    }
}
