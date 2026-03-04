#ifndef PM_ORGAN_AUDIO_TEST_TONE_SOURCE_H
#define PM_ORGAN_AUDIO_TEST_TONE_SOURCE_H

#include "pm_organ/core/types.h"

typedef struct TestToneSourceDesc
{
    f64 frequency_hz;
    f32 amplitude;
} TestToneSourceDesc;

typedef struct TestToneSource
{
    f64 frequency_hz;
    f64 phase;
    f32 amplitude;
} TestToneSource;

bool TestToneSource_Initialize (TestToneSource *source, const TestToneSourceDesc *desc);
void TestToneSource_Render (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
);

#endif // PM_ORGAN_AUDIO_TEST_TONE_SOURCE_H
