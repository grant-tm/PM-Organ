#ifndef PM_ORGAN_AUDIO_AUDIO_DEVICE_H
#define PM_ORGAN_AUDIO_AUDIO_DEVICE_H

#include "pm_organ/core/memory_arena.h"
#include "pm_organ/core/types.h"

typedef void AudioRenderCallback (void *user_data, f32 *output, u32 frame_count, u32 channel_count, u32 sample_rate);

typedef struct AudioDeviceDesc
{
    u32 sample_rate;
    u32 channel_count;
    u32 frames_per_buffer;
    AudioRenderCallback *render_callback;
    void *user_data;
} AudioDeviceDesc;

typedef struct AudioDeviceInfo
{
    u32 sample_rate;
    u32 channel_count;
    u32 frames_per_buffer;
    u32 buffer_frame_count;
    f64 latency_seconds;
} AudioDeviceInfo;

typedef struct AudioDevice
{
    void *backend_state;
    AudioDeviceInfo info;
    bool is_open;
    bool is_running;
} AudioDevice;

bool AudioDevice_Open (AudioDevice *device, MemoryArena *arena, const AudioDeviceDesc *desc);
bool AudioDevice_Start (AudioDevice *device);
void AudioDevice_Stop (AudioDevice *device);
void AudioDevice_Close (AudioDevice *device);

#endif // PM_ORGAN_AUDIO_AUDIO_DEVICE_H
