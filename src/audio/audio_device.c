#include "pm_organ/audio/audio_device.h"
#include "pm_organ/core/assert.h"

bool Win32AudioDevice_Open (AudioDevice *device, MemoryArena *arena, const AudioDeviceDesc *desc);
bool Win32AudioDevice_Start (AudioDevice *device);
void Win32AudioDevice_Stop (AudioDevice *device);
void Win32AudioDevice_Close (AudioDevice *device);

bool AudioDevice_Open (AudioDevice *device, MemoryArena *arena, const AudioDeviceDesc *desc)
{
    ASSERT(device != NULL);
    ASSERT(arena != NULL);
    ASSERT(desc != NULL);

    return Win32AudioDevice_Open(device, arena, desc);
}

bool AudioDevice_Start (AudioDevice *device)
{
    ASSERT(device != NULL);

    return Win32AudioDevice_Start(device);
}

void AudioDevice_Stop (AudioDevice *device)
{
    ASSERT(device != NULL);

    Win32AudioDevice_Stop(device);
}

void AudioDevice_Close (AudioDevice *device)
{
    ASSERT(device != NULL);

    Win32AudioDevice_Close(device);
}
