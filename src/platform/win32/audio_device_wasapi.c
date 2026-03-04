#define COBJMACROS

#include <windows.h>
#include <audioclient.h>
#include <avrt.h>
#include <ksmedia.h>
#include <mmdeviceapi.h>

#include "pm_organ/audio/audio_device.h"
#include "pm_organ/core/assert.h"

typedef struct Win32AudioDeviceState
{
    IMMDeviceEnumerator *device_enumerator;
    IMMDevice *device;
    IAudioClient *audio_client;
    IAudioRenderClient *render_client;
    WAVEFORMATEX *mix_format;
    HANDLE audio_event;
    HANDLE stop_event;
    HANDLE thread_handle;
    AudioRenderCallback *render_callback;
    void *user_data;
    bool com_initialized;
} Win32AudioDeviceState;

static const GUID IEEE_FLOAT_SUBTYPE = 
{
	0x00000003, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71 } 
};
static const GUID CLSID_MMDEVICE_ENUMERATOR_LOCAL =
{
    0xbcde0395, 0xe52f, 0x467c, { 0x8e, 0x3d, 0xc4, 0x57, 0x92, 0x91, 0x69, 0x2e }
};
static const GUID IID_IMMDEVICE_ENUMERATOR_LOCAL =
{
    0xa95664d2, 0x9614, 0x4f35, { 0xa7, 0x46, 0xde, 0x8d, 0xb6, 0x36, 0x17, 0xe6 }
};
static const GUID IID_IAUDIO_CLIENT_LOCAL =
{
    0x1cb9ad4c, 0xdbfa, 0x4c32, { 0xb1, 0x78, 0xc2, 0xf5, 0x68, 0xa7, 0x03, 0xb2 }
};
static const GUID IID_IAUDIO_RENDER_CLIENT_LOCAL =
{
    0xf294acfc, 0x3146, 0x4483, { 0xa7, 0xbf, 0xad, 0xdc, 0xa7, 0xc2, 0x60, 0xe2 }
};

static void ReleaseComObject (IUnknown *unknown);
void Win32AudioDevice_Close (AudioDevice *device);

static void DestroyState (Win32AudioDeviceState *state)
{
    if (state == NULL)
    {
        return;
    }

    if (state->thread_handle != NULL)
    {
        CloseHandle(state->thread_handle);
        state->thread_handle = NULL;
    }

    if (state->audio_event != NULL)
    {
        CloseHandle(state->audio_event);
        state->audio_event = NULL;
    }

    if (state->stop_event != NULL)
    {
        CloseHandle(state->stop_event);
        state->stop_event = NULL;
    }

    if (state->mix_format != NULL)
    {
        CoTaskMemFree(state->mix_format);
        state->mix_format = NULL;
    }

    ReleaseComObject((IUnknown *) state->render_client);
    ReleaseComObject((IUnknown *) state->audio_client);
    ReleaseComObject((IUnknown *) state->device);
    ReleaseComObject((IUnknown *) state->device_enumerator);

    state->render_client = NULL;
    state->audio_client = NULL;
    state->device = NULL;
    state->device_enumerator = NULL;

    if (state->com_initialized)
    {
        CoUninitialize();
        state->com_initialized = false;
    }
}

static bool IsFloatMixFormat (const WAVEFORMATEX *format)
{
    if (format->wFormatTag == WAVE_FORMAT_IEEE_FLOAT)
    {
        return format->wBitsPerSample == 32;
    }

    if (format->wFormatTag == WAVE_FORMAT_EXTENSIBLE)
    {
        const WAVEFORMATEXTENSIBLE *extensible_format;

        extensible_format = (const WAVEFORMATEXTENSIBLE *) format;
        if (IsEqualGUID(&extensible_format->SubFormat, &IEEE_FLOAT_SUBTYPE) == 0)
        {
            return false;
        }

        return extensible_format->Format.wBitsPerSample == 32;
    }

    return false;
}

static void FillBuffer (Win32AudioDeviceState *state, AudioDevice *device, BYTE *buffer, u32 frame_count)
{
    f32 *output;

    ASSERT(state != NULL);
    ASSERT(device != NULL);
    ASSERT(buffer != NULL);

    output = (f32 *) buffer;

    if (state->render_callback != NULL)
    {
        state->render_callback(
            state->user_data,
            output,
            frame_count,
            device->info.channel_count,
            device->info.sample_rate
        );
        return;
    }

    ZeroMemory(output, (SIZE_T) frame_count * (SIZE_T) device->info.channel_count * sizeof(f32));
}

static DWORD WINAPI AudioRenderThreadProc (LPVOID parameter)
{
    AudioDevice *device;
    Win32AudioDeviceState *state;
    HANDLE wait_handles[2];
    HANDLE mmcss_handle;
    DWORD mmcss_task_index;
    HRESULT result;
    bool thread_com_initialized;

    device = (AudioDevice *) parameter;
    ASSERT(device != NULL);

    state = (Win32AudioDeviceState *) device->backend_state;
    ASSERT(state != NULL);

    thread_com_initialized = SUCCEEDED(CoInitializeEx(NULL, COINIT_MULTITHREADED));
    mmcss_task_index = 0;
    mmcss_handle = AvSetMmThreadCharacteristicsA("Pro Audio", &mmcss_task_index);

    wait_handles[0] = state->stop_event;
    wait_handles[1] = state->audio_event;

    while (true)
    {
        UINT32 available_frame_count;
        UINT32 padding_frame_count;
        BYTE *buffer;
        DWORD wait_result;

        wait_result = WaitForMultipleObjects(ARRAY_COUNT(wait_handles), wait_handles, FALSE, INFINITE);
        if (wait_result == WAIT_OBJECT_0)
        {
            break;
        }

        if (wait_result != WAIT_OBJECT_0 + 1)
        {
            continue;
        }

        result = IAudioClient_GetCurrentPadding(state->audio_client, &padding_frame_count);
        if (FAILED(result))
        {
            break;
        }

        available_frame_count = device->info.buffer_frame_count - padding_frame_count;
        if (available_frame_count == 0)
        {
            continue;
        }

        result = IAudioRenderClient_GetBuffer(state->render_client, available_frame_count, &buffer);
        if (FAILED(result))
        {
            break;
        }

        FillBuffer(state, device, buffer, available_frame_count);

        result = IAudioRenderClient_ReleaseBuffer(state->render_client, available_frame_count, 0);
        if (FAILED(result))
        {
            break;
        }
    }

    if (mmcss_handle != NULL)
    {
        AvRevertMmThreadCharacteristics(mmcss_handle);
    }

    if (thread_com_initialized)
    {
        CoUninitialize();
    }

    return 0;
}

static void ReleaseComObject (IUnknown *unknown)
{
    if (unknown != NULL)
    {
        IUnknown_Release(unknown);
    }
}

bool Win32AudioDevice_Open (AudioDevice *device, MemoryArena *arena, const AudioDeviceDesc *desc)
{
    Win32AudioDeviceState *state;
    HRESULT result;
    REFERENCE_TIME buffer_duration;
    UINT32 buffer_frame_count;
    UINT32 requested_frames;
    BYTE *buffer;

    ASSERT(device != NULL);
    ASSERT(arena != NULL);
    ASSERT(desc != NULL);
    ASSERT(desc->render_callback != NULL);

    ZeroMemory(device, sizeof(*device));

    state = MEMORY_ARENA_PUSH_STRUCT(arena, Win32AudioDeviceState);
    if (state == NULL)
    {
        return false;
    }

    ZeroMemory(state, sizeof(*state));

    result = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (FAILED(result))
    {
        return false;
    }

    state->com_initialized = true;

    result = CoCreateInstance(
        &CLSID_MMDEVICE_ENUMERATOR_LOCAL,
        NULL,
        CLSCTX_ALL,
        &IID_IMMDEVICE_ENUMERATOR_LOCAL,
        (void **) &state->device_enumerator
    );
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    result = IMMDeviceEnumerator_GetDefaultAudioEndpoint(state->device_enumerator, eRender, eConsole, &state->device);
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    result = IMMDevice_Activate(
        state->device,
        &IID_IAUDIO_CLIENT_LOCAL,
        CLSCTX_ALL,
        NULL,
        (void **) &state->audio_client
    );
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    result = IAudioClient_GetMixFormat(state->audio_client, &state->mix_format);
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    if (IsFloatMixFormat(state->mix_format) == false)
    {
        DestroyState(state);
        return false;
    }

    requested_frames = (desc->frames_per_buffer > 0) ? desc->frames_per_buffer : 256;
    buffer_duration = (REFERENCE_TIME) (((u64) requested_frames * 10000000ULL) / state->mix_format->nSamplesPerSec);

    result = IAudioClient_Initialize(
        state->audio_client,
        AUDCLNT_SHAREMODE_SHARED,
        AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
        buffer_duration,
        0,
        state->mix_format,
        NULL
    );
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    result = IAudioClient_GetBufferSize(state->audio_client, &buffer_frame_count);
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    result = IAudioClient_GetService(
        state->audio_client,
        &IID_IAUDIO_RENDER_CLIENT_LOCAL,
        (void **) &state->render_client
    );
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    state->audio_event = CreateEventA(NULL, FALSE, FALSE, NULL);
    state->stop_event = CreateEventA(NULL, TRUE, FALSE, NULL);
    if ((state->audio_event == NULL) || (state->stop_event == NULL))
    {
        DestroyState(state);
        return false;
    }

    result = IAudioClient_SetEventHandle(state->audio_client, state->audio_event);
    if (FAILED(result))
    {
        DestroyState(state);
        return false;
    }

    state->render_callback = desc->render_callback;
    state->user_data = desc->user_data;

    device->backend_state = state;
    device->info.sample_rate = state->mix_format->nSamplesPerSec;
    device->info.channel_count = state->mix_format->nChannels;
    device->info.frames_per_buffer = requested_frames;
    device->info.buffer_frame_count = buffer_frame_count;
    device->info.latency_seconds = (f64) buffer_frame_count / (f64) device->info.sample_rate;
    device->is_open = true;

    result = IAudioRenderClient_GetBuffer(state->render_client, buffer_frame_count, &buffer);
    if (FAILED(result))
    {
        Win32AudioDevice_Close(device);
        return false;
    }

    FillBuffer(state, device, buffer, buffer_frame_count);

    result = IAudioRenderClient_ReleaseBuffer(state->render_client, buffer_frame_count, 0);
    if (FAILED(result))
    {
        Win32AudioDevice_Close(device);
        return false;
    }

    return true;
}

bool Win32AudioDevice_Start (AudioDevice *device)
{
    Win32AudioDeviceState *state;
    HRESULT result;

    ASSERT(device != NULL);

    if ((device->is_open == false) || (device->is_running == true))
    {
        return false;
    }

    state = (Win32AudioDeviceState *) device->backend_state;
    ASSERT(state != NULL);

    device->is_running = true;
    ResetEvent(state->stop_event);

    state->thread_handle = CreateThread(NULL, 0, AudioRenderThreadProc, device, 0, NULL);
    if (state->thread_handle == NULL)
    {
        device->is_running = false;
        return false;
    }

    result = IAudioClient_Start(state->audio_client);
    if (FAILED(result))
    {
        device->is_running = false;
        SetEvent(state->stop_event);
        WaitForSingleObject(state->thread_handle, INFINITE);
        CloseHandle(state->thread_handle);
        state->thread_handle = NULL;
        return false;
    }
    return true;
}

void Win32AudioDevice_Stop (AudioDevice *device)
{
    Win32AudioDeviceState *state;

    ASSERT(device != NULL);

    if ((device->is_open == false) || (device->is_running == false))
    {
        return;
    }

    state = (Win32AudioDeviceState *) device->backend_state;
    ASSERT(state != NULL);

    device->is_running = false;

    IAudioClient_Stop(state->audio_client);
    SetEvent(state->stop_event);

    if (state->thread_handle != NULL)
    {
        WaitForSingleObject(state->thread_handle, INFINITE);
        CloseHandle(state->thread_handle);
        state->thread_handle = NULL;
    }
}

void Win32AudioDevice_Close (AudioDevice *device)
{
    Win32AudioDeviceState *state;

    ASSERT(device != NULL);

    if (device->is_open == false)
    {
        ZeroMemory(device, sizeof(*device));
        return;
    }

    Win32AudioDevice_Stop(device);

    state = (Win32AudioDeviceState *) device->backend_state;
    if (state != NULL)
    {
        DestroyState(state);
    }

    ZeroMemory(device, sizeof(*device));
}
