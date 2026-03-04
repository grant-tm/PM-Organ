#include <math.h>

#include "pm_organ/app/app.h"
#include "pm_organ/audio/audio_device.h"
#include "pm_organ/core/assert.h"
#include "pm_organ/core/frame_timer.h"
#include "pm_organ/core/memory_arena.h"
#include "pm_organ/platform/time.h"
#include "pm_organ/platform/window.h"

static const f64 PI64 = 3.14159265358979323846;

typedef struct AppState
{
    AudioDevice audio_device;
    FrameTimer frame_timer;
    Window main_window;
    f64 sine_phase;
} AppState;

static void RenderSineWave (
    void *user_data,
    f32 *output,
    u32 frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    AppState *app;
    u32 frame_index;

    ASSERT(user_data != NULL);
    ASSERT(output != NULL);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    app = (AppState *) user_data;

    for (frame_index = 0; frame_index < frame_count; frame_index += 1)
    {
        f32 sample_value;
        u32 channel_index;

        sample_value = (f32) (0.05 * sin(app->sine_phase));
        app->sine_phase += (2.0 * PI64 * 220.0) / (f64) sample_rate;

        if (app->sine_phase >= 2.0 * PI64)
        {
            app->sine_phase -= 2.0 * PI64;
        }

        for (channel_index = 0; channel_index < channel_count; channel_index += 1)
        {
            output[frame_index * channel_count + channel_index] = sample_value;
        }
    }
}

int App_Run (void)
{
    AppState *app;
    AudioDeviceDesc audio_desc;
    MemoryArena bootstrap_arena;
    WindowDesc window_desc;

    if (MemoryArena_Create(&bootstrap_arena, 1024 * 1024) == false)
    {
        return 1;
    }

    if (PlatformTime_Initialize() == false)
    {
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    app = MEMORY_ARENA_PUSH_STRUCT(&bootstrap_arena, AppState);
    ASSERT(app != NULL);

    FrameTimer_Initialize(&app->frame_timer);
    FrameTimer_SetTargetHz(&app->frame_timer, 60.0);
    app->sine_phase = 0.0;

    window_desc.title = "PM-Organ";
    window_desc.client_width = 1280;
    window_desc.client_height = 720;

    if (PlatformWindow_Create(&app->main_window, &window_desc) == false)
    {
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    audio_desc.sample_rate = 48000;
    audio_desc.channel_count = 2;
    audio_desc.frames_per_buffer = 256;
    audio_desc.render_callback = RenderSineWave;
    audio_desc.user_data = app;

    if (AudioDevice_Open(&app->audio_device, &bootstrap_arena, &audio_desc) == false)
    {
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    if (AudioDevice_Start(&app->audio_device) == false)
    {
        AudioDevice_Close(&app->audio_device);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    while (app->main_window.is_running)
    {
        FrameTimer_BeginFrame(&app->frame_timer);
        PlatformWindow_PumpMessages(&app->main_window);
        FrameTimer_EndFrame(&app->frame_timer);
    }

    AudioDevice_Close(&app->audio_device);
    PlatformWindow_Destroy(&app->main_window);
    MemoryArena_Destroy(&bootstrap_arena);

    return 0;
}
