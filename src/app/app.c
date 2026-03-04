#include "pm_organ/app/app.h"
#include "pm_organ/audio/audio_device.h"
#include "pm_organ/audio/audio_engine.h"
#include "pm_organ/audio/test_tone_source.h"
#include "pm_organ/core/assert.h"
#include "pm_organ/core/frame_timer.h"
#include "pm_organ/core/memory_arena.h"
#include "pm_organ/platform/time.h"
#include "pm_organ/platform/window.h"

typedef struct AppState
{
    AudioDevice audio_device;
    AudioEngine audio_engine;
    FrameTimer frame_timer;
    Window main_window;
    TestToneSource test_tone_source;
} AppState;

static void RenderEngineBlock (
    void *user_data,
    f32 *output,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    AppState *app;

    ASSERT(user_data != NULL);
    ASSERT(output != NULL);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    app = (AppState *) user_data;
    ASSERT(app->audio_engine.config.block_frame_count == block_frame_count);
    ASSERT(app->audio_engine.config.channel_count == channel_count);
    ASSERT(app->audio_engine.config.sample_rate == sample_rate);

    AudioEngine_RenderBlock(&app->audio_engine, output);
}

int App_Run (void)
{
    AppState *app;
    AudioDeviceDesc audio_desc;
    AudioEngineDesc engine_desc;
    TestToneSourceDesc test_tone_desc;
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

    window_desc.title = "PM-Organ";
    window_desc.client_width = 1280;
    window_desc.client_height = 720;

    if (PlatformWindow_Create(&app->main_window, &window_desc) == false)
    {
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    engine_desc.sample_rate = 48000;
    engine_desc.channel_count = 2;
    engine_desc.block_frame_count = 64;

    if (AudioEngine_Initialize(&app->audio_engine, &bootstrap_arena, &engine_desc) == false)
    {
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    test_tone_desc.frequency_hz = 220.0;
    test_tone_desc.amplitude = 0.05f;

    if (TestToneSource_Initialize(&app->test_tone_source, &test_tone_desc) == false)
    {
        AudioEngine_Shutdown(&app->audio_engine);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    AudioEngine_SetRenderSource(&app->audio_engine, TestToneSource_Render, &app->test_tone_source);

    audio_desc.sample_rate = 48000;
    audio_desc.channel_count = 2;
    audio_desc.frames_per_buffer = 256;
    audio_desc.block_frame_count = app->audio_engine.config.block_frame_count;
    audio_desc.render_callback = RenderEngineBlock;
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
    AudioEngine_Shutdown(&app->audio_engine);
    PlatformWindow_Destroy(&app->main_window);
    MemoryArena_Destroy(&bootstrap_arena);

    return 0;
}
