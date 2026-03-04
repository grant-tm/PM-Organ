#include <windows.h>

#include "pm_organ/app/app.h"
#include "pm_organ/audio/audio_device.h"
#include "pm_organ/audio/audio_engine.h"
#include "pm_organ/audio/test_tone_source.h"
#include "pm_organ/core/assert.h"
#include "pm_organ/core/frame_timer.h"
#include "pm_organ/core/memory_arena.h"
#include "pm_organ/platform/time.h"
#include "pm_organ/sim/fdtd_1d_render_source.h"
#include "pm_organ/platform/window.h"

typedef struct AppState
{
    AudioDevice audio_device;
    AudioEngine audio_engine;
    Fdtd1DRenderSource fdtd_1d_render_source;
    FrameTimer frame_timer;
    Window main_window;
    TestToneSource test_tone_source;
    bool fdtd_source_is_active;
    bool previous_space_is_down;
    bool previous_toggle_is_down;
} AppState;

static void SetActiveRenderSource (AppState *app, bool use_fdtd_source)
{
    ASSERT(app != NULL);

    if (use_fdtd_source)
    {
        AudioEngine_SetRenderSource(
            &app->audio_engine,
            Fdtd1DRenderSource_Render,
            &app->fdtd_1d_render_source
        );
    }
    else
    {
        AudioEngine_SetRenderSource(&app->audio_engine, TestToneSource_Render, &app->test_tone_source);
    }

    app->fdtd_source_is_active = use_fdtd_source;
}

static void UpdateDebugInput (AppState *app)
{
    bool space_is_down;
    bool toggle_is_down;

    ASSERT(app != NULL);

    space_is_down = (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;
    toggle_is_down = (GetAsyncKeyState('T') & 0x8000) != 0;

    if (space_is_down && (app->previous_space_is_down == false))
    {
        Fdtd1DRenderSource_TriggerStartupImpulse(&app->fdtd_1d_render_source);
    }

    if (toggle_is_down && (app->previous_toggle_is_down == false))
    {
        SetActiveRenderSource(app, app->fdtd_source_is_active == false);
    }

    app->previous_space_is_down = space_is_down;
    app->previous_toggle_is_down = toggle_is_down;
}

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
    Fdtd1DProbeDesc probe_descs[2];
    Fdtd1DRenderSourceDesc fdtd_render_source_desc;
    Fdtd1DSourceDesc source_descs[1];
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

    probe_descs[0].cell_index = 96;
    probe_descs[0].output_channel_index = 0;
    probe_descs[0].is_enabled = true;

    probe_descs[1].cell_index = 112;
    probe_descs[1].output_channel_index = 1;
    probe_descs[1].is_enabled = true;

    source_descs[0].cell_index = 16;
    source_descs[0].is_enabled = true;

    fdtd_render_source_desc.solver_desc.sample_rate = engine_desc.sample_rate;
    fdtd_render_source_desc.solver_desc.block_frame_count = engine_desc.block_frame_count;
    fdtd_render_source_desc.solver_desc.output_channel_count = engine_desc.channel_count;
    fdtd_render_source_desc.solver_desc.tube_length_m = 0.914666667;
    fdtd_render_source_desc.solver_desc.wave_speed_m_per_s = 343.0;
    fdtd_render_source_desc.solver_desc.density_kg_per_m3 = 1.225;
    fdtd_render_source_desc.solver_desc.dx = 343.0 / 48000.0;
    fdtd_render_source_desc.solver_desc.pressure_cell_count = 128;
    fdtd_render_source_desc.solver_desc.velocity_cell_count = 129;
    fdtd_render_source_desc.solver_desc.courant_number = 1.0;
    fdtd_render_source_desc.solver_desc.uniform_area_m2 = 0.01;
    fdtd_render_source_desc.solver_desc.uniform_loss = 0.0;
    fdtd_render_source_desc.solver_desc.left_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
    fdtd_render_source_desc.solver_desc.left_boundary.reflection_coefficient = 1.0;
    fdtd_render_source_desc.solver_desc.right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
    fdtd_render_source_desc.solver_desc.right_boundary.reflection_coefficient = 1.0;
    fdtd_render_source_desc.solver_desc.probe_count = ARRAY_COUNT(probe_descs);
    fdtd_render_source_desc.solver_desc.probe_descs = probe_descs;
    fdtd_render_source_desc.solver_desc.source_count = ARRAY_COUNT(source_descs);
    fdtd_render_source_desc.solver_desc.source_descs = source_descs;
    fdtd_render_source_desc.startup_impulse_is_enabled = true;
    fdtd_render_source_desc.startup_impulse_target_index = 0;
    fdtd_render_source_desc.startup_impulse_amplitude = 0.25;

    if (Fdtd1DRenderSource_Initialize(&app->fdtd_1d_render_source, &bootstrap_arena, &fdtd_render_source_desc) == false)
    {
        AudioEngine_Shutdown(&app->audio_engine);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    test_tone_desc.frequency_hz = 220.0;
    test_tone_desc.amplitude = 0.05f;

    if (TestToneSource_Initialize(&app->test_tone_source, &test_tone_desc) == false)
    {
        Fdtd1DRenderSource_Shutdown(&app->fdtd_1d_render_source);
        AudioEngine_Shutdown(&app->audio_engine);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    app->previous_space_is_down = false;
    app->previous_toggle_is_down = false;
    SetActiveRenderSource(app, true);

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
        UpdateDebugInput(app);
        FrameTimer_EndFrame(&app->frame_timer);
    }

    AudioDevice_Close(&app->audio_device);
    Fdtd1DRenderSource_Shutdown(&app->fdtd_1d_render_source);
    AudioEngine_Shutdown(&app->audio_engine);
    PlatformWindow_Destroy(&app->main_window);
    MemoryArena_Destroy(&bootstrap_arena);

    return 0;
}
