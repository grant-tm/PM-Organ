#include <stdio.h>
#include <string.h>
#include <math.h>

#include <windows.h>

#include "pm_organ/app/app.h"
#include "pm_organ/audio/audio_device.h"
#include "pm_organ/audio/audio_engine.h"
#include "pm_organ/audio/test_tone_source.h"
#include "pm_organ/core/assert.h"
#include "pm_organ/core/frame_timer.h"
#include "pm_organ/core/memory_arena.h"
#include "pm_organ/debug/debug_gui.h"
#include "pm_organ/platform/time.h"
#include "pm_organ/sim/fdtd_1d_render_source.h"
#include "pm_organ/platform/window.h"

typedef enum FdtdPresetType
{
    FDTD_PRESET_TYPE_UNIFORM_STOPPED = 0,
    FDTD_PRESET_TYPE_NARROW_MOUTH_STOPPED,
    FDTD_PRESET_TYPE_WIDE_MOUTH_STOPPED,
    FDTD_PRESET_TYPE_OPEN_PIPE,
    FDTD_PRESET_TYPE_COUNT,
} FdtdPresetType;

typedef enum AppOutputExtractionMode
{
    APP_OUTPUT_EXTRACTION_MODE_RAW_PROBES = 0,
    APP_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION,
    APP_OUTPUT_EXTRACTION_MODE_LISTENER_MODEL,
    APP_OUTPUT_EXTRACTION_MODE_COUNT,
} AppOutputExtractionMode;

typedef enum AppExcitationMode
{
    APP_EXCITATION_MODE_IMPULSE = 0,
    APP_EXCITATION_MODE_CONSTANT,
    APP_EXCITATION_MODE_NOISE,
    APP_EXCITATION_MODE_BIAS_AND_NOISE,
    APP_EXCITATION_MODE_FEEDBACK_MOUTH,
    APP_EXCITATION_MODE_NONLINEAR_MOUTH,
    APP_EXCITATION_MODE_JET_LABIUM,
    APP_EXCITATION_MODE_COUNT,
} AppExcitationMode;

typedef enum AppSourceCouplingMode
{
    APP_SOURCE_COUPLING_MODE_PRESSURE = 0,
    APP_SOURCE_COUPLING_MODE_VELOCITY,
    APP_SOURCE_COUPLING_MODE_COUNT,
} AppSourceCouplingMode;

typedef struct AppState
{
    AudioDevice audio_device;
    AudioEngine audio_engine;
    DebugGui debug_gui;
    Fdtd1DRenderSource fdtd_render_sources[FDTD_PRESET_TYPE_COUNT];
    Fdtd1DRenderSource rank_render_sources[8];
    FrameTimer frame_timer;
    Window main_window;
    TestToneSource test_tone_source;
    FdtdPresetType active_fdtd_preset;
    AppExcitationMode active_excitation_mode;
    AppSourceCouplingMode active_source_coupling_mode;
    AppOutputExtractionMode active_output_extraction_mode;
    f32 drive_amplitude;
    f32 windchest_pressure;
    f32 speech_attack_seconds;
    f32 speech_chiff_amount;
    f32 speech_chiff_decay_seconds;
    f32 listener_distance_m;
    f32 listener_mouth_pressure_mix;
    f32 listener_crossfeed;
    f32 listener_lowpass_cutoff_hz;
    f32 master_gain;
    bool output_is_muted;
    bool fdtd_source_is_active;
    bool previous_space_is_down;
    bool previous_toggle_is_down;
    bool previous_cycle_is_down;
    bool rank_note_is_down[8];
    bool previous_rank_note_is_down[8];
} AppState;

enum
{
    RANK_NOTE_COUNT = 8,
};

static const u32 RANK_NOTE_SEMITONE_OFFSETS[RANK_NOTE_COUNT] =
{
    0, 2, 4, 5, 7, 9, 11, 12,
};

static Fdtd1DOutputExtractionMode ToRenderSourceOutputExtractionMode (AppOutputExtractionMode app_mode)
{
    switch (app_mode)
    {
        case APP_OUTPUT_EXTRACTION_MODE_RAW_PROBES: return FDTD_1D_OUTPUT_EXTRACTION_MODE_RAW_PROBES;
        case APP_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION: return FDTD_1D_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION;
        case APP_OUTPUT_EXTRACTION_MODE_LISTENER_MODEL: return FDTD_1D_OUTPUT_EXTRACTION_MODE_LISTENER_MODEL;
        case APP_OUTPUT_EXTRACTION_MODE_COUNT: break;
    }

    ASSERT(false);
    return FDTD_1D_OUTPUT_EXTRACTION_MODE_RAW_PROBES;
}

static Fdtd1DExcitationMode ToRenderSourceExcitationMode (AppExcitationMode app_mode)
{
    switch (app_mode)
    {
        case APP_EXCITATION_MODE_IMPULSE: return FDTD_1D_EXCITATION_MODE_IMPULSE;
        case APP_EXCITATION_MODE_CONSTANT: return FDTD_1D_EXCITATION_MODE_CONSTANT;
        case APP_EXCITATION_MODE_NOISE: return FDTD_1D_EXCITATION_MODE_NOISE;
        case APP_EXCITATION_MODE_BIAS_AND_NOISE: return FDTD_1D_EXCITATION_MODE_BIAS_AND_NOISE;
        case APP_EXCITATION_MODE_FEEDBACK_MOUTH: return FDTD_1D_EXCITATION_MODE_FEEDBACK_MOUTH;
        case APP_EXCITATION_MODE_NONLINEAR_MOUTH: return FDTD_1D_EXCITATION_MODE_NONLINEAR_MOUTH;
        case APP_EXCITATION_MODE_JET_LABIUM: return FDTD_1D_EXCITATION_MODE_JET_LABIUM;
        case APP_EXCITATION_MODE_COUNT: break;
    }

    ASSERT(false);
    return FDTD_1D_EXCITATION_MODE_IMPULSE;
}

static Fdtd1DSourceCouplingMode ToRenderSourceSourceCouplingMode (AppSourceCouplingMode app_mode)
{
    switch (app_mode)
    {
        case APP_SOURCE_COUPLING_MODE_PRESSURE: return FDTD_1D_SOURCE_COUPLING_MODE_PRESSURE;
        case APP_SOURCE_COUPLING_MODE_VELOCITY: return FDTD_1D_SOURCE_COUPLING_MODE_VELOCITY;
        case APP_SOURCE_COUPLING_MODE_COUNT: break;
    }

    ASSERT(false);
    return FDTD_1D_SOURCE_COUPLING_MODE_PRESSURE;
}

static const char *GetFdtdPresetName (FdtdPresetType preset_type)
{
    switch (preset_type)
    {
        case FDTD_PRESET_TYPE_UNIFORM_STOPPED: return "Uniform Stopped";
        case FDTD_PRESET_TYPE_NARROW_MOUTH_STOPPED: return "Narrow Mouth";
        case FDTD_PRESET_TYPE_WIDE_MOUTH_STOPPED: return "Wide Mouth";
        case FDTD_PRESET_TYPE_OPEN_PIPE: return "Open Pipe";
        case FDTD_PRESET_TYPE_COUNT: break;
    }

    ASSERT(false);
    return "Unknown";
}

static const char *GetExcitationModeName (AppExcitationMode excitation_mode)
{
    switch (excitation_mode)
    {
        case APP_EXCITATION_MODE_IMPULSE: return "Impulse";
        case APP_EXCITATION_MODE_CONSTANT: return "Constant";
        case APP_EXCITATION_MODE_NOISE: return "Noise";
        case APP_EXCITATION_MODE_BIAS_AND_NOISE: return "Bias + Noise";
        case APP_EXCITATION_MODE_FEEDBACK_MOUTH: return "Feedback Mouth";
        case APP_EXCITATION_MODE_NONLINEAR_MOUTH: return "Nonlinear Mouth";
        case APP_EXCITATION_MODE_JET_LABIUM: return "Jet Labium";
        case APP_EXCITATION_MODE_COUNT: break;
    }

    ASSERT(false);
    return "Unknown";
}

static const char *GetSourceCouplingModeName (AppSourceCouplingMode source_coupling_mode)
{
    switch (source_coupling_mode)
    {
        case APP_SOURCE_COUPLING_MODE_PRESSURE: return "Pressure";
        case APP_SOURCE_COUPLING_MODE_VELOCITY: return "Velocity";
        case APP_SOURCE_COUPLING_MODE_COUNT: break;
    }

    ASSERT(false);
    return "Unknown";
}

static const char *GetOutputExtractionModeName (AppOutputExtractionMode output_extraction_mode)
{
    switch (output_extraction_mode)
    {
        case APP_OUTPUT_EXTRACTION_MODE_RAW_PROBES: return "Raw Probes";
        case APP_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION: return "Mouth Radiation";
        case APP_OUTPUT_EXTRACTION_MODE_LISTENER_MODEL: return "Listener Model";
        case APP_OUTPUT_EXTRACTION_MODE_COUNT: break;
    }

    ASSERT(false);
    return "Unknown";
}

static void UpdateWindowTitle (AppState *app)
{
    ASSERT(app != NULL);
    PlatformWindow_SetTitle(&app->main_window, "PM-Organ");
}

static void BuildFdtdPresetDesc (
    FdtdPresetType preset_type,
    const AudioEngineDesc *engine_desc,
    Fdtd1DRenderSourceDesc *render_source_desc,
    Fdtd1DProbeDesc *probe_descs,
    Fdtd1DSourceDesc *source_descs,
    Fdtd1DAreaSegmentDesc *area_segment_descs,
    u32 *area_segment_count
)
{
    static const f64 TEST_COURANT_NUMBER = 0.9;
    f64 dx;

    ASSERT(engine_desc != NULL);
    ASSERT(render_source_desc != NULL);
    ASSERT(probe_descs != NULL);
    ASSERT(source_descs != NULL);
    ASSERT(area_segment_descs != NULL);
    ASSERT(area_segment_count != NULL);

    dx = 343.0 / (TEST_COURANT_NUMBER * 48000.0);

    probe_descs[0].type = FDTD_1D_PROBE_TYPE_PRESSURE;
    probe_descs[0].cell_index = 72;
    probe_descs[0].output_channel_index = 0;
    probe_descs[0].is_enabled = true;

    probe_descs[1].type = FDTD_1D_PROBE_TYPE_PRESSURE;
    probe_descs[1].cell_index = 104;
    probe_descs[1].output_channel_index = 1;
    probe_descs[1].is_enabled = true;

    probe_descs[2].type = FDTD_1D_PROBE_TYPE_PRESSURE;
    probe_descs[2].cell_index = 0;
    probe_descs[2].output_channel_index = 0;
    probe_descs[2].is_enabled = true;

    probe_descs[3].type = FDTD_1D_PROBE_TYPE_VELOCITY;
    probe_descs[3].cell_index = 0;
    probe_descs[3].output_channel_index = 0;
    probe_descs[3].is_enabled = true;

    probe_descs[4].type = FDTD_1D_PROBE_TYPE_PRESSURE;
    probe_descs[4].cell_index = 127;
    probe_descs[4].output_channel_index = 1;
    probe_descs[4].is_enabled = true;

    probe_descs[5].type = FDTD_1D_PROBE_TYPE_VELOCITY;
    probe_descs[5].cell_index = 128;
    probe_descs[5].output_channel_index = 1;
    probe_descs[5].is_enabled = true;

    probe_descs[6].type = FDTD_1D_PROBE_TYPE_LEFT_BOUNDARY_EMISSION;
    probe_descs[6].cell_index = 0;
    probe_descs[6].output_channel_index = 0;
    probe_descs[6].is_enabled = true;

    probe_descs[7].type = FDTD_1D_PROBE_TYPE_RIGHT_BOUNDARY_EMISSION;
    probe_descs[7].cell_index = 0;
    probe_descs[7].output_channel_index = 1;
    probe_descs[7].is_enabled = true;

    source_descs[0].cell_index = 6;
    source_descs[0].is_enabled = true;

    memset(render_source_desc, 0, sizeof(*render_source_desc));
    render_source_desc->solver_desc.sample_rate = engine_desc->sample_rate;
    render_source_desc->solver_desc.block_frame_count = engine_desc->block_frame_count;
    render_source_desc->solver_desc.output_channel_count = engine_desc->channel_count;
    render_source_desc->solver_desc.tube_length_m = 128.0 * dx;
    render_source_desc->solver_desc.wave_speed_m_per_s = 343.0;
    render_source_desc->solver_desc.density_kg_per_m3 = 1.225;
    render_source_desc->solver_desc.dx = dx;
    render_source_desc->solver_desc.pressure_cell_count = 128;
    render_source_desc->solver_desc.velocity_cell_count = 129;
    render_source_desc->solver_desc.courant_number = TEST_COURANT_NUMBER;
    render_source_desc->solver_desc.uniform_area_m2 = 0.01;
    render_source_desc->solver_desc.uniform_loss = 0.00005;
    render_source_desc->solver_desc.uniform_high_frequency_loss = 0.012;
    render_source_desc->solver_desc.uniform_boundary_loss = 0.00008;
    render_source_desc->solver_desc.uniform_boundary_high_frequency_loss = 0.028;
    render_source_desc->solver_desc.area_loss_reference_m2 = render_source_desc->solver_desc.uniform_area_m2;
    render_source_desc->solver_desc.area_loss_strength = 0.35;
    render_source_desc->solver_desc.open_end_correction_coefficient = 0.45;
    render_source_desc->solver_desc.open_end_radiation_resistance_scale = 1.5;
    render_source_desc->solver_desc.left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
    render_source_desc->solver_desc.left_boundary.reflection_coefficient = -1.0;
    render_source_desc->solver_desc.right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
    render_source_desc->solver_desc.right_boundary.reflection_coefficient = 1.0;
    render_source_desc->solver_desc.probe_count = 8;
    render_source_desc->solver_desc.probe_descs = probe_descs;
    render_source_desc->solver_desc.source_count = 1;
    render_source_desc->solver_desc.source_descs = source_descs;
    render_source_desc->excitation_mode = FDTD_1D_EXCITATION_MODE_IMPULSE;
    render_source_desc->source_coupling_mode = FDTD_1D_SOURCE_COUPLING_MODE_PRESSURE;
    render_source_desc->drive_amplitude = 0.0;
    render_source_desc->windchest_pressure = 1.0;
    render_source_desc->speech_attack_seconds = 0.06;
    render_source_desc->speech_chiff_amount = 0.35;
    render_source_desc->speech_chiff_decay_seconds = 0.05;
    render_source_desc->listener_distance_m = 4.5;
    render_source_desc->listener_mouth_pressure_mix = 0.18;
    render_source_desc->listener_crossfeed = 0.12;
    render_source_desc->listener_lowpass_cutoff_hz = 2800.0;
    render_source_desc->output_extraction_mode = FDTD_1D_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION;
    render_source_desc->startup_impulse_is_enabled = true;
    render_source_desc->startup_impulse_target_index = 0;
    render_source_desc->startup_impulse_amplitude = 0.25;

    *area_segment_count = 0;
    switch (preset_type)
    {
        case FDTD_PRESET_TYPE_UNIFORM_STOPPED:
        {
            source_descs[0].cell_index = 28;
        } break;

        case FDTD_PRESET_TYPE_NARROW_MOUTH_STOPPED:
        {
            area_segment_descs[0].start_cell_index = 0;
            area_segment_descs[0].end_cell_index = 20;
            area_segment_descs[0].area_m2 = 0.006;
            *area_segment_count = 1;
        } break;

        case FDTD_PRESET_TYPE_WIDE_MOUTH_STOPPED:
        {
            area_segment_descs[0].start_cell_index = 0;
            area_segment_descs[0].end_cell_index = 20;
            area_segment_descs[0].area_m2 = 0.014;
            *area_segment_count = 1;
        } break;

        case FDTD_PRESET_TYPE_OPEN_PIPE:
        {
            area_segment_descs[0].start_cell_index = 0;
            area_segment_descs[0].end_cell_index = 16;
            area_segment_descs[0].area_m2 = 0.012;
            area_segment_descs[1].start_cell_index = 112;
            area_segment_descs[1].end_cell_index = 128;
            area_segment_descs[1].area_m2 = 0.012;
            *area_segment_count = 2;
            source_descs[0].cell_index = 16;
            render_source_desc->solver_desc.right_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            render_source_desc->solver_desc.right_boundary.reflection_coefficient = -1.0;
        } break;

        case FDTD_PRESET_TYPE_COUNT:
        {
            ASSERT(false);
        } break;
    }

    render_source_desc->solver_desc.area_segment_count = *area_segment_count;
    render_source_desc->solver_desc.area_segment_descs =
        (*area_segment_count > 0) ? area_segment_descs : NULL;
}

static u32 RoundToMultipleU32 (u32 value, u32 multiple)
{
    if (multiple == 0)
    {
        return value;
    }

    return ((value + multiple - 1) / multiple) * multiple;
}

static void BuildRankVoiceDesc (
    u32 rank_note_index,
    const AudioEngineDesc *engine_desc,
    Fdtd1DRenderSourceDesc *render_source_desc,
    Fdtd1DProbeDesc *probe_descs,
    Fdtd1DSourceDesc *source_descs
)
{
    static const u32 BASE_PRESSURE_CELL_COUNT = 128;
    static const f64 SOURCE_RATIO = 28.0 / 128.0;
    static const f64 LEFT_PROBE_RATIO = 72.0 / 128.0;
    static const f64 RIGHT_PROBE_RATIO = 104.0 / 128.0;
    static const f64 TEST_COURANT_NUMBER = 0.9;
    static const u32 CELL_ALIGNMENT = 8;
    f64 dx;
    f64 length_ratio;
    u32 pressure_cell_count;
    u32 source_cell_index;
    u32 left_probe_index;
    u32 right_probe_index;
    Fdtd1DAreaSegmentDesc area_segment_descs[2];
    u32 area_segment_count;

    ASSERT(rank_note_index < RANK_NOTE_COUNT);
    ASSERT(engine_desc != NULL);
    ASSERT(render_source_desc != NULL);
    ASSERT(probe_descs != NULL);
    ASSERT(source_descs != NULL);

    BuildFdtdPresetDesc(
        FDTD_PRESET_TYPE_UNIFORM_STOPPED,
        engine_desc,
        render_source_desc,
        probe_descs,
        source_descs,
        area_segment_descs,
        &area_segment_count
    );

    length_ratio = pow(2.0, -(f64) RANK_NOTE_SEMITONE_OFFSETS[rank_note_index] / 12.0);
    pressure_cell_count = (u32) ((f64) BASE_PRESSURE_CELL_COUNT * length_ratio + 0.5);
    pressure_cell_count = RoundToMultipleU32(pressure_cell_count, CELL_ALIGNMENT);
    if (pressure_cell_count < 48)
    {
        pressure_cell_count = 48;
    }

    dx = 343.0 / (TEST_COURANT_NUMBER * (f64) engine_desc->sample_rate);
    source_cell_index = (u32) ((f64) pressure_cell_count * SOURCE_RATIO + 0.5);
    left_probe_index = (u32) ((f64) pressure_cell_count * LEFT_PROBE_RATIO + 0.5);
    right_probe_index = (u32) ((f64) pressure_cell_count * RIGHT_PROBE_RATIO + 0.5);

    if (source_cell_index >= pressure_cell_count)
    {
        source_cell_index = pressure_cell_count - 1;
    }
    if (left_probe_index >= pressure_cell_count)
    {
        left_probe_index = pressure_cell_count - 1;
    }
    if (right_probe_index >= pressure_cell_count)
    {
        right_probe_index = pressure_cell_count - 1;
    }

    render_source_desc->solver_desc.pressure_cell_count = pressure_cell_count;
    render_source_desc->solver_desc.velocity_cell_count = pressure_cell_count + 1;
    render_source_desc->solver_desc.tube_length_m = (f64) pressure_cell_count * dx;
    render_source_desc->solver_desc.dx = dx;
    render_source_desc->solver_desc.area_segment_count = 0;
    render_source_desc->solver_desc.area_segment_descs = NULL;
    render_source_desc->solver_desc.left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
    render_source_desc->solver_desc.left_boundary.reflection_coefficient = -1.0;
    render_source_desc->solver_desc.right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
    render_source_desc->solver_desc.right_boundary.reflection_coefficient = 1.0;

    probe_descs[0].cell_index = left_probe_index;
    probe_descs[1].cell_index = right_probe_index;
    probe_descs[2].cell_index = 0;
    probe_descs[3].cell_index = 0;
    probe_descs[4].cell_index = pressure_cell_count - 1;
    probe_descs[5].cell_index = pressure_cell_count;

    source_descs[0].cell_index = source_cell_index;
    render_source_desc->startup_impulse_target_index = 0;
}

static bool AnyRankNoteIsDown (const AppState *app)
{
    u32 note_index;

    ASSERT(app != NULL);

    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        if (app->rank_note_is_down[note_index])
        {
            return true;
        }
    }

    return false;
}

static void RenderRankVoices (
    AppState *app,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    f32 mix_gain;
    u32 active_note_count;
    u32 note_index;
    usize sample_count;
    usize sample_index;

    ASSERT(app != NULL);
    ASSERT(output != NULL);
    ASSERT(scratch_buffer != NULL);
    ASSERT(block_frame_count > 0);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    sample_count = (usize) block_frame_count * (usize) channel_count;
    memset(output, 0, sizeof(f32) * sample_count);

    active_note_count = 0;
    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        if (app->rank_note_is_down[note_index])
        {
            active_note_count += 1;
        }
    }

    if (active_note_count == 0)
    {
        return;
    }

    mix_gain = 1.0f / sqrtf((f32) active_note_count);
    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        if (app->rank_note_is_down[note_index] == false)
        {
            continue;
        }

        Fdtd1DRenderSource_Render(
            &app->rank_render_sources[note_index],
            scratch_buffer,
            scratch_buffer,
            block_frame_count,
            channel_count,
            sample_rate
        );

        for (sample_index = 0; sample_index < sample_count; sample_index += 1)
        {
            output[sample_index] += scratch_buffer[sample_index] * mix_gain;
        }
    }
}

static void RenderFdtdSource (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    AppState *app;
    usize sample_count;

    ASSERT(user_data != NULL);
    ASSERT(output != NULL);
    ASSERT(scratch_buffer != NULL);
    ASSERT(block_frame_count > 0);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    app = (AppState *) user_data;
    sample_count = (usize) block_frame_count * (usize) channel_count;
    if (AnyRankNoteIsDown(app))
    {
        RenderRankVoices(app, output, scratch_buffer, block_frame_count, channel_count, sample_rate);
        return;
    }
    memset(output, 0, sizeof(f32) * sample_count);
    memset(scratch_buffer, 0, sizeof(f32) * sample_count);
}

static void SetActiveRenderSource (AppState *app, bool use_fdtd_source)
{
    ASSERT(app != NULL);

    if (use_fdtd_source)
    {
        AudioEngine_SetRenderSource(
            &app->audio_engine,
            RenderFdtdSource,
            app
        );
    }
    else
    {
        AudioEngine_SetRenderSource(&app->audio_engine, TestToneSource_Render, &app->test_tone_source);
    }

    app->fdtd_source_is_active = use_fdtd_source;
    UpdateWindowTitle(app);
}

static void ApplyOutputExtractionModeToSources (AppState *app)
{
    u32 note_index;
    FdtdPresetType preset_type;

    ASSERT(app != NULL);

    for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
    {
        Fdtd1DRenderSource_SetOutputExtractionMode(
            &app->fdtd_render_sources[preset_type],
            ToRenderSourceOutputExtractionMode(app->active_output_extraction_mode)
        );
    }

    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        Fdtd1DRenderSource_SetOutputExtractionMode(
            &app->rank_render_sources[note_index],
            ToRenderSourceOutputExtractionMode(app->active_output_extraction_mode)
        );
    }
}

static void ApplyExcitationSettingsToSources (AppState *app)
{
    u32 note_index;
    FdtdPresetType preset_type;

    ASSERT(app != NULL);

    for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
    {
        Fdtd1DRenderSource_SetExcitationMode(
            &app->fdtd_render_sources[preset_type],
            ToRenderSourceExcitationMode(app->active_excitation_mode)
        );
        Fdtd1DRenderSource_SetSourceCouplingMode(
            &app->fdtd_render_sources[preset_type],
            ToRenderSourceSourceCouplingMode(app->active_source_coupling_mode)
        );
        Fdtd1DRenderSource_SetDriveAmplitude(
            &app->fdtd_render_sources[preset_type],
            (f64) app->drive_amplitude
        );
        Fdtd1DRenderSource_SetWindchestPressure(
            &app->fdtd_render_sources[preset_type],
            (f64) app->windchest_pressure
        );
        Fdtd1DRenderSource_SetSpeechAttackSeconds(
            &app->fdtd_render_sources[preset_type],
            (f64) app->speech_attack_seconds
        );
        Fdtd1DRenderSource_SetSpeechChiffAmount(
            &app->fdtd_render_sources[preset_type],
            (f64) app->speech_chiff_amount
        );
        Fdtd1DRenderSource_SetSpeechChiffDecaySeconds(
            &app->fdtd_render_sources[preset_type],
            (f64) app->speech_chiff_decay_seconds
        );
    }

    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        Fdtd1DRenderSource_SetExcitationMode(
            &app->rank_render_sources[note_index],
            ToRenderSourceExcitationMode(app->active_excitation_mode)
        );
        Fdtd1DRenderSource_SetSourceCouplingMode(
            &app->rank_render_sources[note_index],
            ToRenderSourceSourceCouplingMode(app->active_source_coupling_mode)
        );
        Fdtd1DRenderSource_SetDriveAmplitude(
            &app->rank_render_sources[note_index],
            (f64) app->drive_amplitude
        );
        Fdtd1DRenderSource_SetWindchestPressure(
            &app->rank_render_sources[note_index],
            (f64) app->windchest_pressure
        );
        Fdtd1DRenderSource_SetSpeechAttackSeconds(
            &app->rank_render_sources[note_index],
            (f64) app->speech_attack_seconds
        );
        Fdtd1DRenderSource_SetSpeechChiffAmount(
            &app->rank_render_sources[note_index],
            (f64) app->speech_chiff_amount
        );
        Fdtd1DRenderSource_SetSpeechChiffDecaySeconds(
            &app->rank_render_sources[note_index],
            (f64) app->speech_chiff_decay_seconds
        );
    }
}

static void ApplyListenerSettingsToSources (AppState *app)
{
    u32 note_index;
    FdtdPresetType preset_type;

    ASSERT(app != NULL);

    for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
    {
        Fdtd1DRenderSource_SetListenerDistance(
            &app->fdtd_render_sources[preset_type],
            (f64) app->listener_distance_m
        );
        Fdtd1DRenderSource_SetListenerMouthPressureMix(
            &app->fdtd_render_sources[preset_type],
            (f64) app->listener_mouth_pressure_mix
        );
        Fdtd1DRenderSource_SetListenerCrossfeed(
            &app->fdtd_render_sources[preset_type],
            (f64) app->listener_crossfeed
        );
        Fdtd1DRenderSource_SetListenerLowpassCutoff(
            &app->fdtd_render_sources[preset_type],
            (f64) app->listener_lowpass_cutoff_hz
        );
    }

    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        Fdtd1DRenderSource_SetListenerDistance(
            &app->rank_render_sources[note_index],
            (f64) app->listener_distance_m
        );
        Fdtd1DRenderSource_SetListenerMouthPressureMix(
            &app->rank_render_sources[note_index],
            (f64) app->listener_mouth_pressure_mix
        );
        Fdtd1DRenderSource_SetListenerCrossfeed(
            &app->rank_render_sources[note_index],
            (f64) app->listener_crossfeed
        );
        Fdtd1DRenderSource_SetListenerLowpassCutoff(
            &app->rank_render_sources[note_index],
            (f64) app->listener_lowpass_cutoff_hz
        );
    }
}

static void RestartSpeechForAllSources (AppState *app)
{
    u32 note_index;
    FdtdPresetType preset_type;

    ASSERT(app != NULL);

    for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
    {
        Fdtd1DRenderSource_RestartSpeech(&app->fdtd_render_sources[preset_type]);
    }

    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        Fdtd1DRenderSource_RestartSpeech(&app->rank_render_sources[note_index]);
    }
}

static void CycleFdtdPreset (AppState *app)
{
    ASSERT(app != NULL);

    app->active_fdtd_preset = (FdtdPresetType) ((app->active_fdtd_preset + 1) % FDTD_PRESET_TYPE_COUNT);
    if (app->fdtd_source_is_active)
    {
        AudioEngine_SetRenderSource(
            &app->audio_engine,
            RenderFdtdSource,
            app
        );
    }

    Fdtd1DRenderSource_TriggerStartupImpulse(&app->fdtd_render_sources[app->active_fdtd_preset]);
    UpdateWindowTitle(app);
}

static void SelectFdtdPreset (AppState *app, FdtdPresetType preset_type)
{
    ASSERT(app != NULL);
    ASSERT(preset_type < FDTD_PRESET_TYPE_COUNT);

    app->active_fdtd_preset = preset_type;
    if (app->fdtd_source_is_active)
    {
        AudioEngine_SetRenderSource(
            &app->audio_engine,
            RenderFdtdSource,
            app
        );
    }

    Fdtd1DRenderSource_TriggerStartupImpulse(&app->fdtd_render_sources[app->active_fdtd_preset]);
    UpdateWindowTitle(app);
}

static void SelectOutputExtractionMode (AppState *app, AppOutputExtractionMode output_extraction_mode)
{
    ASSERT(app != NULL);
    ASSERT(output_extraction_mode < APP_OUTPUT_EXTRACTION_MODE_COUNT);

    app->active_output_extraction_mode = output_extraction_mode;
    ApplyOutputExtractionModeToSources(app);
}

static void SelectExcitationMode (AppState *app, AppExcitationMode excitation_mode)
{
    ASSERT(app != NULL);
    ASSERT(excitation_mode < APP_EXCITATION_MODE_COUNT);

    app->active_excitation_mode = excitation_mode;
    ApplyExcitationSettingsToSources(app);
}

static void SelectSourceCouplingMode (AppState *app, AppSourceCouplingMode source_coupling_mode)
{
    ASSERT(app != NULL);
    ASSERT(source_coupling_mode < APP_SOURCE_COUPLING_MODE_COUNT);

    app->active_source_coupling_mode = source_coupling_mode;
    ApplyExcitationSettingsToSources(app);
}

static void SetDriveAmplitude (AppState *app, f32 drive_amplitude)
{
    ASSERT(app != NULL);

    if (drive_amplitude < 0.0f)
    {
        drive_amplitude = 0.0f;
    }

    app->drive_amplitude = drive_amplitude;
    ApplyExcitationSettingsToSources(app);
}

static void SetWindchestPressure (AppState *app, f32 windchest_pressure)
{
    ASSERT(app != NULL);

    if (windchest_pressure < 0.0f)
    {
        windchest_pressure = 0.0f;
    }

    app->windchest_pressure = windchest_pressure;
    ApplyExcitationSettingsToSources(app);
    RestartSpeechForAllSources(app);
}

static void SetSpeechAttackSeconds (AppState *app, f32 speech_attack_seconds)
{
    ASSERT(app != NULL);

    if (speech_attack_seconds < 0.0f)
    {
        speech_attack_seconds = 0.0f;
    }

    app->speech_attack_seconds = speech_attack_seconds;
    ApplyExcitationSettingsToSources(app);
    RestartSpeechForAllSources(app);
}

static void SetSpeechChiffAmount (AppState *app, f32 speech_chiff_amount)
{
    ASSERT(app != NULL);

    if (speech_chiff_amount < 0.0f)
    {
        speech_chiff_amount = 0.0f;
    }

    app->speech_chiff_amount = speech_chiff_amount;
    ApplyExcitationSettingsToSources(app);
    RestartSpeechForAllSources(app);
}

static void SetSpeechChiffDecaySeconds (AppState *app, f32 speech_chiff_decay_seconds)
{
    ASSERT(app != NULL);

    if (speech_chiff_decay_seconds < 0.0f)
    {
        speech_chiff_decay_seconds = 0.0f;
    }

    app->speech_chiff_decay_seconds = speech_chiff_decay_seconds;
    ApplyExcitationSettingsToSources(app);
    RestartSpeechForAllSources(app);
}

static u32 GetActivePresetSourceCellIndex (const AppState *app)
{
    const Fdtd1DState *state;

    ASSERT(app != NULL);

    state = Fdtd1D_GetState(&app->fdtd_render_sources[app->active_fdtd_preset].solver);
    ASSERT(state != NULL);

    if (state->source_count == 0)
    {
        return 0;
    }

    return state->source_cell_indices[0];
}

static u32 GetActivePresetMaxSourceCellIndex (const AppState *app)
{
    const Fdtd1DState *state;

    ASSERT(app != NULL);

    state = Fdtd1D_GetState(&app->fdtd_render_sources[app->active_fdtd_preset].solver);
    ASSERT(state != NULL);

    if (state->pressure_cell_count == 0)
    {
        return 0;
    }

    return state->pressure_cell_count - 1;
}

static void SetActivePresetSourceCellIndex (AppState *app, u32 source_cell_index)
{
    Fdtd1DRenderSource *render_source;
    u32 max_source_cell_index;

    ASSERT(app != NULL);

    render_source = &app->fdtd_render_sources[app->active_fdtd_preset];
    max_source_cell_index = GetActivePresetMaxSourceCellIndex(app);
    if (source_cell_index > max_source_cell_index)
    {
        source_cell_index = max_source_cell_index;
    }

    Fdtd1DRenderSource_SetSourceCellIndex(render_source, 0, source_cell_index);
    Fdtd1DRenderSource_RestartSpeech(render_source);
}

static void SetListenerDistance (AppState *app, f32 listener_distance_m)
{
    ASSERT(app != NULL);

    if (listener_distance_m < 0.0f)
    {
        listener_distance_m = 0.0f;
    }

    app->listener_distance_m = listener_distance_m;
    ApplyListenerSettingsToSources(app);
}

static void SetListenerMouthPressureMix (AppState *app, f32 listener_mouth_pressure_mix)
{
    ASSERT(app != NULL);

    if (listener_mouth_pressure_mix < 0.0f)
    {
        listener_mouth_pressure_mix = 0.0f;
    }
    if (listener_mouth_pressure_mix > 1.0f)
    {
        listener_mouth_pressure_mix = 1.0f;
    }

    app->listener_mouth_pressure_mix = listener_mouth_pressure_mix;
    ApplyListenerSettingsToSources(app);
}

static void SetListenerCrossfeed (AppState *app, f32 listener_crossfeed)
{
    ASSERT(app != NULL);

    if (listener_crossfeed < 0.0f)
    {
        listener_crossfeed = 0.0f;
    }
    if (listener_crossfeed > 1.0f)
    {
        listener_crossfeed = 1.0f;
    }

    app->listener_crossfeed = listener_crossfeed;
    ApplyListenerSettingsToSources(app);
}

static void SetListenerLowpassCutoff (AppState *app, f32 listener_lowpass_cutoff_hz)
{
    ASSERT(app != NULL);

    if (listener_lowpass_cutoff_hz < 0.0f)
    {
        listener_lowpass_cutoff_hz = 0.0f;
    }

    app->listener_lowpass_cutoff_hz = listener_lowpass_cutoff_hz;
    ApplyListenerSettingsToSources(app);
}

static void UpdateDebugInput (AppState *app)
{
    static const i32 RANK_NOTE_KEYS[RANK_NOTE_COUNT] =
    {
        'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K'
    };
    bool cycle_is_down;
    u32 note_index;
    bool space_is_down;
    bool toggle_is_down;

    ASSERT(app != NULL);

    space_is_down = (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;
    toggle_is_down = (GetAsyncKeyState('T') & 0x8000) != 0;
    cycle_is_down = false;

    if (space_is_down && (app->previous_space_is_down == false))
    {
        Fdtd1DRenderSource_TriggerStartupImpulse(&app->fdtd_render_sources[app->active_fdtd_preset]);
    }

    if (toggle_is_down && (app->previous_toggle_is_down == false))
    {
        SetActiveRenderSource(app, app->fdtd_source_is_active == false);
    }

    for (note_index = 0; note_index < RANK_NOTE_COUNT; note_index += 1)
    {
        bool note_is_down;

        note_is_down = (GetAsyncKeyState(RANK_NOTE_KEYS[note_index]) & 0x8000) != 0;
        app->rank_note_is_down[note_index] = note_is_down;
        if (note_is_down && (app->previous_rank_note_is_down[note_index] == false))
        {
            Fdtd1DRenderSource_RestartSpeech(&app->rank_render_sources[note_index]);
            if (app->active_excitation_mode == APP_EXCITATION_MODE_IMPULSE)
            {
                Fdtd1DRenderSource_TriggerStartupImpulse(&app->rank_render_sources[note_index]);
            }
        }
        app->previous_rank_note_is_down[note_index] = note_is_down;
    }

    app->previous_space_is_down = space_is_down;
    app->previous_toggle_is_down = toggle_is_down;
    app->previous_cycle_is_down = cycle_is_down;
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
    TestToneSourceDesc test_tone_desc;
    MemoryArena bootstrap_arena;
    WindowDesc window_desc;
    FdtdPresetType preset_type;

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
    FrameTimer_SetTargetHz(&app->frame_timer, 240.0);

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

    if (DebugGui_Initialize(&app->debug_gui, &bootstrap_arena, &app->main_window) == false)
    {
        AudioEngine_Shutdown(&app->audio_engine);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    PlatformWindow_SetMessageCallback(&app->main_window, DebugGui_WindowMessageCallback, &app->debug_gui);

    for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
    {
        Fdtd1DAreaSegmentDesc area_segment_descs[2];
        Fdtd1DProbeDesc probe_descs[8];
        Fdtd1DRenderSourceDesc fdtd_render_source_desc;
        Fdtd1DSourceDesc source_descs[1];
        u32 area_segment_count;

        BuildFdtdPresetDesc(
            preset_type,
            &engine_desc,
            &fdtd_render_source_desc,
            probe_descs,
            source_descs,
            area_segment_descs,
            &area_segment_count
        );

        if (Fdtd1DRenderSource_Initialize(
            &app->fdtd_render_sources[preset_type],
            &bootstrap_arena,
            &fdtd_render_source_desc
        ) == false)
        {
            for (; preset_type > 0; preset_type = (FdtdPresetType) (preset_type - 1))
            {
                Fdtd1DRenderSource_Shutdown(&app->fdtd_render_sources[preset_type - 1]);
            }

            DebugGui_Shutdown(&app->debug_gui);
            AudioEngine_Shutdown(&app->audio_engine);
            PlatformWindow_Destroy(&app->main_window);
            MemoryArena_Destroy(&bootstrap_arena);
            return 1;
        }
    }

    for (u32 rank_note_index = 0; rank_note_index < RANK_NOTE_COUNT; rank_note_index += 1)
    {
        Fdtd1DProbeDesc probe_descs[8];
        Fdtd1DRenderSourceDesc fdtd_render_source_desc;
        Fdtd1DSourceDesc source_descs[1];

        BuildRankVoiceDesc(
            rank_note_index,
            &engine_desc,
            &fdtd_render_source_desc,
            probe_descs,
            source_descs
        );

        if (Fdtd1DRenderSource_Initialize(
            &app->rank_render_sources[rank_note_index],
            &bootstrap_arena,
            &fdtd_render_source_desc
        ) == false)
        {
            for (u32 shutdown_rank_note_index = 0;
                 shutdown_rank_note_index < rank_note_index;
                 shutdown_rank_note_index += 1)
            {
                Fdtd1DRenderSource_Shutdown(&app->rank_render_sources[shutdown_rank_note_index]);
            }
            for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
            {
                Fdtd1DRenderSource_Shutdown(&app->fdtd_render_sources[preset_type]);
            }

            DebugGui_Shutdown(&app->debug_gui);
            AudioEngine_Shutdown(&app->audio_engine);
            PlatformWindow_Destroy(&app->main_window);
            MemoryArena_Destroy(&bootstrap_arena);
            return 1;
        }
    }

    test_tone_desc.frequency_hz = 220.0;
    test_tone_desc.amplitude = 0.05f;

    if (TestToneSource_Initialize(&app->test_tone_source, &test_tone_desc) == false)
    {
        for (u32 rank_note_index = 0; rank_note_index < RANK_NOTE_COUNT; rank_note_index += 1)
        {
            Fdtd1DRenderSource_Shutdown(&app->rank_render_sources[rank_note_index]);
        }
        for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
        {
            Fdtd1DRenderSource_Shutdown(&app->fdtd_render_sources[preset_type]);
        }
        DebugGui_Shutdown(&app->debug_gui);
        AudioEngine_Shutdown(&app->audio_engine);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    app->previous_space_is_down = false;
    app->previous_toggle_is_down = false;
    app->previous_cycle_is_down = false;
    memset(app->rank_note_is_down, 0, sizeof(app->rank_note_is_down));
    memset(app->previous_rank_note_is_down, 0, sizeof(app->previous_rank_note_is_down));
    app->active_fdtd_preset = FDTD_PRESET_TYPE_NARROW_MOUTH_STOPPED;
    app->active_excitation_mode = APP_EXCITATION_MODE_IMPULSE;
    app->active_source_coupling_mode = APP_SOURCE_COUPLING_MODE_PRESSURE;
    app->active_output_extraction_mode = APP_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION;
    app->drive_amplitude = 0.0f;
    app->windchest_pressure = 1.0f;
    app->speech_attack_seconds = 0.06f;
    app->speech_chiff_amount = 0.35f;
    app->speech_chiff_decay_seconds = 0.05f;
    app->listener_distance_m = 4.5f;
    app->listener_mouth_pressure_mix = 0.18f;
    app->listener_crossfeed = 0.12f;
    app->listener_lowpass_cutoff_hz = 2800.0f;
    app->master_gain = 0.35f;
    app->output_is_muted = false;
    AudioEngine_SetMasterGain(&app->audio_engine, app->master_gain);
    AudioEngine_SetOutputMuted(&app->audio_engine, app->output_is_muted);
    ApplyExcitationSettingsToSources(app);
    ApplyOutputExtractionModeToSources(app);
    ApplyListenerSettingsToSources(app);
    SetActiveRenderSource(app, true);

    audio_desc.sample_rate = 48000;
    audio_desc.channel_count = 2;
    audio_desc.frames_per_buffer = 256;
    audio_desc.block_frame_count = app->audio_engine.config.block_frame_count;
    audio_desc.render_callback = RenderEngineBlock;
    audio_desc.user_data = app;

    if (AudioDevice_Open(&app->audio_device, &bootstrap_arena, &audio_desc) == false)
    {
        for (u32 rank_note_index = 0; rank_note_index < RANK_NOTE_COUNT; rank_note_index += 1)
        {
            Fdtd1DRenderSource_Shutdown(&app->rank_render_sources[rank_note_index]);
        }
        DebugGui_Shutdown(&app->debug_gui);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    if (AudioDevice_Start(&app->audio_device) == false)
    {
        AudioDevice_Close(&app->audio_device);
        for (u32 rank_note_index = 0; rank_note_index < RANK_NOTE_COUNT; rank_note_index += 1)
        {
            Fdtd1DRenderSource_Shutdown(&app->rank_render_sources[rank_note_index]);
        }
        DebugGui_Shutdown(&app->debug_gui);
        PlatformWindow_Destroy(&app->main_window);
        MemoryArena_Destroy(&bootstrap_arena);
        return 1;
    }

    while (app->main_window.is_running)
    {
        const Fdtd1DRenderSource *active_render_source;
        DebugGuiFrameActions gui_actions;
        DebugGuiFrameDesc gui_frame_desc;
        const char *excitation_mode_names[APP_EXCITATION_MODE_COUNT];
        const char *source_coupling_mode_names[APP_SOURCE_COUPLING_MODE_COUNT];
        const char *output_extraction_mode_names[APP_OUTPUT_EXTRACTION_MODE_COUNT];
        const char *preset_names[FDTD_PRESET_TYPE_COUNT];

        FrameTimer_BeginFrame(&app->frame_timer);
        PlatformWindow_PumpMessages(&app->main_window);
        UpdateDebugInput(app);

        for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
        {
            preset_names[preset_type] = GetFdtdPresetName(preset_type);
        }
        active_render_source = &app->fdtd_render_sources[app->active_fdtd_preset];
        excitation_mode_names[0] = GetExcitationModeName(APP_EXCITATION_MODE_IMPULSE);
        excitation_mode_names[1] = GetExcitationModeName(APP_EXCITATION_MODE_CONSTANT);
        excitation_mode_names[2] = GetExcitationModeName(APP_EXCITATION_MODE_NOISE);
        excitation_mode_names[3] = GetExcitationModeName(APP_EXCITATION_MODE_BIAS_AND_NOISE);
        excitation_mode_names[4] = GetExcitationModeName(APP_EXCITATION_MODE_FEEDBACK_MOUTH);
        excitation_mode_names[5] = GetExcitationModeName(APP_EXCITATION_MODE_NONLINEAR_MOUTH);
        excitation_mode_names[6] = GetExcitationModeName(APP_EXCITATION_MODE_JET_LABIUM);
        source_coupling_mode_names[0] = GetSourceCouplingModeName(APP_SOURCE_COUPLING_MODE_PRESSURE);
        source_coupling_mode_names[1] = GetSourceCouplingModeName(APP_SOURCE_COUPLING_MODE_VELOCITY);
        output_extraction_mode_names[0] = GetOutputExtractionModeName(APP_OUTPUT_EXTRACTION_MODE_RAW_PROBES);
        output_extraction_mode_names[1] = GetOutputExtractionModeName(APP_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION);
        output_extraction_mode_names[2] = GetOutputExtractionModeName(APP_OUTPUT_EXTRACTION_MODE_LISTENER_MODEL);

        gui_frame_desc.fdtd_source_is_active = app->fdtd_source_is_active;
        gui_frame_desc.output_is_muted = app->output_is_muted;
        gui_frame_desc.drive_amplitude = app->drive_amplitude;
        gui_frame_desc.windchest_pressure = app->windchest_pressure;
        gui_frame_desc.speech_attack_seconds = app->speech_attack_seconds;
        gui_frame_desc.speech_chiff_amount = app->speech_chiff_amount;
        gui_frame_desc.speech_chiff_decay_seconds = app->speech_chiff_decay_seconds;
        gui_frame_desc.listener_distance_m = app->listener_distance_m;
        gui_frame_desc.listener_mouth_pressure_mix = app->listener_mouth_pressure_mix;
        gui_frame_desc.listener_crossfeed = app->listener_crossfeed;
        gui_frame_desc.listener_lowpass_cutoff_hz = app->listener_lowpass_cutoff_hz;
        gui_frame_desc.effective_drive_requested = (f32) active_render_source->last_requested_drive;
        gui_frame_desc.effective_drive_applied = (f32) active_render_source->last_applied_drive;
        gui_frame_desc.effective_drive_saturation_ratio = (f32) active_render_source->last_drive_saturation_ratio;
        gui_frame_desc.master_gain = app->master_gain;
        gui_frame_desc.source_cell_index = GetActivePresetSourceCellIndex(app);
        gui_frame_desc.source_cell_index_max = GetActivePresetMaxSourceCellIndex(app);
        gui_frame_desc.active_preset_index = app->active_fdtd_preset;
        gui_frame_desc.active_excitation_mode = app->active_excitation_mode;
        gui_frame_desc.excitation_mode_count = APP_EXCITATION_MODE_COUNT;
        gui_frame_desc.active_source_coupling_mode = app->active_source_coupling_mode;
        gui_frame_desc.source_coupling_mode_count = APP_SOURCE_COUPLING_MODE_COUNT;
        gui_frame_desc.active_output_extraction_mode = app->active_output_extraction_mode;
        gui_frame_desc.output_extraction_mode_count = APP_OUTPUT_EXTRACTION_MODE_COUNT;
        gui_frame_desc.preset_count = FDTD_PRESET_TYPE_COUNT;
        gui_frame_desc.excitation_mode_names = excitation_mode_names;
        gui_frame_desc.source_coupling_mode_names = source_coupling_mode_names;
        gui_frame_desc.output_extraction_mode_names = output_extraction_mode_names;
        gui_frame_desc.preset_names = preset_names;
        gui_frame_desc.delta_seconds = app->frame_timer.delta_seconds;

        DebugGui_BeginFrame(&app->debug_gui);
        DebugGui_Draw(&app->debug_gui, &gui_frame_desc, &gui_actions);

        if (gui_actions.request_use_fdtd_source)
        {
            SetActiveRenderSource(app, true);
        }

        if (gui_actions.request_use_test_tone)
        {
            SetActiveRenderSource(app, false);
        }

        if (gui_actions.request_select_preset)
        {
            SelectFdtdPreset(app, (FdtdPresetType) gui_actions.selected_preset_index);
        }

        if (gui_actions.request_set_source_cell_index)
        {
            SetActivePresetSourceCellIndex(app, gui_actions.source_cell_index);
        }

        if (gui_actions.request_select_excitation_mode)
        {
            SelectExcitationMode(app, (AppExcitationMode) gui_actions.selected_excitation_mode);
        }

        if (gui_actions.request_select_source_coupling_mode)
        {
            SelectSourceCouplingMode(app, (AppSourceCouplingMode) gui_actions.selected_source_coupling_mode);
        }

        if (gui_actions.request_select_output_extraction_mode)
        {
            SelectOutputExtractionMode(app, (AppOutputExtractionMode) gui_actions.selected_output_extraction_mode);
        }

        if (gui_actions.request_set_drive_amplitude)
        {
            SetDriveAmplitude(app, gui_actions.drive_amplitude);
        }

        if (gui_actions.request_set_windchest_pressure)
        {
            SetWindchestPressure(app, gui_actions.windchest_pressure);
        }

        if (gui_actions.request_set_speech_attack_seconds)
        {
            SetSpeechAttackSeconds(app, gui_actions.speech_attack_seconds);
        }

        if (gui_actions.request_set_speech_chiff_amount)
        {
            SetSpeechChiffAmount(app, gui_actions.speech_chiff_amount);
        }

        if (gui_actions.request_set_speech_chiff_decay_seconds)
        {
            SetSpeechChiffDecaySeconds(app, gui_actions.speech_chiff_decay_seconds);
        }

        if (gui_actions.request_set_listener_distance)
        {
            SetListenerDistance(app, gui_actions.listener_distance_m);
        }

        if (gui_actions.request_set_listener_mouth_pressure_mix)
        {
            SetListenerMouthPressureMix(app, gui_actions.listener_mouth_pressure_mix);
        }

        if (gui_actions.request_set_listener_crossfeed)
        {
            SetListenerCrossfeed(app, gui_actions.listener_crossfeed);
        }

        if (gui_actions.request_set_listener_lowpass_cutoff)
        {
            SetListenerLowpassCutoff(app, gui_actions.listener_lowpass_cutoff_hz);
        }

        if (gui_actions.request_set_master_gain)
        {
            app->master_gain = gui_actions.master_gain;
            AudioEngine_SetMasterGain(&app->audio_engine, app->master_gain);
        }

        if (gui_actions.request_set_output_muted)
        {
            app->output_is_muted = gui_actions.output_is_muted;
            AudioEngine_SetOutputMuted(&app->audio_engine, app->output_is_muted);
        }

        if (gui_actions.request_kill_output)
        {
            app->output_is_muted = true;
            AudioEngine_KillOutput(&app->audio_engine);
        }

        if (gui_actions.request_trigger_impulse)
        {
            Fdtd1DRenderSource_TriggerStartupImpulse(&app->fdtd_render_sources[app->active_fdtd_preset]);
        }

        DebugGui_Render(&app->debug_gui);
        FrameTimer_EndFrame(&app->frame_timer);
    }

    AudioDevice_Close(&app->audio_device);
    DebugGui_Shutdown(&app->debug_gui);
    for (preset_type = 0; preset_type < FDTD_PRESET_TYPE_COUNT; preset_type = (FdtdPresetType) (preset_type + 1))
    {
        Fdtd1DRenderSource_Shutdown(&app->fdtd_render_sources[preset_type]);
    }
    for (u32 rank_note_index = 0; rank_note_index < RANK_NOTE_COUNT; rank_note_index += 1)
    {
        Fdtd1DRenderSource_Shutdown(&app->rank_render_sources[rank_note_index]);
    }
    AudioEngine_Shutdown(&app->audio_engine);
    PlatformWindow_Destroy(&app->main_window);
    MemoryArena_Destroy(&bootstrap_arena);

    return 0;
}
