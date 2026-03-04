#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/core/memory_arena.h"
#include "pm_organ/sim/fdtd_1d.h"

#define MODE_COUNT 5

typedef struct VerificationResult
{
    u32 first_arrival_frame;
    f32 first_arrival_value;
    f32 peak_abs_sample;
    u32 peak_frame;
    bool first_arrival_was_found;
} VerificationResult;

typedef struct WindowPeakResult
{
    u32 start_frame;
    u32 end_frame;
    u32 peak_frame;
    f32 peak_value;
    f32 peak_abs_value;
} WindowPeakResult;

typedef struct WindowEnergyResult
{
    u32 start_frame;
    u32 end_frame;
    f64 energy_sum;
    f64 energy_centroid_frame;
} WindowEnergyResult;

typedef struct EnergyResult
{
    f64 initial_energy;
    f64 minimum_energy;
    f64 maximum_energy;
    f64 final_energy;
} EnergyResult;

typedef struct ArrivalSummary
{
    f64 expected_frame;
    VerificationResult first_crossing;
    WindowPeakResult main_lobe_peak;
} ArrivalSummary;

typedef struct ModeResult
{
    f64 expected_frequency_hz;
    f64 measured_peak_frequency_hz;
    f64 measured_peak_magnitude;
} ModeResult;

typedef enum VerificationExcitationType
{
    VERIFICATION_EXCITATION_TYPE_IMPULSE = 0,
    VERIFICATION_EXCITATION_TYPE_NOISE_BURST,
} VerificationExcitationType;

typedef enum VerificationPreset
{
    VERIFICATION_PRESET_RIGID_RIGID = 0,
    VERIFICATION_PRESET_OPEN_OPEN,
    VERIFICATION_PRESET_OPEN_RIGID,
    VERIFICATION_PRESET_STAGE2_STOPPED,
} VerificationPreset;

typedef struct VerificationSettings
{
    VerificationPreset preset;
    Fdtd1DProbeType probe_type;
    VerificationExcitationType excitation_type;
    const char *csv_output_path;
    u32 block_count;
    u32 source_cell_index;
    u32 left_probe_index;
    u32 right_probe_index;
} VerificationSettings;

static const char *GetProbeTypeName (Fdtd1DProbeType probe_type)
{
    switch (probe_type)
    {
        case FDTD_1D_PROBE_TYPE_PRESSURE: return "pressure";
        case FDTD_1D_PROBE_TYPE_VELOCITY: return "velocity";
    }

    ASSERT(false);
    return "unknown";
}

static const char *GetBoundaryTypeName (Fdtd1DBoundaryType boundary_type)
{
    switch (boundary_type)
    {
        case FDTD_1D_BOUNDARY_TYPE_RIGID: return "rigid";
        case FDTD_1D_BOUNDARY_TYPE_OPEN: return "open";
        case FDTD_1D_BOUNDARY_TYPE_REFLECTION_COEFFICIENT: return "reflection";
    }

    ASSERT(false);
    return "unknown";
}

static const char *GetPresetName (VerificationPreset preset)
{
    switch (preset)
    {
        case VERIFICATION_PRESET_RIGID_RIGID: return "rigid-rigid";
        case VERIFICATION_PRESET_OPEN_OPEN: return "open-open";
        case VERIFICATION_PRESET_OPEN_RIGID: return "open-rigid";
        case VERIFICATION_PRESET_STAGE2_STOPPED: return "stage2-stopped";
    }

    ASSERT(false);
    return "unknown";
}

static bool TryParsePresetName (const char *text, VerificationPreset *preset)
{
    ASSERT(text != NULL);
    ASSERT(preset != NULL);

    if (_stricmp(text, "rigid-rigid") == 0)
    {
        *preset = VERIFICATION_PRESET_RIGID_RIGID;
        return true;
    }

    if (_stricmp(text, "open-open") == 0)
    {
        *preset = VERIFICATION_PRESET_OPEN_OPEN;
        return true;
    }

    if ((_stricmp(text, "open-rigid") == 0) || (_stricmp(text, "stopped") == 0))
    {
        *preset = VERIFICATION_PRESET_OPEN_RIGID;
        return true;
    }

    if ((_stricmp(text, "stage2-stopped") == 0) || (_stricmp(text, "stepped-stopped") == 0))
    {
        *preset = VERIFICATION_PRESET_STAGE2_STOPPED;
        return true;
    }

    return false;
}

static const char *GetExcitationTypeName (VerificationExcitationType excitation_type)
{
    switch (excitation_type)
    {
        case VERIFICATION_EXCITATION_TYPE_IMPULSE: return "impulse";
        case VERIFICATION_EXCITATION_TYPE_NOISE_BURST: return "noise-burst";
    }

    ASSERT(false);
    return "unknown";
}

static void ConfigureSolver (
    const VerificationSettings *settings,
    Fdtd1DDesc *desc,
    Fdtd1DAreaSegmentDesc *area_segment_descs,
    Fdtd1DProbeDesc *probe_descs,
    Fdtd1DSourceDesc *source_descs
)
{
    static const f64 TEST_COURANT_NUMBER = 0.9;

    ASSERT(settings != NULL);
    ASSERT(desc != NULL);
    ASSERT(area_segment_descs != NULL);
    ASSERT(probe_descs != NULL);
    ASSERT(source_descs != NULL);

    probe_descs[0].type = settings->probe_type;
    probe_descs[0].cell_index = settings->left_probe_index;
    probe_descs[0].output_channel_index = 0;
    probe_descs[0].is_enabled = true;

    probe_descs[1].type = settings->probe_type;
    probe_descs[1].cell_index = settings->right_probe_index;
    probe_descs[1].output_channel_index = 1;
    probe_descs[1].is_enabled = true;

    source_descs[0].cell_index = settings->source_cell_index;
    source_descs[0].is_enabled = true;

    desc->sample_rate = 48000;
    desc->block_frame_count = 64;
    desc->output_channel_count = 2;
    desc->tube_length_m = 128.0 * (343.0 / (TEST_COURANT_NUMBER * 48000.0));
    desc->wave_speed_m_per_s = 343.0;
    desc->density_kg_per_m3 = 1.225;
    desc->dx = 343.0 / (TEST_COURANT_NUMBER * 48000.0);
    desc->pressure_cell_count = 128;
    desc->velocity_cell_count = 129;
    desc->courant_number = TEST_COURANT_NUMBER;
    desc->uniform_area_m2 = 0.01;
    desc->uniform_loss = 0.00005;
    desc->area_segment_count = 0;
    desc->area_segment_descs = NULL;
    switch (settings->preset)
    {
        case VERIFICATION_PRESET_RIGID_RIGID:
        {
            desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
            desc->left_boundary.reflection_coefficient = 1.0;
            desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
            desc->right_boundary.reflection_coefficient = 1.0;
        } break;

        case VERIFICATION_PRESET_OPEN_OPEN:
        {
            desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->left_boundary.reflection_coefficient = -1.0;
            desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->right_boundary.reflection_coefficient = -1.0;
        } break;

        case VERIFICATION_PRESET_OPEN_RIGID:
        {
            desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->left_boundary.reflection_coefficient = -1.0;
            desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
            desc->right_boundary.reflection_coefficient = 1.0;
        } break;

        case VERIFICATION_PRESET_STAGE2_STOPPED:
        {
            area_segment_descs[0].start_cell_index = 0;
            area_segment_descs[0].end_cell_index = 20;
            area_segment_descs[0].area_m2 = 0.006;

            desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->left_boundary.reflection_coefficient = -1.0;
            desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
            desc->right_boundary.reflection_coefficient = 1.0;
            desc->area_segment_count = 1;
            desc->area_segment_descs = area_segment_descs;
        } break;
    }

    desc->probe_count = 2;
    desc->probe_descs = probe_descs;
    desc->source_count = 1;
    desc->source_descs = source_descs;
}

static bool TryParseProbeTypeName (const char *text, Fdtd1DProbeType *probe_type)
{
    ASSERT(text != NULL);
    ASSERT(probe_type != NULL);

    if (_stricmp(text, "pressure") == 0)
    {
        *probe_type = FDTD_1D_PROBE_TYPE_PRESSURE;
        return true;
    }

    if (_stricmp(text, "velocity") == 0)
    {
        *probe_type = FDTD_1D_PROBE_TYPE_VELOCITY;
        return true;
    }

    return false;
}

static bool TryParseExcitationTypeName (const char *text, VerificationExcitationType *excitation_type)
{
    ASSERT(text != NULL);
    ASSERT(excitation_type != NULL);

    if (_stricmp(text, "impulse") == 0)
    {
        *excitation_type = VERIFICATION_EXCITATION_TYPE_IMPULSE;
        return true;
    }

    if ((_stricmp(text, "noise-burst") == 0) || (_stricmp(text, "noise") == 0))
    {
        *excitation_type = VERIFICATION_EXCITATION_TYPE_NOISE_BURST;
        return true;
    }

    return false;
}

static bool TryParseUnsignedValue (const char *text, const char *prefix, u32 *value)
{
    usize prefix_length;
    char *end_pointer;
    unsigned long parsed_value;

    ASSERT(text != NULL);
    ASSERT(prefix != NULL);
    ASSERT(value != NULL);

    prefix_length = strlen(prefix);
    if (_strnicmp(text, prefix, prefix_length) != 0)
    {
        return false;
    }

    parsed_value = strtoul(text + prefix_length, &end_pointer, 10);
    if ((end_pointer == (text + prefix_length)) || (*end_pointer != '\0'))
    {
        return false;
    }

    *value = (u32) parsed_value;
    return true;
}

static void InitializeVerificationSettings (VerificationSettings *settings)
{
    ASSERT(settings != NULL);

    settings->preset = VERIFICATION_PRESET_RIGID_RIGID;
    settings->probe_type = FDTD_1D_PROBE_TYPE_PRESSURE;
    settings->excitation_type = VERIFICATION_EXCITATION_TYPE_IMPULSE;
    settings->csv_output_path = NULL;
    settings->block_count = 256;
    settings->source_cell_index = 16;
    settings->left_probe_index = 96;
    settings->right_probe_index = 112;
}

static void ParseArguments (int argc, char **argv, VerificationSettings *settings)
{
    int argument_index;

    ASSERT(settings != NULL);

    for (argument_index = 1; argument_index < argc; argument_index += 1)
    {
        const char *argument;
        u32 parsed_value;

        argument = argv[argument_index];

        if (TryParsePresetName(argument, &settings->preset))
        {
            continue;
        }

        if (TryParseProbeTypeName(argument, &settings->probe_type))
        {
            continue;
        }

        if (TryParseExcitationTypeName(argument, &settings->excitation_type))
        {
            continue;
        }

        if (TryParseUnsignedValue(argument, "blocks=", &parsed_value))
        {
            if (parsed_value > 0)
            {
                settings->block_count = parsed_value;
            }
            continue;
        }

        if (TryParseUnsignedValue(argument, "source=", &parsed_value))
        {
            settings->source_cell_index = parsed_value;
            continue;
        }

        if (TryParseUnsignedValue(argument, "left_probe=", &parsed_value))
        {
            settings->left_probe_index = parsed_value;
            continue;
        }

        if (TryParseUnsignedValue(argument, "right_probe=", &parsed_value))
        {
            settings->right_probe_index = parsed_value;
            continue;
        }

        settings->csv_output_path = argument;
    }
}

static bool WriteCaptureCsv (const char *path, const SimulationOfflineCapture *capture)
{
    FILE *file;
    u32 frame_index;
    u32 channel_index;

    ASSERT(path != NULL);
    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);

    file = NULL;
    if (fopen_s(&file, path, "w") != 0)
    {
        file = NULL;
    }

    if (file == NULL)
    {
        return false;
    }

    fprintf(file, "frame");
    for (channel_index = 0; channel_index < capture->channel_count; channel_index += 1)
    {
        fprintf(file, ",ch%u", channel_index);
    }
    fprintf(file, "\n");

    for (frame_index = 0; frame_index < capture->frame_count; frame_index += 1)
    {
        usize sample_offset;

        fprintf(file, "%u", frame_index);
        sample_offset = (usize) frame_index * (usize) capture->channel_count;

        for (channel_index = 0; channel_index < capture->channel_count; channel_index += 1)
        {
            fprintf(file, ",%.9f", capture->samples[sample_offset + channel_index]);
        }

        fprintf(file, "\n");
    }

    fclose(file);
    return true;
}

static VerificationResult AnalyzeCapture (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    f32 first_arrival_threshold
)
{
    VerificationResult result;
    u32 frame_index;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);

    result.first_arrival_frame = 0;
    result.first_arrival_value = 0.0f;
    result.peak_abs_sample = 0.0f;
    result.peak_frame = 0;
    result.first_arrival_was_found = false;

    for (frame_index = 0; frame_index < capture->frame_count; frame_index += 1)
    {
        f32 sample;
        f32 abs_sample;

        sample = capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        abs_sample = (sample < 0.0f) ? -sample : sample;

        if ((result.first_arrival_was_found == false) && (abs_sample >= first_arrival_threshold))
        {
            result.first_arrival_frame = frame_index;
            result.first_arrival_value = sample;
            result.first_arrival_was_found = true;
        }

        if (abs_sample > result.peak_abs_sample)
        {
            result.peak_abs_sample = abs_sample;
            result.peak_frame = frame_index;
        }
    }

    return result;
}

static WindowPeakResult AnalyzeWindowPeak (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 end_frame
)
{
    WindowPeakResult result;
    u32 frame_index;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);
    ASSERT(start_frame <= end_frame);

    result.start_frame = start_frame;
    result.end_frame = end_frame;
    result.peak_frame = start_frame;
    result.peak_value = 0.0f;
    result.peak_abs_value = 0.0f;

    if (capture->frame_count == 0)
    {
        return result;
    }

    if (result.end_frame >= capture->frame_count)
    {
        result.end_frame = capture->frame_count - 1;
    }

    for (frame_index = result.start_frame; frame_index <= result.end_frame; frame_index += 1)
    {
        f32 sample;
        f32 abs_sample;

        sample = capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        abs_sample = (sample < 0.0f) ? -sample : sample;

        if (abs_sample > result.peak_abs_value)
        {
            result.peak_frame = frame_index;
            result.peak_value = sample;
            result.peak_abs_value = abs_sample;
        }
    }

    return result;
}

static WindowEnergyResult AnalyzeWindowEnergy (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 end_frame
)
{
    WindowEnergyResult result;
    f64 weighted_sum;
    u32 frame_index;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);
    ASSERT(start_frame <= end_frame);

    result.start_frame = start_frame;
    result.end_frame = end_frame;
    result.energy_sum = 0.0;
    result.energy_centroid_frame = (f64) start_frame;

    if (capture->frame_count == 0)
    {
        return result;
    }

    if (result.end_frame >= capture->frame_count)
    {
        result.end_frame = capture->frame_count - 1;
    }

    weighted_sum = 0.0;

    for (frame_index = result.start_frame; frame_index <= result.end_frame; frame_index += 1)
    {
        f64 sample;
        f64 sample_energy;

        sample = (f64) capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        sample_energy = sample * sample;

        result.energy_sum += sample_energy;
        weighted_sum += (f64) frame_index * sample_energy;
    }

    if (result.energy_sum > 0.0)
    {
        result.energy_centroid_frame = weighted_sum / result.energy_sum;
    }

    return result;
}

static f64 ComputeSpectrumMagnitude (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 end_frame,
    f64 sample_rate,
    f64 frequency_hz
)
{
    f64 real_sum;
    f64 imaginary_sum;
    u32 frame_index;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);
    ASSERT(sample_rate > 0.0);
    ASSERT(start_frame <= end_frame);

    real_sum = 0.0;
    imaginary_sum = 0.0;

    if (capture->frame_count == 0)
    {
        return 0.0;
    }

    if (end_frame >= capture->frame_count)
    {
        end_frame = capture->frame_count - 1;
    }

    for (frame_index = start_frame; frame_index <= end_frame; frame_index += 1)
    {
        f64 sample_count;
        f64 sample;
        f64 window;
        f64 phase;

        sample_count = (f64) (end_frame - start_frame + 1);
        sample = (f64) capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        window = 0.5 - 0.5 * cos(
            (2.0 * 3.14159265358979323846 * (f64) (frame_index - start_frame)) /
            (sample_count - 1.0)
        );
        sample *= window;
        phase = 2.0 * 3.14159265358979323846 * frequency_hz * ((f64) frame_index / sample_rate);

        real_sum += sample * cos(phase);
        imaginary_sum -= sample * sin(phase);
    }

    return sqrt(real_sum * real_sum + imaginary_sum * imaginary_sum);
}

static f64 GetFundamentalFrequencyHz (const Fdtd1DDesc *desc)
{
    ASSERT(desc != NULL);
    ASSERT(desc->tube_length_m > 0.0);

    if ((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN) &&
        (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID))
    {
        return desc->wave_speed_m_per_s / (4.0 * desc->tube_length_m);
    }

    if ((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID) &&
        (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN))
    {
        return desc->wave_speed_m_per_s / (4.0 * desc->tube_length_m);
    }

    return desc->wave_speed_m_per_s / (2.0 * desc->tube_length_m);
}

static void AnalyzeModeSeries (
    const SimulationOfflineCapture *capture,
    const Fdtd1DDesc *desc,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 mode_count,
    ModeResult *results
)
{
    f64 fundamental_frequency_hz;
    u32 mode_index;

    ASSERT(capture != NULL);
    ASSERT(desc != NULL);
    ASSERT(results != NULL);

    fundamental_frequency_hz = GetFundamentalFrequencyHz(desc);

    for (mode_index = 0; mode_index < mode_count; mode_index += 1)
    {
        f64 expected_frequency_hz;
        f64 search_start_frequency_hz;
        f64 search_end_frequency_hz;
        f64 best_frequency_hz;
        f64 best_magnitude;
        f64 search_frequency_hz;

        if (((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN) &&
             (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID)) ||
            ((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID) &&
             (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)))
        {
            expected_frequency_hz = fundamental_frequency_hz * (f64) (2 * (i32) mode_index + 1);
        }
        else
        {
            expected_frequency_hz = fundamental_frequency_hz * (f64) (mode_index + 1);
        }

        search_start_frequency_hz = expected_frequency_hz * 0.90;
        search_end_frequency_hz = expected_frequency_hz * 1.10;
        best_frequency_hz = expected_frequency_hz;
        best_magnitude = 0.0;

        for (search_frequency_hz = search_start_frequency_hz;
             search_frequency_hz <= search_end_frequency_hz;
             search_frequency_hz += 1.0)
        {
            f64 magnitude;

            magnitude = ComputeSpectrumMagnitude(
                capture,
                analysis_channel_index,
                start_frame,
                capture->frame_count - 1,
                (f64) desc->sample_rate,
                search_frequency_hz
            );

            if (magnitude > best_magnitude)
            {
                best_magnitude = magnitude;
                best_frequency_hz = search_frequency_hz;
            }
        }

        results[mode_index].expected_frequency_hz = expected_frequency_hz;
        results[mode_index].measured_peak_frequency_hz = best_frequency_hz;
        results[mode_index].measured_peak_magnitude = best_magnitude;
    }
}

static f64 ComputeStateEnergy (const Fdtd1DState *state)
{
    f64 energy;
    u32 pressure_index;
    u32 velocity_index;

    ASSERT(state != NULL);

    energy = 0.0;

    for (pressure_index = 0; pressure_index < state->pressure_cell_count; pressure_index += 1)
    {
        f64 pressure_value;

        pressure_value = (f64) state->pressure[pressure_index];
        energy += pressure_value * pressure_value;
    }

    for (velocity_index = 0; velocity_index < state->velocity_cell_count; velocity_index += 1)
    {
        f64 velocity_value;

        velocity_value = (f64) state->velocity[velocity_index];
        energy += velocity_value * velocity_value;
    }

    return energy;
}

int main (int argc, char **argv)
{
    static const f64 IMPULSE_AMPLITUDE = 0.25;
    static const u32 NOISE_BURST_FRAME_COUNT = 128;
    static const f32 FIRST_ARRIVAL_THRESHOLD = 0.001f;
    static const u32 WINDOW_RADIUS = 4;

    VerificationSettings settings;
    MemoryArena arena;
    Fdtd1D solver;
    Fdtd1DDesc solver_desc;
    Fdtd1DAreaSegmentDesc area_segment_descs[1];
    Fdtd1DProbeDesc probe_descs[2];
    Fdtd1DSourceDesc source_descs[1];
    Simulation *simulation;
    SimulationExcitation excitation;
    SimulationOfflineCapture capture;
    ArrivalSummary left_arrival;
    ArrivalSummary right_arrival;
    WindowPeakResult left_reflection_window;
    WindowPeakResult right_reflection_window;
    WindowEnergyResult left_reflection_energy;
    WindowEnergyResult right_reflection_energy;
    ModeResult mode_results[MODE_COUNT];
    const SimulationStats *stats;
    const Fdtd1DState *state;
    usize capture_sample_count;
    f64 expected_left_arrival_seconds;
    f64 expected_right_arrival_seconds;
    f64 expected_left_arrival_frames;
    f64 expected_right_arrival_frames;
    f64 expected_left_reflection_frames;
    f64 expected_right_reflection_frames;
    EnergyResult energy;
    u32 block_index;
    int exit_code;

    exit_code = 0;

    InitializeVerificationSettings(&settings);
    ParseArguments(argc, argv, &settings);

    memset(&arena, 0, sizeof(arena));
    memset(&solver, 0, sizeof(solver));
    memset(&solver_desc, 0, sizeof(solver_desc));
    memset(&capture, 0, sizeof(capture));
    memset(&excitation, 0, sizeof(excitation));

    if (MemoryArena_Create(&arena, 16 * 1024 * 1024) == false)
    {
        fprintf(stderr, "Failed to create verification arena.\n");
        return 1;
    }

    ConfigureSolver(&settings, &solver_desc, area_segment_descs, probe_descs, source_descs);

    if (Fdtd1D_Initialize(&solver, &arena, &solver_desc) == false)
    {
        fprintf(stderr, "Failed to initialize 1D FDTD solver.\n");
        MemoryArena_Destroy(&arena);
        return 1;
    }

    simulation = Fdtd1D_GetSimulation(&solver);

    capture.channel_count = solver_desc.output_channel_count;
    capture.frame_capacity = solver_desc.block_frame_count * settings.block_count;
    capture.frame_count = 0;
    capture_sample_count = (usize) capture.frame_capacity * (usize) capture.channel_count;
    capture.samples = MEMORY_ARENA_PUSH_ARRAY(&arena, capture_sample_count, f32);

    if (capture.samples == NULL)
    {
        fprintf(stderr, "Failed to allocate capture buffer.\n");
        Fdtd1D_Shutdown(&solver);
        MemoryArena_Destroy(&arena);
        return 1;
    }

    excitation.type =
        (settings.excitation_type == VERIFICATION_EXCITATION_TYPE_NOISE_BURST) ?
        SIMULATION_EXCITATION_TYPE_NOISE :
        SIMULATION_EXCITATION_TYPE_IMPULSE;
    excitation.target_index = 0;
    excitation.remaining_frame_count =
        (settings.excitation_type == VERIFICATION_EXCITATION_TYPE_NOISE_BURST) ?
        NOISE_BURST_FRAME_COUNT :
        1;
    excitation.value = IMPULSE_AMPLITUDE;
    excitation.is_active = true;

    if (Simulation_QueueExcitation(simulation, &excitation) == false)
    {
        fprintf(stderr, "Failed to queue verification impulse.\n");
        Fdtd1D_Shutdown(&solver);
        MemoryArena_Destroy(&arena);
        return 1;
    }

    energy.initial_energy = 0.0;
    energy.minimum_energy = 0.0;
    energy.maximum_energy = 0.0;
    energy.final_energy = 0.0;

    for (block_index = 0; block_index < settings.block_count; block_index += 1)
    {
        if (Simulation_ProcessBlock(simulation) == false)
        {
            fprintf(stderr, "Offline solver run failed.\n");
            exit_code = 1;
            goto cleanup;
        }

        if (Simulation_CaptureOutput(simulation, &capture) == false)
        {
            fprintf(stderr, "Failed to capture offline output block.\n");
            exit_code = 1;
            goto cleanup;
        }

        state = Fdtd1D_GetState(&solver);
        if (state != NULL)
        {
            f64 block_energy;

            block_energy = ComputeStateEnergy(state);
            if (block_index == 0)
            {
                energy.initial_energy = block_energy;
                energy.minimum_energy = block_energy;
                energy.maximum_energy = block_energy;
            }
            else
            {
                if (block_energy < energy.minimum_energy)
                {
                    energy.minimum_energy = block_energy;
                }

                if (block_energy > energy.maximum_energy)
                {
                    energy.maximum_energy = block_energy;
                }
            }

            energy.final_energy = block_energy;
        }
    }

    stats = Simulation_GetStats(simulation);

    expected_left_arrival_seconds =
        ((f64) (probe_descs[0].cell_index - source_descs[0].cell_index) * solver_desc.dx) /
        solver_desc.wave_speed_m_per_s;
    expected_right_arrival_seconds =
        ((f64) (probe_descs[1].cell_index - source_descs[0].cell_index) * solver_desc.dx) /
        solver_desc.wave_speed_m_per_s;

    expected_left_arrival_frames = expected_left_arrival_seconds * (f64) solver_desc.sample_rate;
    expected_right_arrival_frames = expected_right_arrival_seconds * (f64) solver_desc.sample_rate;
    left_arrival.expected_frame = expected_left_arrival_frames;
    right_arrival.expected_frame = expected_right_arrival_frames;
    left_arrival.first_crossing = AnalyzeCapture(&capture, 0, FIRST_ARRIVAL_THRESHOLD);
    right_arrival.first_crossing = AnalyzeCapture(&capture, 1, FIRST_ARRIVAL_THRESHOLD);
    left_arrival.main_lobe_peak = AnalyzeWindowPeak(
        &capture,
        0,
        (u32) expected_left_arrival_frames - WINDOW_RADIUS,
        (u32) expected_left_arrival_frames + WINDOW_RADIUS
    );
    right_arrival.main_lobe_peak = AnalyzeWindowPeak(
        &capture,
        1,
        (u32) expected_right_arrival_frames - WINDOW_RADIUS,
        (u32) expected_right_arrival_frames + WINDOW_RADIUS
    );
    expected_left_reflection_frames =
        (f64) (source_descs[0].cell_index + probe_descs[0].cell_index);
    expected_right_reflection_frames =
        (f64) (source_descs[0].cell_index + probe_descs[1].cell_index);

    left_reflection_window = AnalyzeWindowPeak(
        &capture,
        0,
        (u32) expected_left_reflection_frames - WINDOW_RADIUS,
        (u32) expected_left_reflection_frames + WINDOW_RADIUS
    );
    right_reflection_window = AnalyzeWindowPeak(
        &capture,
        1,
        (u32) expected_right_reflection_frames - WINDOW_RADIUS,
        (u32) expected_right_reflection_frames + WINDOW_RADIUS
    );
    left_reflection_energy = AnalyzeWindowEnergy(
        &capture,
        0,
        (u32) expected_left_reflection_frames - WINDOW_RADIUS,
        (u32) expected_left_reflection_frames + WINDOW_RADIUS
    );
    right_reflection_energy = AnalyzeWindowEnergy(
        &capture,
        1,
        (u32) expected_right_reflection_frames - WINDOW_RADIUS,
        (u32) expected_right_reflection_frames + WINDOW_RADIUS
    );
    AnalyzeModeSeries(
        &capture,
        &solver_desc,
        0,
        solver_desc.block_frame_count,
        MODE_COUNT,
        mode_results
    );

    printf("FDTD 1D Verification\n");
    printf("\n");

    printf("Setup\n");
    printf("  preset:                 %s\n",
        GetPresetName(settings.preset)
    );
    printf("  sample_rate:            %u\n",
        solver_desc.sample_rate
    );
    printf("  block_frames:           %u\n",
        solver_desc.block_frame_count
    );
    printf("  total_frames:           %u\n",
        capture.frame_count
    );
    printf("  courant:                %.6f\n",
        solver.desc.courant_number
    );
    printf("  left_boundary:          %s\n",
        GetBoundaryTypeName(solver_desc.left_boundary.type)
    );
    printf("  right_boundary:         %s\n",
        GetBoundaryTypeName(solver_desc.right_boundary.type)
    );
    printf("  probe_type:            %s\n",
        GetProbeTypeName(settings.probe_type)
    );
    printf("  excitation:            %s\n",
        GetExcitationTypeName(settings.excitation_type)
    );
    printf("  source_index:          %u\n",
        settings.source_cell_index
    );
    printf("  left_probe_index:      %u\n",
        settings.left_probe_index
    );
    printf("  right_probe_index:     %u\n",
        settings.right_probe_index
    );
    printf("\n");

    printf("Direct Arrivals\n");
    printf("  left expected frame:    %.2f\n",
        left_arrival.expected_frame
    );
    printf("  right expected frame:   %.2f\n",
        right_arrival.expected_frame
    );
    printf("  left first crossing:    ");
    if (left_arrival.first_crossing.first_arrival_was_found)
    {
        printf(
            "frame %u, value %.6f\n",
            left_arrival.first_crossing.first_arrival_frame,
            left_arrival.first_crossing.first_arrival_value
        );
    }
    else
    {
        printf("not found\n");
    }
    printf("  left main-lobe peak:    frame %u, value %.6f, abs %.6f, window [%u, %u]\n",
        left_arrival.main_lobe_peak.peak_frame,
        left_arrival.main_lobe_peak.peak_value,
        left_arrival.main_lobe_peak.peak_abs_value,
        left_arrival.main_lobe_peak.start_frame,
        left_arrival.main_lobe_peak.end_frame
    );
    printf("  right first crossing:   ");
    if (right_arrival.first_crossing.first_arrival_was_found)
    {
        printf(
            "frame %u, value %.6f\n",
            right_arrival.first_crossing.first_arrival_frame,
            right_arrival.first_crossing.first_arrival_value
        );
    }
    else
    {
        printf("not found\n");
    }
    printf("  right main-lobe peak:   frame %u, value %.6f, abs %.6f, window [%u, %u]\n",
        right_arrival.main_lobe_peak.peak_frame,
        right_arrival.main_lobe_peak.peak_value,
        right_arrival.main_lobe_peak.peak_abs_value,
        right_arrival.main_lobe_peak.start_frame,
        right_arrival.main_lobe_peak.end_frame
    );
    printf("\n");

    printf("Reflection Windows\n");
    printf("  left expected frame:    %.2f\n",
        expected_left_reflection_frames
    );
    printf("  left search window:     [%u, %u]\n",
        left_reflection_window.start_frame,
        left_reflection_window.end_frame
    );
    printf("  left window peak:       frame %u, value %.6f, abs %.6f\n",
        left_reflection_window.peak_frame,
        left_reflection_window.peak_value,
        left_reflection_window.peak_abs_value
    );
    printf("  left energy centroid:   %.3f (window energy %.9f)\n",
        left_reflection_energy.energy_centroid_frame,
        left_reflection_energy.energy_sum
    );
    printf("  right expected frame:   %.2f\n",
        expected_right_reflection_frames
    );
    printf("  right search window:    [%u, %u]\n",
        right_reflection_window.start_frame,
        right_reflection_window.end_frame
    );
    printf("  right window peak:      frame %u, value %.6f, abs %.6f\n",
        right_reflection_window.peak_frame,
        right_reflection_window.peak_value,
        right_reflection_window.peak_abs_value
    );
    printf("  right energy centroid:  %.3f (window energy %.9f)\n",
        right_reflection_energy.energy_centroid_frame,
        right_reflection_energy.energy_sum
    );
    printf("\n");

    printf("Runtime Stats\n");
    printf("  processed blocks:       %llu\n",
        (unsigned long long) stats->processed_block_count
    );
    printf("  processed frames:       %llu\n",
        (unsigned long long) stats->processed_frame_count
    );
    printf("  processed seconds:      %.6f\n",
        stats->processed_seconds
    );
    printf("  max output abs:         %.6f\n",
        stats->max_abs_output
    );
    printf("  max probe abs:          %.6f\n",
        stats->max_abs_probe
    );
    printf("  saw NaN:                %s\n",
        stats->saw_nan ? "yes" : "no"
    );
    printf("  saw Inf:                %s\n",
        stats->saw_inf ? "yes" : "no"
    );
    printf("\n");

    printf("Energy\n");
    printf("  initial:                %.9f\n",
        energy.initial_energy
    );
    printf("  minimum:                %.9f\n",
        energy.minimum_energy
    );
    printf("  maximum:                %.9f\n",
        energy.maximum_energy
    );
    printf("  final:                  %.9f\n",
        energy.final_energy
    );
    printf("  drift:                  %.9f\n",
        energy.final_energy - energy.initial_energy
    );
    printf("\n");

    printf("Modal Analysis\n");
    printf("  analysis_channel:       0\n");
    printf("  analysis_start_frame:   %u\n",
        solver_desc.block_frame_count
    );
    for (block_index = 0; block_index < MODE_COUNT; block_index += 1)
    {
        printf("  mode %u:                 expected %.2f Hz, peak %.2f Hz, magnitude %.6f\n",
            block_index + 1,
            mode_results[block_index].expected_frequency_hz,
            mode_results[block_index].measured_peak_frequency_hz,
            mode_results[block_index].measured_peak_magnitude
        );
    }
    printf("\n");

    if (settings.csv_output_path != NULL)
    {
        if (WriteCaptureCsv(settings.csv_output_path, &capture))
        {
            printf("csv_written=%s\n", settings.csv_output_path);
        }
        else
        {
            fprintf(stderr, "Failed to write CSV output: %s\n", settings.csv_output_path);
            exit_code = 1;
        }
    }

cleanup:
    Fdtd1D_Shutdown(&solver);
    MemoryArena_Destroy(&arena);

    return exit_code;
}
