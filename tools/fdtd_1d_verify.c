#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/core/memory_arena.h"
#include "pm_organ/sim/fdtd_1d.h"

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

typedef struct EnergyResult
{
    f64 initial_energy;
    f64 minimum_energy;
    f64 maximum_energy;
    f64 final_energy;
} EnergyResult;

static void ConfigureSolver (
    Fdtd1DDesc *desc,
    Fdtd1DProbeDesc *probe_descs,
    Fdtd1DSourceDesc *source_descs
)
{
    static const f64 TEST_COURANT_NUMBER = 0.9;

    ASSERT(desc != NULL);
    ASSERT(probe_descs != NULL);
    ASSERT(source_descs != NULL);

    probe_descs[0].cell_index = 96;
    probe_descs[0].output_channel_index = 0;
    probe_descs[0].is_enabled = true;

    probe_descs[1].cell_index = 112;
    probe_descs[1].output_channel_index = 1;
    probe_descs[1].is_enabled = true;

    source_descs[0].cell_index = 16;
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
    desc->uniform_loss = 0.0;
    desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
    desc->left_boundary.reflection_coefficient = 1.0;
    desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
    desc->right_boundary.reflection_coefficient = 1.0;
    desc->probe_count = 2;
    desc->probe_descs = probe_descs;
    desc->source_count = 1;
    desc->source_descs = source_descs;
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
    static const u32 BLOCK_COUNT = 256;
    static const f64 IMPULSE_AMPLITUDE = 0.25;
    static const f32 FIRST_ARRIVAL_THRESHOLD = 0.001f;
    static const u32 WINDOW_RADIUS = 4;

    const char *csv_output_path;
    MemoryArena arena;
    Fdtd1D solver;
    Fdtd1DDesc solver_desc;
    Fdtd1DProbeDesc probe_descs[2];
    Fdtd1DSourceDesc source_descs[1];
    Simulation *simulation;
    SimulationExcitation excitation;
    SimulationOfflineCapture capture;
    VerificationResult left_result;
    VerificationResult right_result;
    WindowPeakResult left_reflection_window;
    WindowPeakResult right_reflection_window;
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

    csv_output_path = (argc >= 2) ? argv[1] : NULL;
    exit_code = 0;

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

    ConfigureSolver(&solver_desc, probe_descs, source_descs);

    if (Fdtd1D_Initialize(&solver, &arena, &solver_desc) == false)
    {
        fprintf(stderr, "Failed to initialize 1D FDTD solver.\n");
        MemoryArena_Destroy(&arena);
        return 1;
    }

    simulation = Fdtd1D_GetSimulation(&solver);

    capture.channel_count = solver_desc.output_channel_count;
    capture.frame_capacity = solver_desc.block_frame_count * BLOCK_COUNT;
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

    excitation.type = SIMULATION_EXCITATION_TYPE_IMPULSE;
    excitation.target_index = 0;
    excitation.remaining_frame_count = 1;
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

    for (block_index = 0; block_index < BLOCK_COUNT; block_index += 1)
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

    left_result = AnalyzeCapture(&capture, 0, FIRST_ARRIVAL_THRESHOLD);
    right_result = AnalyzeCapture(&capture, 1, FIRST_ARRIVAL_THRESHOLD);
    stats = Simulation_GetStats(simulation);

    expected_left_arrival_seconds =
        ((f64) (probe_descs[0].cell_index - source_descs[0].cell_index) * solver_desc.dx) /
        solver_desc.wave_speed_m_per_s;
    expected_right_arrival_seconds =
        ((f64) (probe_descs[1].cell_index - source_descs[0].cell_index) * solver_desc.dx) /
        solver_desc.wave_speed_m_per_s;

    expected_left_arrival_frames = expected_left_arrival_seconds * (f64) solver_desc.sample_rate;
    expected_right_arrival_frames = expected_right_arrival_seconds * (f64) solver_desc.sample_rate;
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

    printf("FDTD 1D Verification\n");
    printf("\n");

    printf("Setup\n");
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
    printf("\n");

    printf("Direct Arrivals\n");
    printf("  left expected frame:    %.2f\n",
        expected_left_arrival_frames
    );
    printf("  right expected frame:   %.2f\n",
        expected_right_arrival_frames
    );
    printf("  left first arrival:     ");
    if (left_result.first_arrival_was_found)
    {
        printf("frame %u, value %.6f\n", left_result.first_arrival_frame, left_result.first_arrival_value);
    }
    else
    {
        printf("not found\n");
    }
    printf("  right first arrival:    ");
    if (right_result.first_arrival_was_found)
    {
        printf("frame %u, value %.6f\n", right_result.first_arrival_frame, right_result.first_arrival_value);
    }
    else
    {
        printf("not found\n");
    }
    printf("  left peak:              frame %u, abs %.6f\n",
        left_result.peak_frame,
        left_result.peak_abs_sample
    );
    printf("  right peak:             frame %u, abs %.6f\n",
        right_result.peak_frame,
        right_result.peak_abs_sample
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

    if (csv_output_path != NULL)
    {
        if (WriteCaptureCsv(csv_output_path, &capture))
        {
            printf("csv_written=%s\n", csv_output_path);
        }
        else
        {
            fprintf(stderr, "Failed to write CSV output: %s\n", csv_output_path);
            exit_code = 1;
        }
    }

cleanup:
    Fdtd1D_Shutdown(&solver);
    MemoryArena_Destroy(&arena);

    return exit_code;
}
