#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/sim/fdtd_1d_render_source.h"

enum
{
    RENDER_SOURCE_PROBE_INDEX_DEBUG_LEFT = 0,
    RENDER_SOURCE_PROBE_INDEX_DEBUG_RIGHT,
    RENDER_SOURCE_PROBE_INDEX_LEFT_MOUTH_PRESSURE,
    RENDER_SOURCE_PROBE_INDEX_LEFT_MOUTH_VELOCITY,
    RENDER_SOURCE_PROBE_INDEX_RIGHT_MOUTH_PRESSURE,
    RENDER_SOURCE_PROBE_INDEX_RIGHT_MOUTH_VELOCITY,
};

bool Fdtd1DRenderSource_Initialize (
    Fdtd1DRenderSource *source,
    MemoryArena *arena,
    const Fdtd1DRenderSourceDesc *desc
)
{
    ASSERT(source != NULL);
    ASSERT(arena != NULL);
    ASSERT(desc != NULL);

    memset(source, 0, sizeof(*source));

    if (Fdtd1D_Initialize(&source->solver, arena, &desc->solver_desc) == false)
    {
        return false;
    }

    source->startup_impulse_is_pending = desc->startup_impulse_is_enabled;
    source->startup_impulse_target_index = desc->startup_impulse_target_index;
    source->startup_impulse_amplitude = desc->startup_impulse_amplitude;
    source->excitation_mode = desc->excitation_mode;
    source->drive_amplitude = desc->drive_amplitude;
    source->output_extraction_mode = desc->output_extraction_mode;

    return true;
}

void Fdtd1DRenderSource_Shutdown (Fdtd1DRenderSource *source)
{
    ASSERT(source != NULL);

    Fdtd1D_Shutdown(&source->solver);
    memset(source, 0, sizeof(*source));
}

void Fdtd1DRenderSource_TriggerStartupImpulse (Fdtd1DRenderSource *source)
{
    ASSERT(source != NULL);

    source->startup_impulse_is_pending = true;
}

void Fdtd1DRenderSource_SetOutputExtractionMode (
    Fdtd1DRenderSource *source,
    Fdtd1DOutputExtractionMode output_extraction_mode
)
{
    ASSERT(source != NULL);

    source->output_extraction_mode = output_extraction_mode;
}

void Fdtd1DRenderSource_SetExcitationMode (
    Fdtd1DRenderSource *source,
    Fdtd1DExcitationMode excitation_mode
)
{
    ASSERT(source != NULL);

    source->excitation_mode = excitation_mode;
}

void Fdtd1DRenderSource_SetDriveAmplitude (Fdtd1DRenderSource *source, f64 drive_amplitude)
{
    ASSERT(source != NULL);

    source->drive_amplitude = drive_amplitude;
}

void Fdtd1DRenderSource_Render (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    const Fdtd1DState *state;
    Fdtd1DRenderSource *source;
    Simulation *simulation;
    SimulationExcitation excitation;
    usize frame_index;
    usize output_sample_count;

    ASSERT(user_data != NULL);
    ASSERT(output != NULL);
    ASSERT(scratch_buffer != NULL);
    ASSERT(block_frame_count > 0);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    source = (Fdtd1DRenderSource *) user_data;
    simulation = Fdtd1D_GetSimulation(&source->solver);
    state = Fdtd1D_GetState(&source->solver);

    ASSERT(simulation->config.block_frame_count == block_frame_count);
    ASSERT(simulation->config.output_channel_count == channel_count);
    ASSERT(simulation->config.sample_rate == sample_rate);
    ASSERT(state != NULL);

    /* Rebuild the excitation set every block so GUI changes take effect immediately. */
    Simulation_ClearExcitations(simulation);

    if (source->startup_impulse_is_pending)
    {
        memset(&excitation, 0, sizeof(excitation));
        excitation.type = SIMULATION_EXCITATION_TYPE_IMPULSE;
        excitation.target_index = source->startup_impulse_target_index;
        excitation.remaining_frame_count = 1;
        excitation.value = source->startup_impulse_amplitude;
        excitation.is_active = true;

        if (Simulation_QueueExcitation(simulation, &excitation))
        {
            source->startup_impulse_is_pending = false;
        }
    }

    if (source->drive_amplitude > 0.0)
    {
        switch (source->excitation_mode)
        {
            case FDTD_1D_EXCITATION_MODE_IMPULSE:
            {
            } break;

            case FDTD_1D_EXCITATION_MODE_CONSTANT:
            case FDTD_1D_EXCITATION_MODE_NOISE:
            {
                memset(&excitation, 0, sizeof(excitation));
                excitation.type = (source->excitation_mode == FDTD_1D_EXCITATION_MODE_CONSTANT) ?
                    SIMULATION_EXCITATION_TYPE_CONSTANT :
                    SIMULATION_EXCITATION_TYPE_NOISE;
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = source->drive_amplitude;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);
            } break;
        }
    }

    output_sample_count = (usize) block_frame_count * (usize) channel_count;
    memset(scratch_buffer, 0, sizeof(f32) * output_sample_count);

    if (Simulation_ProcessBlock(simulation) == false)
    {
        memset(output, 0, sizeof(f32) * output_sample_count);
        return;
    }

    switch (source->output_extraction_mode)
    {
        case FDTD_1D_OUTPUT_EXTRACTION_MODE_RAW_PROBES:
        {
            memcpy(output, simulation->output_buffer, sizeof(f32) * output_sample_count);
        } break;

        case FDTD_1D_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION:
        {
            f32 characteristic_impedance;
            bool left_is_open;
            bool right_is_open;

            ASSERT(simulation->probe_buffer != NULL);
            ASSERT(simulation->config.probe_count >= 6);

            characteristic_impedance = (f32) state->characteristic_impedance;
            left_is_open = (state->left_boundary_type == FDTD_1D_BOUNDARY_TYPE_OPEN);
            right_is_open = (state->right_boundary_type == FDTD_1D_BOUNDARY_TYPE_OPEN);

            for (frame_index = 0; frame_index < block_frame_count; frame_index += 1)
            {
                f32 left_mouth_pressure;
                f32 left_mouth_velocity;
                f32 left_outgoing_wave;
                f32 right_mouth_pressure;
                f32 right_mouth_velocity;
                f32 right_outgoing_wave;
                usize output_offset;
                usize probe_offset;

                probe_offset = frame_index * (usize) simulation->config.probe_count;
                output_offset = frame_index * (usize) channel_count;

                left_mouth_pressure = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_LEFT_MOUTH_PRESSURE];
                left_mouth_velocity = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_LEFT_MOUTH_VELOCITY];
                right_mouth_pressure = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_RIGHT_MOUTH_PRESSURE];
                right_mouth_velocity = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_RIGHT_MOUTH_VELOCITY];

                left_outgoing_wave = left_is_open ?
                    0.5f * (left_mouth_pressure - characteristic_impedance * left_mouth_velocity) :
                    0.0f;
                right_outgoing_wave = right_is_open ?
                    0.5f * (right_mouth_pressure + characteristic_impedance * right_mouth_velocity) :
                    0.0f;

                if (left_is_open && right_is_open)
                {
                    output[output_offset] = left_outgoing_wave;
                    if (channel_count > 1)
                    {
                        output[output_offset + 1] = right_outgoing_wave;
                    }
                }
                else
                {
                    f32 mono_output;

                    mono_output = left_is_open ? left_outgoing_wave : right_outgoing_wave;
                    output[output_offset] = mono_output;
                    if (channel_count > 1)
                    {
                        output[output_offset + 1] = mono_output;
                    }
                }
            }
        } break;
    }
}
