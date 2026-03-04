#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/sim/fdtd_1d_render_source.h"

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

void Fdtd1DRenderSource_Render (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
)
{
    Fdtd1DRenderSource *source;
    Simulation *simulation;
    SimulationExcitation excitation;
    usize output_sample_count;

    ASSERT(user_data != NULL);
    ASSERT(output != NULL);
    ASSERT(scratch_buffer != NULL);
    ASSERT(block_frame_count > 0);
    ASSERT(channel_count > 0);
    ASSERT(sample_rate > 0);

    source = (Fdtd1DRenderSource *) user_data;
    simulation = Fdtd1D_GetSimulation(&source->solver);

    ASSERT(simulation->config.block_frame_count == block_frame_count);
    ASSERT(simulation->config.output_channel_count == channel_count);
    ASSERT(simulation->config.sample_rate == sample_rate);

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

    output_sample_count = (usize) block_frame_count * (usize) channel_count;
    memset(scratch_buffer, 0, sizeof(f32) * output_sample_count);

    if (Simulation_ProcessBlock(simulation) == false)
    {
        memset(output, 0, sizeof(f32) * output_sample_count);
        return;
    }

    memcpy(output, simulation->output_buffer, sizeof(f32) * output_sample_count);
}
