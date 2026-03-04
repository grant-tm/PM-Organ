#include <math.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/sim/simulation.h"

static const usize MODEL_STATE_ALIGNMENT = 16;

static void ClearBuffers (Simulation *simulation)
{
    usize output_sample_count;
    usize probe_sample_count;

    ASSERT(simulation != NULL);

    output_sample_count = (usize) simulation->config.block_frame_count * (usize) simulation->config.output_channel_count;
    memset(simulation->output_buffer, 0, sizeof(f32) * output_sample_count);
    memset(simulation->scratch_buffer, 0, sizeof(f32) * (usize) simulation->config.scratch_sample_count);

    if ((simulation->probe_buffer != NULL) && (simulation->config.probe_count > 0))
    {
        probe_sample_count = (usize) simulation->config.block_frame_count * (usize) simulation->config.probe_count;
        memset(simulation->probe_buffer, 0, sizeof(f32) * probe_sample_count);
    }
}

static void UpdatePeakAndFiniteState (f32 sample, f32 *peak_value, bool *saw_nan, bool *saw_inf)
{
    f32 abs_value;

    ASSERT(peak_value != NULL);
    ASSERT(saw_nan != NULL);
    ASSERT(saw_inf != NULL);

    if (isnan(sample))
    {
        *saw_nan = true;
        return;
    }

    if (isinf(sample))
    {
        *saw_inf = true;
        return;
    }

    abs_value = fabsf(sample);
    if (abs_value > *peak_value)
    {
        *peak_value = abs_value;
    }
}

static void UpdateStats (Simulation *simulation)
{
    usize output_sample_count;
    usize probe_sample_count;
    usize sample_index;

    ASSERT(simulation != NULL);

    output_sample_count = (usize) simulation->config.block_frame_count * (usize) simulation->config.output_channel_count;
    for (sample_index = 0; sample_index < output_sample_count; sample_index += 1)
    {
        UpdatePeakAndFiniteState(
            simulation->output_buffer[sample_index],
            &simulation->stats.max_abs_output,
            &simulation->stats.saw_nan,
            &simulation->stats.saw_inf
        );
    }

    if ((simulation->probe_buffer != NULL) && (simulation->config.probe_count > 0))
    {
        probe_sample_count = (usize) simulation->config.block_frame_count * (usize) simulation->config.probe_count;
        for (sample_index = 0; sample_index < probe_sample_count; sample_index += 1)
        {
            UpdatePeakAndFiniteState(
                simulation->probe_buffer[sample_index],
                &simulation->stats.max_abs_probe,
                &simulation->stats.saw_nan,
                &simulation->stats.saw_inf
            );
        }
    }

    simulation->stats.processed_block_count += 1;
    simulation->stats.processed_frame_count += simulation->config.block_frame_count;
    simulation->stats.processed_seconds =
        (f64) simulation->stats.processed_frame_count / (f64) simulation->config.sample_rate;
}

bool Simulation_ValidateConfig (const SimulationConfig *config)
{
    ASSERT(config != NULL);

    if (config->sample_rate == 0)
    {
        return false;
    }

    if (config->block_frame_count == 0)
    {
        return false;
    }

    if (config->output_channel_count == 0)
    {
        return false;
    }

    if (config->scratch_sample_count == 0)
    {
        return false;
    }

    return true;
}

bool Simulation_Initialize (
    Simulation *simulation,
    MemoryArena *arena,
    const SimulationConfig *config,
    const SimulationInterface *simulation_interface
)
{
    usize output_sample_count;
    usize probe_sample_count;

    ASSERT(simulation != NULL);
    ASSERT(arena != NULL);
    ASSERT(config != NULL);
    ASSERT(simulation_interface != NULL);
    ASSERT(simulation_interface->process_block != NULL);

    memset(simulation, 0, sizeof(*simulation));

    if (Simulation_ValidateConfig(config) == false)
    {
        return false;
    }

    if ((simulation_interface->validate_config != NULL) &&
        (simulation_interface->validate_config(config) == false))
    {
        return false;
    }

    simulation->config = *config;
    simulation->interface = *simulation_interface;

    output_sample_count = (usize) config->block_frame_count * (usize) config->output_channel_count;
    simulation->output_buffer = MEMORY_ARENA_PUSH_ARRAY(arena, output_sample_count, f32);
    simulation->scratch_buffer = MEMORY_ARENA_PUSH_ARRAY(arena, config->scratch_sample_count, f32);

    if ((simulation->output_buffer == NULL) || (simulation->scratch_buffer == NULL))
    {
        return false;
    }

    if (config->probe_count > 0)
    {
        probe_sample_count = (usize) config->block_frame_count * (usize) config->probe_count;
        simulation->probe_buffer = MEMORY_ARENA_PUSH_ARRAY(arena, probe_sample_count, f32);
        simulation->probes = MEMORY_ARENA_PUSH_ARRAY(arena, config->probe_count, SimulationProbe);

        if ((simulation->probe_buffer == NULL) || (simulation->probes == NULL))
        {
            return false;
        }
    }

    if (config->parameter_count > 0)
    {
        simulation->parameters = MEMORY_ARENA_PUSH_ARRAY(arena, config->parameter_count, SimulationParameter);
        if (simulation->parameters == NULL)
        {
            return false;
        }
    }

    if (config->excitation_capacity > 0)
    {
        simulation->excitations = MEMORY_ARENA_PUSH_ARRAY(arena, config->excitation_capacity, SimulationExcitation);
        if (simulation->excitations == NULL)
        {
            return false;
        }
    }

    if (config->model_state_size > 0)
    {
        simulation->model_state = MemoryArena_PushZeroSize(arena, config->model_state_size, MODEL_STATE_ALIGNMENT);
        if (simulation->model_state == NULL)
        {
            return false;
        }
    }

    simulation->is_initialized = true;
    Simulation_Reset(simulation);

    if ((simulation->interface.initialize_state != NULL) &&
        (simulation->interface.initialize_state(simulation) == false))
    {
        simulation->is_initialized = false;
        return false;
    }

    return true;
}

void Simulation_Shutdown (Simulation *simulation)
{
    ASSERT(simulation != NULL);

    memset(simulation, 0, sizeof(*simulation));
}

void Simulation_Reset (Simulation *simulation)
{
    u32 parameter_index;

    ASSERT(simulation != NULL);
    ASSERT(simulation->is_initialized == true);

    ClearBuffers(simulation);
    Simulation_ClearExcitations(simulation);
    Simulation_ResetStats(simulation);

    for (parameter_index = 0; parameter_index < simulation->config.parameter_count; parameter_index += 1)
    {
        simulation->parameters[parameter_index].current_value = simulation->parameters[parameter_index].default_value;
        simulation->parameters[parameter_index].is_dirty = false;
    }

    if (simulation->interface.reset_state != NULL)
    {
        simulation->interface.reset_state(simulation);
    }
}

bool Simulation_SetProbe (Simulation *simulation, u32 probe_index, const SimulationProbe *probe)
{
    ASSERT(simulation != NULL);
    ASSERT(probe != NULL);
    ASSERT(simulation->is_initialized == true);

    if (probe_index >= simulation->config.probe_count)
    {
        return false;
    }

    simulation->probes[probe_index] = *probe;
    return true;
}

bool Simulation_SetParameter (Simulation *simulation, u32 parameter_index, const SimulationParameter *parameter)
{
    ASSERT(simulation != NULL);
    ASSERT(parameter != NULL);
    ASSERT(simulation->is_initialized == true);

    if (parameter_index >= simulation->config.parameter_count)
    {
        return false;
    }

    if (parameter->minimum_value > parameter->maximum_value)
    {
        return false;
    }

    if ((parameter->current_value < parameter->minimum_value) ||
        (parameter->current_value > parameter->maximum_value))
    {
        return false;
    }

    simulation->parameters[parameter_index] = *parameter;
    simulation->parameters[parameter_index].is_dirty = true;
    return true;
}

bool Simulation_QueueExcitation (Simulation *simulation, const SimulationExcitation *excitation)
{
    u32 excitation_index;

    ASSERT(simulation != NULL);
    ASSERT(excitation != NULL);
    ASSERT(simulation->is_initialized == true);

    if (simulation->config.excitation_capacity == 0)
    {
        return false;
    }

    for (excitation_index = 0; excitation_index < simulation->config.excitation_capacity; excitation_index += 1)
    {
        if (simulation->excitations[excitation_index].is_active == false)
        {
            simulation->excitations[excitation_index] = *excitation;
            simulation->excitations[excitation_index].is_active = true;

            if (excitation_index >= simulation->excitation_count)
            {
                simulation->excitation_count = excitation_index + 1;
            }

            return true;
        }
    }

    return false;
}

void Simulation_ClearExcitations (Simulation *simulation)
{
    ASSERT(simulation != NULL);

    if (simulation->excitations != NULL)
    {
        memset(
            simulation->excitations,
            0,
            sizeof(SimulationExcitation) * (usize) simulation->config.excitation_capacity
        );
    }

    simulation->excitation_count = 0;
}

bool Simulation_ProcessBlock (Simulation *simulation)
{
    SimulationProcessContext process_context;

    ASSERT(simulation != NULL);
    ASSERT(simulation->is_initialized == true);
    ASSERT(simulation->interface.process_block != NULL);

    ClearBuffers(simulation);

    process_context.output_buffer = simulation->output_buffer;
    process_context.scratch_buffer = simulation->scratch_buffer;
    process_context.probe_buffer = simulation->probe_buffer;
    process_context.probes = simulation->probes;
    process_context.parameters = simulation->parameters;
    process_context.excitations = simulation->excitations;
    process_context.block_frame_count = simulation->config.block_frame_count;
    process_context.output_channel_count = simulation->config.output_channel_count;
    process_context.probe_count = simulation->config.probe_count;
    process_context.parameter_count = simulation->config.parameter_count;
    process_context.excitation_count = simulation->excitation_count;
    process_context.sample_rate = simulation->config.sample_rate;

    simulation->interface.process_block(simulation, &process_context);
    UpdateStats(simulation);

    return (simulation->stats.saw_nan == false) && (simulation->stats.saw_inf == false);
}

bool Simulation_CaptureOutput (Simulation *simulation, SimulationOfflineCapture *capture)
{
    usize output_sample_count;
    usize destination_sample_offset;

    ASSERT(simulation != NULL);
    ASSERT(capture != NULL);
    ASSERT(simulation->is_initialized == true);

    if (capture->samples == NULL)
    {
        return false;
    }

    if (capture->channel_count != simulation->config.output_channel_count)
    {
        return false;
    }

    if ((capture->frame_count + simulation->config.block_frame_count) > capture->frame_capacity)
    {
        return false;
    }

    output_sample_count = (usize) simulation->config.block_frame_count * (usize) simulation->config.output_channel_count;
    destination_sample_offset = (usize) capture->frame_count * (usize) capture->channel_count;

    memcpy(
        capture->samples + destination_sample_offset,
        simulation->output_buffer,
        sizeof(f32) * output_sample_count
    );

    capture->frame_count += simulation->config.block_frame_count;
    return true;
}

bool Simulation_RunOfflineBlocks (Simulation *simulation, u32 block_count, SimulationOfflineCapture *capture)
{
    u32 block_index;

    ASSERT(simulation != NULL);
    ASSERT(simulation->is_initialized == true);

    for (block_index = 0; block_index < block_count; block_index += 1)
    {
        if (Simulation_ProcessBlock(simulation) == false)
        {
            return false;
        }

        if ((capture != NULL) && (Simulation_CaptureOutput(simulation, capture) == false))
        {
            return false;
        }
    }

    return true;
}

void Simulation_ResetStats (Simulation *simulation)
{
    ASSERT(simulation != NULL);

    memset(&simulation->stats, 0, sizeof(simulation->stats));
}

const SimulationStats *Simulation_GetStats (const Simulation *simulation)
{
    ASSERT(simulation != NULL);

    return &simulation->stats;
}
