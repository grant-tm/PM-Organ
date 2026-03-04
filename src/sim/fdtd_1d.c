#include <math.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/sim/fdtd_1d.h"

static f64 AbsF64 (f64 value)
{
    return (value < 0.0) ? -value : value;
}

static f64 ComputeCourantNumber (const Fdtd1DDesc *desc)
{
    f64 dt;

    ASSERT(desc != NULL);
    ASSERT(desc->sample_rate > 0);
    ASSERT(desc->dx > 0.0);

    dt = 1.0 / (f64) desc->sample_rate;
    return (desc->wave_speed_m_per_s * dt) / desc->dx;
}

static f32 GetBoundaryReflectionCoefficient (const Fdtd1DBoundaryDesc *boundary)
{
    ASSERT(boundary != NULL);

    switch (boundary->type)
    {
        case FDTD_1D_BOUNDARY_TYPE_RIGID: return 1.0f;
        case FDTD_1D_BOUNDARY_TYPE_OPEN: return -1.0f;

        case FDTD_1D_BOUNDARY_TYPE_REFLECTION_COEFFICIENT:
        {
            return (f32) boundary->reflection_coefficient;
        }
    }

    ASSERT(false);
    return 0.0f;
}

static usize MaxUsize (usize a, usize b)
{
    return (a > b) ? a : b;
}

static void InitializeUniformField (f32 *values, u32 count, f32 value)
{
    u32 index;

    ASSERT(values != NULL);

    for (index = 0; index < count; index += 1)
    {
        values[index] = value;
    }
}

static bool ValidateBoundary (const Fdtd1DBoundaryDesc *boundary)
{
    ASSERT(boundary != NULL);

    switch (boundary->type)
    {
        case FDTD_1D_BOUNDARY_TYPE_RIGID:
        case FDTD_1D_BOUNDARY_TYPE_OPEN:
        {
            return true;
        }

        case FDTD_1D_BOUNDARY_TYPE_REFLECTION_COEFFICIENT:
        {
            return (boundary->reflection_coefficient >= -1.0) &&
                   (boundary->reflection_coefficient <= 1.0);
        }
    }

    return false;
}

static bool ValidateProbeDescs (const Fdtd1DDesc *desc)
{
    u32 probe_index;

    ASSERT(desc != NULL);

    if ((desc->probe_count > 0) && (desc->probe_descs == NULL))
    {
        return false;
    }

    for (probe_index = 0; probe_index < desc->probe_count; probe_index += 1)
    {
        const Fdtd1DProbeDesc *probe_desc;

        probe_desc = &desc->probe_descs[probe_index];
        if (probe_desc->cell_index >= desc->pressure_cell_count)
        {
            return false;
        }

        if (probe_desc->output_channel_index >= desc->output_channel_count)
        {
            return false;
        }
    }

    return true;
}

static bool ValidateSourceDescs (const Fdtd1DDesc *desc)
{
    u32 source_index;

    ASSERT(desc != NULL);

    if ((desc->source_count > 0) && (desc->source_descs == NULL))
    {
        return false;
    }

    for (source_index = 0; source_index < desc->source_count; source_index += 1)
    {
        if (desc->source_descs[source_index].cell_index >= desc->pressure_cell_count)
        {
            return false;
        }
    }

    return true;
}

static void BuildSimulationConfig (const Fdtd1DDesc *desc, SimulationConfig *config)
{
    usize scratch_sample_count;

    ASSERT(desc != NULL);
    ASSERT(config != NULL);

    scratch_sample_count = MaxUsize(
        (usize) desc->block_frame_count * (usize) desc->output_channel_count,
        MaxUsize((usize) desc->pressure_cell_count, (usize) desc->velocity_cell_count)
    );

    config->sample_rate = desc->sample_rate;
    config->block_frame_count = desc->block_frame_count;
    config->output_channel_count = desc->output_channel_count;
    config->probe_count = desc->probe_count;
    config->parameter_count = 0;
    config->excitation_capacity = desc->source_count;
    config->scratch_sample_count = (u32) scratch_sample_count;
    config->model_state_size = 0;
}

static void ResetStateCallback (Simulation *simulation)
{
    Fdtd1DState *state;

    ASSERT(simulation != NULL);

    state = (Fdtd1DState *) simulation->model_state;
    ASSERT(state != NULL);

    memset(state->pressure, 0, sizeof(f32) * (usize) state->pressure_cell_count);
    memset(state->velocity, 0, sizeof(f32) * (usize) state->velocity_cell_count);
}

static void ProcessBlockCallback (Simulation *simulation, SimulationProcessContext *process_context)
{
    Fdtd1DState *state;
    u32 frame_index;
    u32 probe_index;

    ASSERT(simulation != NULL);
    ASSERT(process_context != NULL);

    state = (Fdtd1DState *) simulation->model_state;
    ASSERT(state != NULL);

    for (frame_index = 0; frame_index < process_context->block_frame_count; frame_index += 1)
    {
        for (probe_index = 0; probe_index < state->probe_count; probe_index += 1)
        {
            f32 sample_value;
            u32 cell_index;
            u32 output_channel_index;

            if (process_context->probes[probe_index].is_enabled == false)
            {
                continue;
            }

            cell_index = state->probe_cell_indices[probe_index];
            output_channel_index = state->probe_output_channels[probe_index];
            sample_value = state->pressure[cell_index];

            if (process_context->probe_buffer != NULL)
            {
                process_context->probe_buffer[frame_index * state->probe_count + probe_index] = sample_value;
            }

            process_context->output_buffer[
                frame_index * process_context->output_channel_count + output_channel_index
            ] += sample_value;
        }
    }
}

bool Fdtd1D_ValidateDesc (const Fdtd1DDesc *desc)
{
    f64 computed_courant_number;
    f64 expected_tube_length_m;

    ASSERT(desc != NULL);

    if (desc->sample_rate == 0)
    {
        return false;
    }

    if (desc->block_frame_count == 0)
    {
        return false;
    }

    if (desc->output_channel_count == 0)
    {
        return false;
    }

    if (desc->tube_length_m <= 0.0)
    {
        return false;
    }

    if (desc->wave_speed_m_per_s <= 0.0)
    {
        return false;
    }

    if (desc->density_kg_per_m3 <= 0.0)
    {
        return false;
    }

    if (desc->dx <= 0.0)
    {
        return false;
    }

    if (desc->pressure_cell_count == 0)
    {
        return false;
    }

    if (desc->velocity_cell_count != (desc->pressure_cell_count + 1))
    {
        return false;
    }

    if (desc->uniform_area_m2 <= 0.0)
    {
        return false;
    }

    if (desc->uniform_loss < 0.0)
    {
        return false;
    }

    if (ValidateBoundary(&desc->left_boundary) == false)
    {
        return false;
    }

    if (ValidateBoundary(&desc->right_boundary) == false)
    {
        return false;
    }

    if (ValidateProbeDescs(desc) == false)
    {
        return false;
    }

    if (ValidateSourceDescs(desc) == false)
    {
        return false;
    }

    expected_tube_length_m = (f64) desc->pressure_cell_count * desc->dx;
    if (AbsF64(expected_tube_length_m - desc->tube_length_m) > desc->dx)
    {
        return false;
    }

    computed_courant_number = ComputeCourantNumber(desc);
    if (computed_courant_number > 1.0)
    {
        return false;
    }

    if (desc->courant_number > 0.0)
    {
        if (AbsF64(desc->courant_number - computed_courant_number) > 0.000001)
        {
            return false;
        }
    }

    return true;
}

bool Fdtd1D_Initialize (Fdtd1D *solver, MemoryArena *arena, const Fdtd1DDesc *desc)
{
    SimulationConfig config;
    SimulationInterface simulation_interface;
    Fdtd1DState *state;
    f64 dt;
    f64 pressure_update_coeff;
    f64 velocity_update_coeff;
    u32 probe_index;
    u32 source_index;

    ASSERT(solver != NULL);
    ASSERT(arena != NULL);
    ASSERT(desc != NULL);

    memset(solver, 0, sizeof(*solver));

    if (Fdtd1D_ValidateDesc(desc) == false)
    {
        return false;
    }

    BuildSimulationConfig(desc, &config);

    memset(&simulation_interface, 0, sizeof(simulation_interface));
    simulation_interface.reset_state = ResetStateCallback;
    simulation_interface.process_block = ProcessBlockCallback;

    if (Simulation_Initialize(&solver->simulation, arena, &config, &simulation_interface) == false)
    {
        return false;
    }

    state = MEMORY_ARENA_PUSH_STRUCT(arena, Fdtd1DState);
    if (state == NULL)
    {
        Simulation_Shutdown(&solver->simulation);
        return false;
    }

    memset(state, 0, sizeof(*state));

    state->pressure_cell_count = desc->pressure_cell_count;
    state->velocity_cell_count = desc->velocity_cell_count;
    state->probe_count = desc->probe_count;
    state->source_count = desc->source_count;

    state->dt = 1.0 / (f64) desc->sample_rate;
    state->dx = desc->dx;
    state->wave_speed_m_per_s = desc->wave_speed_m_per_s;
    state->density_kg_per_m3 = desc->density_kg_per_m3;
    state->courant_number = ComputeCourantNumber(desc);

    state->pressure = MEMORY_ARENA_PUSH_ARRAY(arena, desc->pressure_cell_count, f32);
    state->velocity = MEMORY_ARENA_PUSH_ARRAY(arena, desc->velocity_cell_count, f32);
    state->area_pressure = MEMORY_ARENA_PUSH_ARRAY(arena, desc->pressure_cell_count, f32);
    state->area_velocity = MEMORY_ARENA_PUSH_ARRAY(arena, desc->velocity_cell_count, f32);
    state->pressure_loss = MEMORY_ARENA_PUSH_ARRAY(arena, desc->pressure_cell_count, f32);
    state->velocity_loss = MEMORY_ARENA_PUSH_ARRAY(arena, desc->velocity_cell_count, f32);
    state->pressure_update_coeff = MEMORY_ARENA_PUSH_ARRAY(arena, desc->pressure_cell_count, f32);
    state->velocity_update_coeff = MEMORY_ARENA_PUSH_ARRAY(arena, desc->velocity_cell_count, f32);

    if ((state->pressure == NULL) ||
        (state->velocity == NULL) ||
        (state->area_pressure == NULL) ||
        (state->area_velocity == NULL) ||
        (state->pressure_loss == NULL) ||
        (state->velocity_loss == NULL) ||
        (state->pressure_update_coeff == NULL) ||
        (state->velocity_update_coeff == NULL))
    {
        Simulation_Shutdown(&solver->simulation);
        return false;
    }

    if (desc->probe_count > 0)
    {
        state->probe_cell_indices = MEMORY_ARENA_PUSH_ARRAY(arena, desc->probe_count, u32);
        state->probe_output_channels = MEMORY_ARENA_PUSH_ARRAY(arena, desc->probe_count, u32);
        if ((state->probe_cell_indices == NULL) || (state->probe_output_channels == NULL))
        {
            Simulation_Shutdown(&solver->simulation);
            return false;
        }
    }

    if (desc->source_count > 0)
    {
        state->source_cell_indices = MEMORY_ARENA_PUSH_ARRAY(arena, desc->source_count, u32);
        if (state->source_cell_indices == NULL)
        {
            Simulation_Shutdown(&solver->simulation);
            return false;
        }
    }

    dt = state->dt;
    velocity_update_coeff = dt / (desc->density_kg_per_m3 * desc->dx);
    pressure_update_coeff =
        (desc->density_kg_per_m3 * desc->wave_speed_m_per_s * desc->wave_speed_m_per_s * dt) / desc->dx;

    InitializeUniformField(state->area_pressure, desc->pressure_cell_count, (f32) desc->uniform_area_m2);
    InitializeUniformField(state->area_velocity, desc->velocity_cell_count, (f32) desc->uniform_area_m2);
    InitializeUniformField(state->pressure_loss, desc->pressure_cell_count, (f32) desc->uniform_loss);
    InitializeUniformField(state->velocity_loss, desc->velocity_cell_count, (f32) desc->uniform_loss);
    InitializeUniformField(state->pressure_update_coeff, desc->pressure_cell_count, (f32) pressure_update_coeff);
    InitializeUniformField(state->velocity_update_coeff, desc->velocity_cell_count, (f32) velocity_update_coeff);

    for (probe_index = 0; probe_index < desc->probe_count; probe_index += 1)
    {
        SimulationProbe probe;

        state->probe_cell_indices[probe_index] = desc->probe_descs[probe_index].cell_index;
        state->probe_output_channels[probe_index] = desc->probe_descs[probe_index].output_channel_index;

        probe.read_index = desc->probe_descs[probe_index].cell_index;
        probe.output_channel_index = desc->probe_descs[probe_index].output_channel_index;
        probe.is_enabled = desc->probe_descs[probe_index].is_enabled;

        if (Simulation_SetProbe(&solver->simulation, probe_index, &probe) == false)
        {
            Simulation_Shutdown(&solver->simulation);
            return false;
        }
    }

    for (source_index = 0; source_index < desc->source_count; source_index += 1)
    {
        state->source_cell_indices[source_index] = desc->source_descs[source_index].cell_index;
    }

    state->left_reflection_coefficient = GetBoundaryReflectionCoefficient(&desc->left_boundary);
    state->right_reflection_coefficient = GetBoundaryReflectionCoefficient(&desc->right_boundary);

    solver->desc = *desc;
    solver->desc.courant_number = state->courant_number;
    solver->state = state;
    solver->simulation.model_state = state;

    Simulation_Reset(&solver->simulation);

    return true;
}

void Fdtd1D_Shutdown (Fdtd1D *solver)
{
    ASSERT(solver != NULL);

    Simulation_Shutdown(&solver->simulation);
    memset(solver, 0, sizeof(*solver));
}

void Fdtd1D_Reset (Fdtd1D *solver)
{
    ASSERT(solver != NULL);
    ASSERT(solver->simulation.is_initialized == true);

    Simulation_Reset(&solver->simulation);
}

Simulation *Fdtd1D_GetSimulation (Fdtd1D *solver)
{
    ASSERT(solver != NULL);

    return &solver->simulation;
}

const Fdtd1DState *Fdtd1D_GetState (const Fdtd1D *solver)
{
    ASSERT(solver != NULL);

    return solver->state;
}
