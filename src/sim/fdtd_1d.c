#include <math.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/sim/fdtd_1d.h"

static const f64 FDTD_1D_PI = 3.14159265358979323846;
static const f64 OPEN_END_CORRECTION_COEFFICIENT = 0.61;
static const f64 OPEN_END_RADIATION_RESISTANCE_SCALE = 1.0;

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
        switch (probe_desc->type)
        {
            case FDTD_1D_PROBE_TYPE_PRESSURE:
            {
                if (probe_desc->cell_index >= desc->pressure_cell_count)
                {
                    return false;
                }
            } break;

            case FDTD_1D_PROBE_TYPE_VELOCITY:
            {
                if (probe_desc->cell_index >= desc->velocity_cell_count)
                {
                    return false;
                }
            } break;

            default:
            {
                return false;
            } break;
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
    state->left_previous_outgoing_pressure = 0.0f;
    state->left_previous_incoming_pressure = 0.0f;
    state->right_previous_outgoing_pressure = 0.0f;
    state->right_previous_incoming_pressure = 0.0f;
    state->noise_state = 0x13572468u;
}

static f32 NextNoiseSample (Fdtd1DState *state)
{
    ASSERT(state != NULL);

    state->noise_state = state->noise_state * 1664525u + 1013904223u;
    return ((f32) ((state->noise_state >> 8) & 0x00FFFFFFu) / 8388607.5f) - 1.0f;
}

static void InjectDistributedPressureImpulse (Fdtd1DState *state, u32 cell_index, f32 amplitude)
{
    ASSERT(state != NULL);
    ASSERT(cell_index < state->pressure_cell_count);

    if ((cell_index > 0) && (cell_index + 1 < state->pressure_cell_count))
    {
        state->pressure[cell_index - 1] += 0.25f * amplitude;
        state->pressure[cell_index] += 0.50f * amplitude;
        state->pressure[cell_index + 1] += 0.25f * amplitude;
        return;
    }

    state->pressure[cell_index] += amplitude;
}

static f32 FilterOpenBoundaryReflection (
    const Fdtd1DState *state,
    f32 outgoing_pressure,
    f32 *previous_outgoing_pressure,
    f32 *previous_incoming_pressure
)
{
    f32 incoming_pressure;

    ASSERT(state != NULL);
    ASSERT(previous_outgoing_pressure != NULL);
    ASSERT(previous_incoming_pressure != NULL);

    incoming_pressure =
        -state->open_reflection_filter_a1 * (*previous_incoming_pressure) +
        state->open_reflection_filter_b0 * outgoing_pressure +
        state->open_reflection_filter_b1 * (*previous_outgoing_pressure);

    *previous_outgoing_pressure = outgoing_pressure;
    *previous_incoming_pressure = incoming_pressure;

    return incoming_pressure;
}

static void UpdateVelocityField (Fdtd1DState *state)
{
    f32 characteristic_impedance;
    u32 velocity_index;

    ASSERT(state != NULL);
    ASSERT(state->pressure_cell_count > 0);
    ASSERT(state->velocity_cell_count == (state->pressure_cell_count + 1));

    characteristic_impedance = (f32) state->characteristic_impedance;

    for (velocity_index = 1; velocity_index < (state->velocity_cell_count - 1); velocity_index += 1)
    {
        state->velocity[velocity_index] -= state->velocity_update_coeff[velocity_index] *
            (state->pressure[velocity_index] - state->pressure[velocity_index - 1]);
        state->velocity[velocity_index] *= (1.0f - state->velocity_loss[velocity_index]);
    }

    switch (state->left_boundary_type)
    {
        case FDTD_1D_BOUNDARY_TYPE_RIGID:
        {
            state->velocity[0] = 0.0f;
        } break;

        case FDTD_1D_BOUNDARY_TYPE_OPEN:
        case FDTD_1D_BOUNDARY_TYPE_REFLECTION_COEFFICIENT:
        {
            f32 incoming_pressure;
            f32 outgoing_pressure;

            outgoing_pressure = 0.5f * (state->pressure[0] - characteristic_impedance * state->velocity[0]);
            if (state->left_boundary_type == FDTD_1D_BOUNDARY_TYPE_OPEN)
            {
                incoming_pressure = FilterOpenBoundaryReflection(
                    state,
                    outgoing_pressure,
                    &state->left_previous_outgoing_pressure,
                    &state->left_previous_incoming_pressure
                );
            }
            else
            {
                incoming_pressure = state->left_reflection_coefficient * outgoing_pressure;
            }

            state->velocity[0] = (incoming_pressure - outgoing_pressure) / characteristic_impedance;
            state->velocity[0] *= (1.0f - state->velocity_loss[0]);
        } break;
    }

    switch (state->right_boundary_type)
    {
        case FDTD_1D_BOUNDARY_TYPE_RIGID:
        {
            state->velocity[state->velocity_cell_count - 1] = 0.0f;
        } break;

        case FDTD_1D_BOUNDARY_TYPE_OPEN:
        case FDTD_1D_BOUNDARY_TYPE_REFLECTION_COEFFICIENT:
        {
            f32 incoming_pressure;
            f32 outgoing_pressure;

            outgoing_pressure = 0.5f * (
                state->pressure[state->pressure_cell_count - 1] +
                characteristic_impedance * state->velocity[state->velocity_cell_count - 1]
            );
            if (state->right_boundary_type == FDTD_1D_BOUNDARY_TYPE_OPEN)
            {
                incoming_pressure = FilterOpenBoundaryReflection(
                    state,
                    outgoing_pressure,
                    &state->right_previous_outgoing_pressure,
                    &state->right_previous_incoming_pressure
                );
            }
            else
            {
                incoming_pressure = state->right_reflection_coefficient * outgoing_pressure;
            }

            state->velocity[state->velocity_cell_count - 1] =
                (outgoing_pressure - incoming_pressure) / characteristic_impedance;
            state->velocity[state->velocity_cell_count - 1] *=
                (1.0f - state->velocity_loss[state->velocity_cell_count - 1]);
        } break;
    }
}

static void UpdatePressureField (Fdtd1DState *state)
{
    u32 pressure_index;

    ASSERT(state != NULL);

    for (pressure_index = 0; pressure_index < state->pressure_cell_count; pressure_index += 1)
    {
        state->pressure[pressure_index] -= state->pressure_update_coeff[pressure_index] *
            (state->velocity[pressure_index + 1] - state->velocity[pressure_index]);
        state->pressure[pressure_index] *= (1.0f - state->pressure_loss[pressure_index]);
    }
}

static void ApplyExcitations (Simulation *simulation, SimulationProcessContext *process_context, Fdtd1DState *state)
{
    u32 excitation_index;

    ASSERT(simulation != NULL);
    ASSERT(process_context != NULL);
    ASSERT(state != NULL);

    for (excitation_index = 0; excitation_index < simulation->excitation_count; excitation_index += 1)
    {
        SimulationExcitation *excitation;
        f32 excitation_sample;
        u32 source_index;
        u32 cell_index;

        excitation = &process_context->excitations[excitation_index];
        if (excitation->is_active == false)
        {
            continue;
        }

        if (excitation->target_index >= state->source_count)
        {
            excitation->is_active = false;
            continue;
        }

        source_index = excitation->target_index;
        cell_index = state->source_cell_indices[source_index];
        excitation_sample = 0.0f;

        switch (excitation->type)
        {
            case SIMULATION_EXCITATION_TYPE_NONE:
            {
            } break;

            case SIMULATION_EXCITATION_TYPE_IMPULSE:
            {
                excitation_sample = (f32) excitation->value;
                excitation->remaining_frame_count = 0;
            } break;

            case SIMULATION_EXCITATION_TYPE_CONSTANT:
            {
                excitation_sample = (f32) excitation->value;
            } break;

            case SIMULATION_EXCITATION_TYPE_NOISE:
            {
                excitation_sample = (f32) excitation->value * NextNoiseSample(state);
            } break;

            case SIMULATION_EXCITATION_TYPE_CUSTOM:
            {
            } break;
        }

        if (excitation->type == SIMULATION_EXCITATION_TYPE_IMPULSE)
        {
            InjectDistributedPressureImpulse(state, cell_index, excitation_sample);
        }
        else
        {
            state->pressure[cell_index] += excitation_sample;
        }

        if (excitation->remaining_frame_count > 0)
        {
            excitation->remaining_frame_count -= 1;
        }

        if ((excitation->type == SIMULATION_EXCITATION_TYPE_IMPULSE) ||
            ((excitation->remaining_frame_count == 0) &&
             (excitation->type != SIMULATION_EXCITATION_TYPE_CONSTANT) &&
             (excitation->type != SIMULATION_EXCITATION_TYPE_CUSTOM)))
        {
            excitation->is_active = false;
        }
    }
}

static void ReadProbes (SimulationProcessContext *process_context, const Fdtd1DState *state, u32 frame_index)
{
    u32 probe_index;

    ASSERT(process_context != NULL);
    ASSERT(state != NULL);

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
        switch (state->probe_types[probe_index])
        {
            case FDTD_1D_PROBE_TYPE_PRESSURE:
            {
                sample_value = state->pressure[cell_index];
            } break;

            case FDTD_1D_PROBE_TYPE_VELOCITY:
            {
                sample_value = state->velocity[cell_index];
            } break;

            default:
            {
                ASSERT(false);
                sample_value = 0.0f;
            } break;
        }

        if (process_context->probe_buffer != NULL)
        {
            process_context->probe_buffer[frame_index * state->probe_count + probe_index] = sample_value;
        }

        process_context->output_buffer[
            frame_index * process_context->output_channel_count + output_channel_index
        ] += sample_value;
    }
}

static void CompactExcitations (Simulation *simulation)
{
    while (simulation->excitation_count > 0)
    {
        if (simulation->excitations[simulation->excitation_count - 1].is_active)
        {
            break;
        }

        simulation->excitation_count -= 1;
    }
}

static void ProcessBlockCallback (Simulation *simulation, SimulationProcessContext *process_context)
{
    Fdtd1DState *state;
    u32 frame_index;

    ASSERT(simulation != NULL);
    ASSERT(process_context != NULL);

    state = (Fdtd1DState *) simulation->model_state;
    ASSERT(state != NULL);

    for (frame_index = 0; frame_index < process_context->block_frame_count; frame_index += 1)
    {
        UpdateVelocityField(state);
        UpdatePressureField(state);
        ApplyExcitations(simulation, process_context, state);
        ReadProbes(process_context, state, frame_index);
    }

    CompactExcitations(simulation);
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

    if ((desc->uniform_loss < 0.0) || (desc->uniform_loss > 1.0))
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
    f64 bilinear_scale;
    f64 denominator_0;
    f64 denominator_1;
    f64 dt;
    f64 end_correction_m;
    f64 pipe_radius_m;
    f64 pressure_update_coeff;
    f64 radiation_inertance;
    f64 radiation_resistance;
    f64 reflection_denominator_constant;
    f64 reflection_denominator_slope;
    f64 reflection_numerator_constant;
    f64 reflection_numerator_slope;
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
    state->characteristic_impedance = desc->density_kg_per_m3 * desc->wave_speed_m_per_s;

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
        state->probe_types = MEMORY_ARENA_PUSH_ARRAY(arena, desc->probe_count, Fdtd1DProbeType);
        state->probe_cell_indices = MEMORY_ARENA_PUSH_ARRAY(arena, desc->probe_count, u32);
        state->probe_output_channels = MEMORY_ARENA_PUSH_ARRAY(arena, desc->probe_count, u32);
        if ((state->probe_types == NULL) ||
            (state->probe_cell_indices == NULL) ||
            (state->probe_output_channels == NULL))
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

        state->probe_types[probe_index] = desc->probe_descs[probe_index].type;
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

    state->left_boundary_type = desc->left_boundary.type;
    state->right_boundary_type = desc->right_boundary.type;
    state->left_reflection_coefficient = GetBoundaryReflectionCoefficient(&desc->left_boundary);
    state->right_reflection_coefficient = GetBoundaryReflectionCoefficient(&desc->right_boundary);

    pipe_radius_m = sqrt(desc->uniform_area_m2 / FDTD_1D_PI);
    end_correction_m = OPEN_END_CORRECTION_COEFFICIENT * pipe_radius_m;
    radiation_inertance = desc->density_kg_per_m3 * end_correction_m;
    radiation_resistance = OPEN_END_RADIATION_RESISTANCE_SCALE * state->characteristic_impedance;
    bilinear_scale = 2.0 / state->dt;

    reflection_numerator_slope =
        (radiation_resistance - state->characteristic_impedance) * radiation_inertance;
    reflection_numerator_constant = -state->characteristic_impedance * radiation_resistance;
    reflection_denominator_slope =
        (radiation_resistance + state->characteristic_impedance) * radiation_inertance;
    reflection_denominator_constant = state->characteristic_impedance * radiation_resistance;

    denominator_0 = reflection_denominator_slope * bilinear_scale + reflection_denominator_constant;
    denominator_1 = -reflection_denominator_slope * bilinear_scale + reflection_denominator_constant;

    state->open_reflection_filter_a1 = (f32) (denominator_1 / denominator_0);
    state->open_reflection_filter_b0 =
        (f32) ((reflection_numerator_slope * bilinear_scale + reflection_numerator_constant) / denominator_0);
    state->open_reflection_filter_b1 =
        (f32) ((-reflection_numerator_slope * bilinear_scale + reflection_numerator_constant) / denominator_0);

    if (desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)
    {
        state->left_reflection_coefficient = -1.0f;
    }

    if (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)
    {
        state->right_reflection_coefficient = -1.0f;
    }

    solver->desc = *desc;
    solver->desc.courant_number = state->courant_number;
    solver->state = state;
    solver->simulation.model_state = state;
    solver->simulation.interface.reset_state = ResetStateCallback;

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
