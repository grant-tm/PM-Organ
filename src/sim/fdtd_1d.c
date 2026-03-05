#include <math.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/sim/fdtd_1d.h"

static const f64 FDTD_1D_PI = 3.14159265358979323846;
static const f64 DEFAULT_OPEN_END_CORRECTION_COEFFICIENT = 0.45;
static const f64 DEFAULT_OPEN_END_RADIATION_RESISTANCE_SCALE = 1.5;
static const u32 NONLINEAR_MOUTH_MAX_DELAY_SAMPLES = 64;
static const f32 JET_LABIUM_INJECTION_MAX = 0.01f;
static const Fdtd1DNonlinearMouthParameters DEFAULT_NONLINEAR_MOUTH_PARAMETERS =
{
    0.0002f, /* max_output */
    0.04f,   /* noise_scale */
    0.35f,   /* pressure_feedback */
    0.10f,   /* velocity_feedback */
    0.60f,   /* feedback_leak */
    80.0f,   /* saturation_gain */
    0.0005f, /* drive_limit */
    8u       /* delay_samples */
};
static const Fdtd1DJetLabiumParameters DEFAULT_JET_LABIUM_PARAMETERS =
{
    0.0020f,  /* max_output */
    0.15f,    /* noise_scale */
    0.16f,    /* pressure_feedback */
    0.05f,    /* velocity_feedback */
    0.35f,    /* feedback_leak */
    0.30f,    /* jet_smoothing */
    1.9f,     /* labium_split_gain */
    12.0f,    /* saturation_gain */
    0.0022f,  /* drive_limit */
    9u        /* delay_samples */
};

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

static bool ValidateAreaSegmentDescs (const Fdtd1DDesc *desc)
{
    u32 segment_index;

    ASSERT(desc != NULL);

    if ((desc->area_segment_count > 0) && (desc->area_segment_descs == NULL))
    {
        return false;
    }

    for (segment_index = 0; segment_index < desc->area_segment_count; segment_index += 1)
    {
        const Fdtd1DAreaSegmentDesc *segment_desc;

        segment_desc = &desc->area_segment_descs[segment_index];
        if (segment_desc->start_cell_index >= segment_desc->end_cell_index)
        {
            return false;
        }

        if (segment_desc->end_cell_index > desc->pressure_cell_count)
        {
            return false;
        }

        if (segment_desc->area_m2 <= 0.0)
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
    usize delay_sample_count;

    ASSERT(simulation != NULL);

    state = (Fdtd1DState *) simulation->model_state;
    ASSERT(state != NULL);

    memset(state->pressure, 0, sizeof(f32) * (usize) state->pressure_cell_count);
    memset(state->velocity, 0, sizeof(f32) * (usize) state->velocity_cell_count);
    if (state->pressure_previous != NULL)
    {
        memset(state->pressure_previous, 0, sizeof(f32) * (usize) state->pressure_cell_count);
    }
    if (state->velocity_previous != NULL)
    {
        memset(state->velocity_previous, 0, sizeof(f32) * (usize) state->velocity_cell_count);
    }
    if ((state->source_jet_state != NULL) && (state->source_count > 0))
    {
        memset(state->source_jet_state, 0, sizeof(f32) * (usize) state->source_count);
    }
    state->left_previous_outgoing_pressure = 0.0f;
    state->left_previous_incoming_pressure = 0.0f;
    state->right_previous_outgoing_pressure = 0.0f;
    state->right_previous_incoming_pressure = 0.0f;
    state->noise_state = 0x13572468u;

    if ((state->mouth_feedback_delay_buffer != NULL) && (state->source_count > 0))
    {
        delay_sample_count = (usize) state->source_count * (usize) state->mouth_feedback_delay_capacity;
        memset(state->mouth_feedback_delay_buffer, 0, sizeof(f32) * delay_sample_count);
        memset(state->mouth_feedback_delay_indices, 0, sizeof(u32) * (usize) state->source_count);
    }
}

static f32 NextNoiseSample (Fdtd1DState *state)
{
    ASSERT(state != NULL);

    state->noise_state = state->noise_state * 1664525u + 1013904223u;
    return ((f32) ((state->noise_state >> 8) & 0x00FFFFFFu) / 8388607.5f) - 1.0f;
}

static f32 ClampF32 (f32 value, f32 min_value, f32 max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static f32 ApplyFrequencyDependentLoss (
    f32 current_sample,
    f32 *previous_sample,
    f32 high_frequency_loss
)
{
    f32 delta_sample;

    ASSERT(previous_sample != NULL);

    if (high_frequency_loss <= 0.0f)
    {
        *previous_sample = current_sample;
        return current_sample;
    }

    delta_sample = current_sample - (*previous_sample);
    current_sample -= high_frequency_loss * delta_sample;
    *previous_sample = current_sample;
    return current_sample;
}

static bool ValidateNonlinearMouthParameters (const Fdtd1DNonlinearMouthParameters *parameters)
{
    ASSERT(parameters != NULL);

    if (parameters->max_output <= 0.0f)
    {
        return false;
    }

    if (parameters->noise_scale < 0.0f)
    {
        return false;
    }

    if (parameters->pressure_feedback < 0.0f)
    {
        return false;
    }

    if (parameters->velocity_feedback < 0.0f)
    {
        return false;
    }

    if ((parameters->feedback_leak < 0.0f) || (parameters->feedback_leak >= 1.0f))
    {
        return false;
    }

    if (parameters->saturation_gain <= 0.0f)
    {
        return false;
    }

    if (parameters->drive_limit <= 0.0f)
    {
        return false;
    }

    if ((parameters->delay_samples == 0) || (parameters->delay_samples > NONLINEAR_MOUTH_MAX_DELAY_SAMPLES))
    {
        return false;
    }

    return true;
}

static bool ValidateJetLabiumParameters (const Fdtd1DJetLabiumParameters *parameters)
{
    ASSERT(parameters != NULL);

    if (parameters->max_output <= 0.0f)
    {
        return false;
    }

    if (parameters->noise_scale < 0.0f)
    {
        return false;
    }

    if (parameters->pressure_feedback < 0.0f)
    {
        return false;
    }

    if (parameters->velocity_feedback < 0.0f)
    {
        return false;
    }

    if ((parameters->feedback_leak < 0.0f) || (parameters->feedback_leak >= 1.0f))
    {
        return false;
    }

    if ((parameters->jet_smoothing < 0.0f) || (parameters->jet_smoothing > 1.0f))
    {
        return false;
    }

    if (parameters->labium_split_gain <= 0.0f)
    {
        return false;
    }

    if (parameters->saturation_gain <= 0.0f)
    {
        return false;
    }

    if (parameters->drive_limit <= 0.0f)
    {
        return false;
    }

    if ((parameters->delay_samples == 0) || (parameters->delay_samples > NONLINEAR_MOUTH_MAX_DELAY_SAMPLES))
    {
        return false;
    }

    return true;
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

static void InjectVelocityExcitation (Fdtd1DState *state, u32 cell_index, f32 amplitude)
{
    u32 velocity_index;

    ASSERT(state != NULL);
    ASSERT(cell_index < state->pressure_cell_count);

    velocity_index = cell_index;
    if (velocity_index >= state->velocity_cell_count)
    {
        velocity_index = state->velocity_cell_count - 1;
    }

    state->velocity[velocity_index] += amplitude;
}

static void ResetSourceDelayState (Fdtd1DState *state, u32 source_index, u32 delay_samples)
{
    f32 *delay_buffer;
    usize buffer_offset;

    ASSERT(state != NULL);
    ASSERT(source_index < state->source_count);
    ASSERT(delay_samples > 0);
    ASSERT(delay_samples <= state->mouth_feedback_delay_capacity);
    ASSERT(state->mouth_feedback_delay_lengths != NULL);
    ASSERT(state->mouth_feedback_delay_indices != NULL);
    ASSERT(state->mouth_feedback_delay_buffer != NULL);

    state->mouth_feedback_delay_lengths[source_index] = delay_samples;
    state->mouth_feedback_delay_indices[source_index] = 0;

    buffer_offset = (usize) source_index * (usize) state->mouth_feedback_delay_capacity;
    delay_buffer = state->mouth_feedback_delay_buffer + buffer_offset;
    memset(delay_buffer, 0, sizeof(f32) * (usize) state->mouth_feedback_delay_capacity);
}

static f32 ComputeNonlinearMouthExcitation (
    Fdtd1DState *state,
    u32 source_index,
    u32 cell_index,
    f32 wind_drive
)
{
    f32 delayed_feedback;
    f32 feedback_signal;
    f32 local_pressure;
    f32 local_velocity;
    f32 mouth_input;
    f32 *delay_buffer;
    u32 delay_index;
    u32 velocity_index;

    ASSERT(state != NULL);
    ASSERT(source_index < state->source_count);
    ASSERT(cell_index < state->pressure_cell_count);
    ASSERT(state->mouth_feedback_delay_buffer != NULL);
    ASSERT(state->mouth_feedback_delay_lengths != NULL);
    ASSERT(state->mouth_feedback_delay_indices != NULL);

    if (state->mouth_feedback_delay_lengths[source_index] != state->nonlinear_mouth.delay_samples)
    {
        ResetSourceDelayState(state, source_index, state->nonlinear_mouth.delay_samples);
    }

    velocity_index = cell_index;
    if (velocity_index >= state->velocity_cell_count)
    {
        velocity_index = state->velocity_cell_count - 1;
    }

    local_pressure = state->pressure[cell_index];
    local_velocity = state->velocity[velocity_index];
    feedback_signal =
        (state->nonlinear_mouth.pressure_feedback * local_pressure) +
        (state->nonlinear_mouth.velocity_feedback * (f32) state->characteristic_impedance * local_velocity);

    delay_buffer = state->mouth_feedback_delay_buffer +
        ((usize) source_index * (usize) state->mouth_feedback_delay_capacity);
    delay_index = state->mouth_feedback_delay_indices[source_index];
    delayed_feedback = delay_buffer[delay_index];
    delay_buffer[delay_index] =
        feedback_signal + (state->nonlinear_mouth.feedback_leak * delayed_feedback);
    state->mouth_feedback_delay_indices[source_index] =
        (delay_index + 1) % state->mouth_feedback_delay_lengths[source_index];

    mouth_input =
        wind_drive +
        (wind_drive * state->nonlinear_mouth.noise_scale * NextNoiseSample(state)) -
        ClampF32(delayed_feedback, -4.0f * state->nonlinear_mouth.drive_limit, 4.0f * state->nonlinear_mouth.drive_limit);

    return state->nonlinear_mouth.max_output * tanhf(state->nonlinear_mouth.saturation_gain * mouth_input);
}

static f32 ComputeJetLabiumExcitation (
    Fdtd1DState *state,
    u32 source_index,
    u32 cell_index,
    f32 wind_drive
)
{
    static const u32 MIN_CONVECTION_DELAY_SAMPLES = 2;
    static const f32 JET_LABIUM_BACKPRESSURE_COUPLING = 0.35f;
    static const f32 JET_LABIUM_BACKPRESSURE_LIMIT_SCALE = 6.0f;
    f32 delayed_feedback;
    f32 backpressure_term;
    f32 feedback_term;
    f32 feedback_signal;
    f32 drive_reference;
    f32 available_wind_drive;
    f32 normalized_wind_drive;
    f32 jet_drive;
    f32 jet_error;
    f32 jet_noise;
    f32 jet_state;
    f32 labium_split;
    f32 local_pressure;
    f32 local_velocity;
    f32 mouth_input;
    f32 output_sample;
    f32 soft_wind_drive;
    f32 drive_norm;
    f32 turbulence;
    f32 turbulence_weight;
    f32 *delay_buffer;
    u32 delay_index;
    u32 dynamic_delay_max;
    u32 dynamic_delay_min;
    u32 dynamic_delay_range;
    u32 target_delay_samples;
    u32 velocity_index;

    ASSERT(state != NULL);
    ASSERT(source_index < state->source_count);
    ASSERT(cell_index < state->pressure_cell_count);
    ASSERT(state->source_jet_state != NULL);
    ASSERT(state->mouth_feedback_delay_buffer != NULL);
    ASSERT(state->mouth_feedback_delay_lengths != NULL);
    ASSERT(state->mouth_feedback_delay_indices != NULL);

    drive_reference = state->jet_labium.drive_limit;
    if (drive_reference <= 0.0f)
    {
        drive_reference = 0.000001f;
    }

    wind_drive = ClampF32(wind_drive, 0.0f, 0.1f);
    soft_wind_drive = drive_reference * tanhf(wind_drive / drive_reference);
    normalized_wind_drive = (2.0f / (f32) FDTD_1D_PI) * atanf(wind_drive / drive_reference);
    if (normalized_wind_drive < 0.0f)
    {
        normalized_wind_drive = 0.0f;
    }
    else if (normalized_wind_drive > 1.0f)
    {
        normalized_wind_drive = 1.0f;
    }

    dynamic_delay_min = state->jet_labium.delay_samples / 2;
    if (dynamic_delay_min < MIN_CONVECTION_DELAY_SAMPLES)
    {
        dynamic_delay_min = MIN_CONVECTION_DELAY_SAMPLES;
    }

    dynamic_delay_max = state->jet_labium.delay_samples + (state->jet_labium.delay_samples / 2);
    if (dynamic_delay_max > state->mouth_feedback_delay_capacity)
    {
        dynamic_delay_max = state->mouth_feedback_delay_capacity;
    }

    if (dynamic_delay_max < dynamic_delay_min)
    {
        dynamic_delay_max = dynamic_delay_min;
    }

    dynamic_delay_range = dynamic_delay_max - dynamic_delay_min;
    target_delay_samples = dynamic_delay_max -
        (u32) ((f32) dynamic_delay_range * normalized_wind_drive + 0.5f);

    if (target_delay_samples < dynamic_delay_min)
    {
        target_delay_samples = dynamic_delay_min;
    }
    else if (target_delay_samples > dynamic_delay_max)
    {
        target_delay_samples = dynamic_delay_max;
    }

    if (state->mouth_feedback_delay_lengths[source_index] != target_delay_samples)
    {
        ResetSourceDelayState(state, source_index, target_delay_samples);
    }

    velocity_index = cell_index;
    if (velocity_index >= state->velocity_cell_count)
    {
        velocity_index = state->velocity_cell_count - 1;
    }

    local_pressure = state->pressure[cell_index];
    local_velocity = state->velocity[velocity_index];
    feedback_signal =
        (state->jet_labium.pressure_feedback * local_pressure) +
        (state->jet_labium.velocity_feedback * (f32) state->characteristic_impedance * local_velocity);

    delay_buffer = state->mouth_feedback_delay_buffer +
        ((usize) source_index * (usize) state->mouth_feedback_delay_capacity);
    delay_index = state->mouth_feedback_delay_indices[source_index];
    delayed_feedback = delay_buffer[delay_index];
    delay_buffer[delay_index] = feedback_signal + (state->jet_labium.feedback_leak * delayed_feedback);
    state->mouth_feedback_delay_indices[source_index] =
        (delay_index + 1) % state->mouth_feedback_delay_lengths[source_index];

    feedback_term = ClampF32(
        delayed_feedback,
        -4.0f * drive_reference,
        4.0f * drive_reference
    );
    backpressure_term = ClampF32(
        local_pressure,
        -JET_LABIUM_BACKPRESSURE_LIMIT_SCALE * drive_reference,
        JET_LABIUM_BACKPRESSURE_LIMIT_SCALE * drive_reference
    );
    available_wind_drive = soft_wind_drive - (JET_LABIUM_BACKPRESSURE_COUPLING * backpressure_term);
    available_wind_drive = ClampF32(available_wind_drive, 0.0f, 0.1f);

    jet_noise = available_wind_drive * state->jet_labium.noise_scale * NextNoiseSample(state);
    jet_drive = available_wind_drive + jet_noise - feedback_term;
    jet_state = state->source_jet_state[source_index];
    jet_state += state->jet_labium.jet_smoothing * (jet_drive - jet_state);
    state->source_jet_state[source_index] = jet_state;

    jet_error = jet_state - (0.5f * feedback_term);
    labium_split = tanhf(state->jet_labium.labium_split_gain * jet_error);
    drive_norm = state->jet_labium.drive_limit;
    if (drive_norm <= 0.0f)
    {
        drive_norm = 0.000001f;
    }
    turbulence_weight = ClampF32(fabsf(jet_state) / drive_norm, 0.0f, 1.0f);
    turbulence =
        (0.15f + (0.85f * turbulence_weight)) *
        state->jet_labium.noise_scale *
        wind_drive *
        NextNoiseSample(state);
    mouth_input = (jet_error * labium_split) + turbulence;
    output_sample = state->jet_labium.max_output * tanhf(state->jet_labium.saturation_gain * mouth_input);

    return ClampF32(output_sample, -state->jet_labium.max_output, state->jet_labium.max_output);
}

static f32 FilterOpenBoundaryReflection (
    f32 filter_a1,
    f32 filter_b0,
    f32 filter_b1,
    f32 outgoing_pressure,
    f32 *previous_outgoing_pressure,
    f32 *previous_incoming_pressure
)
{
    f32 incoming_pressure;

    ASSERT(previous_outgoing_pressure != NULL);
    ASSERT(previous_incoming_pressure != NULL);

    incoming_pressure =
        -filter_a1 * (*previous_incoming_pressure) +
        filter_b0 * outgoing_pressure +
        filter_b1 * (*previous_outgoing_pressure);

    *previous_outgoing_pressure = outgoing_pressure;
    *previous_incoming_pressure = incoming_pressure;

    return incoming_pressure;
}

static void InitializeAreaFields (Fdtd1DState *state, const Fdtd1DDesc *desc)
{
    u32 cell_index;
    u32 segment_index;

    ASSERT(state != NULL);
    ASSERT(desc != NULL);
    ASSERT(state->pressure_cell_count == desc->pressure_cell_count);
    ASSERT(state->velocity_cell_count == desc->velocity_cell_count);

    InitializeUniformField(state->area_pressure, desc->pressure_cell_count, (f32) desc->uniform_area_m2);

    for (segment_index = 0; segment_index < desc->area_segment_count; segment_index += 1)
    {
        const Fdtd1DAreaSegmentDesc *segment_desc;

        segment_desc = &desc->area_segment_descs[segment_index];
        for (cell_index = segment_desc->start_cell_index; cell_index < segment_desc->end_cell_index; cell_index += 1)
        {
            state->area_pressure[cell_index] = (f32) segment_desc->area_m2;
        }
    }

    state->area_velocity[0] = state->area_pressure[0];
    for (cell_index = 1; cell_index < (state->velocity_cell_count - 1); cell_index += 1)
    {
        state->area_velocity[cell_index] =
            0.5f * (state->area_pressure[cell_index - 1] + state->area_pressure[cell_index]);
    }
    state->area_velocity[state->velocity_cell_count - 1] =
        state->area_pressure[state->pressure_cell_count - 1];
}

static f32 ComputeAreaLossScale (
    f32 reference_area_m2,
    f32 local_area_m2,
    f32 area_loss_strength
)
{
    f32 area_ratio;
    f32 loss_scale;

    if (area_loss_strength <= 0.0f)
    {
        return 1.0f;
    }

    if (local_area_m2 <= 0.0f)
    {
        return 1.0f;
    }

    area_ratio = reference_area_m2 / local_area_m2;
    loss_scale = powf(area_ratio, area_loss_strength);
    return ClampF32(loss_scale, 0.25f, 4.0f);
}

static void InitializeGeometryAwareLossFields (Fdtd1DState *state, const Fdtd1DDesc *desc)
{
    f32 area_loss_strength;
    f32 reference_area_m2;
    u32 pressure_index;
    u32 velocity_index;

    ASSERT(state != NULL);
    ASSERT(desc != NULL);
    ASSERT(state->pressure_loss != NULL);
    ASSERT(state->velocity_loss != NULL);
    ASSERT(state->area_pressure != NULL);
    ASSERT(state->area_velocity != NULL);

    reference_area_m2 = (f32) desc->area_loss_reference_m2;
    area_loss_strength = (f32) desc->area_loss_strength;

    for (pressure_index = 0; pressure_index < state->pressure_cell_count; pressure_index += 1)
    {
        f32 loss_scale;

        loss_scale = ComputeAreaLossScale(
            reference_area_m2,
            state->area_pressure[pressure_index],
            area_loss_strength
        );
        state->pressure_loss[pressure_index] = ClampF32(
            (f32) desc->uniform_loss * loss_scale,
            0.0f,
            1.0f
        );
    }

    for (velocity_index = 0; velocity_index < state->velocity_cell_count; velocity_index += 1)
    {
        f32 loss_scale;

        loss_scale = ComputeAreaLossScale(
            reference_area_m2,
            state->area_velocity[velocity_index],
            area_loss_strength
        );
        state->velocity_loss[velocity_index] = ClampF32(
            (f32) desc->uniform_loss * loss_scale,
            0.0f,
            1.0f
        );
    }
}

static void InitializePressureUpdateCoefficients (Fdtd1DState *state)
{
    f64 pressure_update_numerator;
    u32 pressure_index;

    ASSERT(state != NULL);

    pressure_update_numerator =
        state->density_kg_per_m3 * state->wave_speed_m_per_s * state->wave_speed_m_per_s * state->dt;

    for (pressure_index = 0; pressure_index < state->pressure_cell_count; pressure_index += 1)
    {
        state->pressure_update_coeff[pressure_index] = (f32) (
            pressure_update_numerator / (state->dx * (f64) state->area_pressure[pressure_index])
        );
    }
}

static void InitializeOpenBoundaryFilter (
    const Fdtd1DState *state,
    f32 boundary_area_m2,
    f64 open_end_correction_coefficient,
    f64 open_end_radiation_resistance_scale,
    f32 *filter_a1,
    f32 *filter_b0,
    f32 *filter_b1
)
{
    f64 bilinear_scale;
    f64 denominator_0;
    f64 denominator_1;
    f64 end_correction_m;
    f64 pipe_radius_m;
    f64 radiation_inertance;
    f64 radiation_resistance;
    f64 reflection_denominator_constant;
    f64 reflection_denominator_slope;
    f64 reflection_numerator_constant;
    f64 reflection_numerator_slope;

    ASSERT(state != NULL);
    ASSERT(filter_a1 != NULL);
    ASSERT(filter_b0 != NULL);
    ASSERT(filter_b1 != NULL);
    ASSERT(boundary_area_m2 > 0.0f);

    pipe_radius_m = sqrt((f64) boundary_area_m2 / FDTD_1D_PI);
    end_correction_m = open_end_correction_coefficient * pipe_radius_m;
    radiation_inertance = state->density_kg_per_m3 * end_correction_m;
    radiation_resistance = open_end_radiation_resistance_scale * state->characteristic_impedance;
    bilinear_scale = 2.0 / state->dt;

    reflection_numerator_slope =
        (radiation_resistance - state->characteristic_impedance) * radiation_inertance;
    reflection_numerator_constant = -state->characteristic_impedance * radiation_resistance;
    reflection_denominator_slope =
        (radiation_resistance + state->characteristic_impedance) * radiation_inertance;
    reflection_denominator_constant = state->characteristic_impedance * radiation_resistance;

    denominator_0 = reflection_denominator_slope * bilinear_scale + reflection_denominator_constant;
    denominator_1 = -reflection_denominator_slope * bilinear_scale + reflection_denominator_constant;

    *filter_a1 = (f32) (denominator_1 / denominator_0);
    *filter_b0 =
        (f32) ((reflection_numerator_slope * bilinear_scale + reflection_numerator_constant) / denominator_0);
    *filter_b1 =
        (f32) ((-reflection_numerator_slope * bilinear_scale + reflection_numerator_constant) / denominator_0);
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
        state->velocity[velocity_index] = ApplyFrequencyDependentLoss(
            state->velocity[velocity_index],
            &state->velocity_previous[velocity_index],
            state->velocity_high_frequency_loss
        );
    }

    switch (state->left_boundary_type)
    {
        case FDTD_1D_BOUNDARY_TYPE_RIGID:
        {
            state->velocity[0] = 0.0f;
            state->velocity_previous[0] = 0.0f;
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
                    state->left_open_reflection_filter_a1,
                    state->left_open_reflection_filter_b0,
                    state->left_open_reflection_filter_b1,
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
            state->velocity[0] *= (1.0f - state->boundary_loss);
            state->velocity[0] = ApplyFrequencyDependentLoss(
                state->velocity[0],
                &state->velocity_previous[0],
                state->boundary_high_frequency_loss
            );
        } break;
    }

    switch (state->right_boundary_type)
    {
        case FDTD_1D_BOUNDARY_TYPE_RIGID:
        {
            state->velocity[state->velocity_cell_count - 1] = 0.0f;
            state->velocity_previous[state->velocity_cell_count - 1] = 0.0f;
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
                    state->right_open_reflection_filter_a1,
                    state->right_open_reflection_filter_b0,
                    state->right_open_reflection_filter_b1,
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
                (1.0f - state->boundary_loss);
            state->velocity[state->velocity_cell_count - 1] = ApplyFrequencyDependentLoss(
                state->velocity[state->velocity_cell_count - 1],
                &state->velocity_previous[state->velocity_cell_count - 1],
                state->boundary_high_frequency_loss
            );
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
            ((state->area_velocity[pressure_index + 1] * state->velocity[pressure_index + 1]) -
             (state->area_velocity[pressure_index] * state->velocity[pressure_index]));
        state->pressure[pressure_index] *= (1.0f - state->pressure_loss[pressure_index]);
        state->pressure[pressure_index] = ApplyFrequencyDependentLoss(
            state->pressure[pressure_index],
            &state->pressure_previous[pressure_index],
            state->pressure_high_frequency_loss
        );
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

            case SIMULATION_EXCITATION_TYPE_VELOCITY_IMPULSE:
            {
                excitation_sample = (f32) excitation->value;
                excitation->remaining_frame_count = 0;
            } break;

            case SIMULATION_EXCITATION_TYPE_VELOCITY_CONSTANT:
            {
                excitation_sample = (f32) excitation->value;
            } break;

            case SIMULATION_EXCITATION_TYPE_VELOCITY_NOISE:
            {
                excitation_sample = (f32) excitation->value * NextNoiseSample(state);
            } break;

            case SIMULATION_EXCITATION_TYPE_NONLINEAR_MOUTH:
            {
                excitation_sample = ComputeNonlinearMouthExcitation(
                    state,
                    source_index,
                    cell_index,
                    ClampF32((f32) excitation->value, 0.0f, state->nonlinear_mouth.drive_limit)
                );
            } break;

            case SIMULATION_EXCITATION_TYPE_JET_LABIUM:
            {
                excitation_sample = ComputeJetLabiumExcitation(
                    state,
                    source_index,
                    cell_index,
                    ClampF32((f32) excitation->value, 0.0f, 0.1f)
                );
                excitation_sample = ClampF32(excitation_sample, -JET_LABIUM_INJECTION_MAX, JET_LABIUM_INJECTION_MAX);
            } break;

            case SIMULATION_EXCITATION_TYPE_CUSTOM:
            {
            } break;
        }

        if (excitation->type == SIMULATION_EXCITATION_TYPE_IMPULSE)
        {
            InjectDistributedPressureImpulse(state, cell_index, excitation_sample);
        }
        else if (excitation->type == SIMULATION_EXCITATION_TYPE_VELOCITY_IMPULSE)
        {
            InjectVelocityExcitation(state, cell_index, excitation_sample);
        }
        else if ((excitation->type == SIMULATION_EXCITATION_TYPE_VELOCITY_CONSTANT) ||
                 (excitation->type == SIMULATION_EXCITATION_TYPE_VELOCITY_NOISE) ||
                 (excitation->type == SIMULATION_EXCITATION_TYPE_NONLINEAR_MOUTH) ||
                 (excitation->type == SIMULATION_EXCITATION_TYPE_JET_LABIUM))
        {
            InjectVelocityExcitation(state, cell_index, excitation_sample);
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
            (excitation->type == SIMULATION_EXCITATION_TYPE_VELOCITY_IMPULSE) ||
            ((excitation->remaining_frame_count == 0) &&
             (excitation->type != SIMULATION_EXCITATION_TYPE_CONSTANT) &&
             (excitation->type != SIMULATION_EXCITATION_TYPE_VELOCITY_CONSTANT) &&
             (excitation->type != SIMULATION_EXCITATION_TYPE_NONLINEAR_MOUTH) &&
             (excitation->type != SIMULATION_EXCITATION_TYPE_JET_LABIUM) &&
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

    if ((desc->uniform_high_frequency_loss < 0.0) || (desc->uniform_high_frequency_loss > 1.0))
    {
        return false;
    }

    if ((desc->uniform_boundary_loss < 0.0) || (desc->uniform_boundary_loss > 1.0))
    {
        return false;
    }

    if ((desc->uniform_boundary_high_frequency_loss < 0.0) ||
        (desc->uniform_boundary_high_frequency_loss > 1.0))
    {
        return false;
    }

    if (desc->area_loss_reference_m2 <= 0.0)
    {
        return false;
    }

    if ((desc->area_loss_strength < 0.0) || (desc->area_loss_strength > 2.0))
    {
        return false;
    }

    if (desc->open_end_correction_coefficient < 0.0)
    {
        return false;
    }

    if (desc->open_end_radiation_resistance_scale < 0.0)
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

    if (ValidateAreaSegmentDescs(desc) == false)
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
    f64 open_end_correction_coefficient;
    f64 open_end_radiation_resistance_scale;
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
    state->pressure_previous = MEMORY_ARENA_PUSH_ARRAY(arena, desc->pressure_cell_count, f32);
    state->velocity_previous = MEMORY_ARENA_PUSH_ARRAY(arena, desc->velocity_cell_count, f32);
    state->pressure_update_coeff = MEMORY_ARENA_PUSH_ARRAY(arena, desc->pressure_cell_count, f32);
    state->velocity_update_coeff = MEMORY_ARENA_PUSH_ARRAY(arena, desc->velocity_cell_count, f32);

    if ((state->pressure == NULL) ||
        (state->velocity == NULL) ||
        (state->area_pressure == NULL) ||
        (state->area_velocity == NULL) ||
        (state->pressure_loss == NULL) ||
        (state->velocity_loss == NULL) ||
        (state->pressure_previous == NULL) ||
        (state->velocity_previous == NULL) ||
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
        state->source_jet_state = MEMORY_ARENA_PUSH_ARRAY(arena, desc->source_count, f32);
        state->mouth_feedback_delay_lengths = MEMORY_ARENA_PUSH_ARRAY(arena, desc->source_count, u32);
        state->mouth_feedback_delay_indices = MEMORY_ARENA_PUSH_ARRAY(arena, desc->source_count, u32);
        state->mouth_feedback_delay_capacity = NONLINEAR_MOUTH_MAX_DELAY_SAMPLES;
        state->mouth_feedback_delay_buffer = MEMORY_ARENA_PUSH_ARRAY(
            arena,
            (usize) desc->source_count * (usize) state->mouth_feedback_delay_capacity,
            f32
        );
        if ((state->source_cell_indices == NULL) ||
            (state->source_jet_state == NULL) ||
            (state->mouth_feedback_delay_lengths == NULL) ||
            (state->mouth_feedback_delay_indices == NULL) ||
            (state->mouth_feedback_delay_buffer == NULL))
        {
            Simulation_Shutdown(&solver->simulation);
            return false;
        }
    }

    dt = state->dt;
    velocity_update_coeff = dt / (desc->density_kg_per_m3 * desc->dx);
    InitializeAreaFields(state, desc);
    InitializeGeometryAwareLossFields(state, desc);
    state->pressure_high_frequency_loss = (f32) desc->uniform_high_frequency_loss;
    state->velocity_high_frequency_loss = (f32) desc->uniform_high_frequency_loss;
    state->boundary_loss = (f32) desc->uniform_boundary_loss;
    state->boundary_high_frequency_loss = (f32) desc->uniform_boundary_high_frequency_loss;
    open_end_correction_coefficient = desc->open_end_correction_coefficient;
    if (open_end_correction_coefficient <= 0.0)
    {
        open_end_correction_coefficient = DEFAULT_OPEN_END_CORRECTION_COEFFICIENT;
    }
    open_end_radiation_resistance_scale = desc->open_end_radiation_resistance_scale;
    if (open_end_radiation_resistance_scale <= 0.0)
    {
        open_end_radiation_resistance_scale = DEFAULT_OPEN_END_RADIATION_RESISTANCE_SCALE;
    }
    InitializeUniformField(state->velocity_update_coeff, desc->velocity_cell_count, (f32) velocity_update_coeff);
    InitializePressureUpdateCoefficients(state);

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
        state->mouth_feedback_delay_lengths[source_index] = DEFAULT_NONLINEAR_MOUTH_PARAMETERS.delay_samples;
        state->mouth_feedback_delay_indices[source_index] = 0;
    }
    state->nonlinear_mouth = DEFAULT_NONLINEAR_MOUTH_PARAMETERS;
    state->jet_labium = DEFAULT_JET_LABIUM_PARAMETERS;

    state->left_boundary_type = desc->left_boundary.type;
    state->right_boundary_type = desc->right_boundary.type;
    state->left_reflection_coefficient = GetBoundaryReflectionCoefficient(&desc->left_boundary);
    state->right_reflection_coefficient = GetBoundaryReflectionCoefficient(&desc->right_boundary);

    if (desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)
    {
        state->left_reflection_coefficient = -1.0f;
        InitializeOpenBoundaryFilter(
            state,
            state->area_velocity[0],
            open_end_correction_coefficient,
            open_end_radiation_resistance_scale,
            &state->left_open_reflection_filter_a1,
            &state->left_open_reflection_filter_b0,
            &state->left_open_reflection_filter_b1
        );
    }

    if (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)
    {
        state->right_reflection_coefficient = -1.0f;
        InitializeOpenBoundaryFilter(
            state,
            state->area_velocity[state->velocity_cell_count - 1],
            open_end_correction_coefficient,
            open_end_radiation_resistance_scale,
            &state->right_open_reflection_filter_a1,
            &state->right_open_reflection_filter_b0,
            &state->right_open_reflection_filter_b1
        );
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

bool Fdtd1D_SetNonlinearMouthParameters (
    Fdtd1D *solver,
    const Fdtd1DNonlinearMouthParameters *parameters
)
{
    Fdtd1DState *state;
    u32 source_index;

    ASSERT(solver != NULL);
    ASSERT(parameters != NULL);
    ASSERT(solver->simulation.is_initialized == true);

    if (ValidateNonlinearMouthParameters(parameters) == false)
    {
        return false;
    }

    state = solver->state;
    ASSERT(state != NULL);

    state->nonlinear_mouth = *parameters;
    for (source_index = 0; source_index < state->source_count; source_index += 1)
    {
        state->mouth_feedback_delay_lengths[source_index] = parameters->delay_samples;
        state->mouth_feedback_delay_indices[source_index] = 0;
    }

    if ((state->mouth_feedback_delay_buffer != NULL) && (state->source_count > 0))
    {
        usize delay_sample_count;

        delay_sample_count = (usize) state->source_count * (usize) state->mouth_feedback_delay_capacity;
        memset(state->mouth_feedback_delay_buffer, 0, sizeof(f32) * delay_sample_count);
    }

    return true;
}

bool Fdtd1D_SetJetLabiumParameters (
    Fdtd1D *solver,
    const Fdtd1DJetLabiumParameters *parameters
)
{
    Fdtd1DState *state;
    u32 source_index;

    ASSERT(solver != NULL);
    ASSERT(parameters != NULL);
    ASSERT(solver->simulation.is_initialized == true);

    if (ValidateJetLabiumParameters(parameters) == false)
    {
        return false;
    }

    state = solver->state;
    ASSERT(state != NULL);

    state->jet_labium = *parameters;
    for (source_index = 0; source_index < state->source_count; source_index += 1)
    {
        ResetSourceDelayState(state, source_index, parameters->delay_samples);
        state->source_jet_state[source_index] = 0.0f;
    }

    return true;
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


