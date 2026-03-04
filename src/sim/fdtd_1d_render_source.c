#include <math.h>
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

static const f64 DRIVE_SMOOTHING_SECONDS = 0.05;
static const f64 BIAS_AND_NOISE_NOISE_SCALE = 0.35;
static const f64 FEEDBACK_NOISE_SCALE = 0.10;
static const f64 FEEDBACK_PRESSURE_COEFFICIENT = 0.65;
static const f64 FEEDBACK_VELOCITY_COEFFICIENT = 0.25;
static const f64 MAX_SAFE_DRIVE_AMPLITUDE = 0.002;
static const f64 JET_LABIUM_SOFT_DRIVE_SCALE = 0.0035;
static const f64 JET_LABIUM_HARD_DRIVE_LIMIT = 0.01;
static const f64 MAX_SAFE_STARTUP_CHIFF_NOISE = 0.0025;
static const f64 DEFAULT_SPEECH_ATTACK_SECONDS = 0.06;
static const f64 DEFAULT_SPEECH_CHIFF_AMOUNT = 0.35;
static const f64 DEFAULT_SPEECH_CHIFF_DECAY_SECONDS = 0.05;

static f64 ClampF64 (f64 value, f64 min_value, f64 max_value)
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

static f64 ClampNonNegativeF64 (f64 value)
{
    return (value < 0.0) ? 0.0 : value;
}

static f64 ClampUnitF64 (f64 value)
{
    return ClampF64(value, 0.0, 1.0);
}

static f64 ComputeSmoothedDriveAmplitude (f64 current_value, f64 target_value, f64 block_seconds)
{
    f64 smoothing_alpha;

    if (DRIVE_SMOOTHING_SECONDS <= 0.0)
    {
        return target_value;
    }

    smoothing_alpha = 1.0 - exp(-block_seconds / DRIVE_SMOOTHING_SECONDS);
    return current_value + (target_value - current_value) * smoothing_alpha;
}

static f64 ComputeSoftLimitedDrive (f64 requested_drive, f64 soft_scale, f64 hard_limit)
{
    f64 soft_limited;

    if (requested_drive <= 0.0)
    {
        return 0.0;
    }

    if (soft_scale <= 0.0)
    {
        return ClampF64(requested_drive, 0.0, hard_limit);
    }

    soft_limited = soft_scale * tanh(requested_drive / soft_scale);
    return ClampF64(soft_limited, 0.0, hard_limit);
}

static bool IsSpeechExcitationMode (Fdtd1DExcitationMode excitation_mode)
{
    switch (excitation_mode)
    {
        case FDTD_1D_EXCITATION_MODE_CONSTANT:
        case FDTD_1D_EXCITATION_MODE_NOISE:
        case FDTD_1D_EXCITATION_MODE_BIAS_AND_NOISE:
        case FDTD_1D_EXCITATION_MODE_FEEDBACK_MOUTH:
        case FDTD_1D_EXCITATION_MODE_NONLINEAR_MOUTH:
        case FDTD_1D_EXCITATION_MODE_JET_LABIUM:
        {
            return true;
        }

        case FDTD_1D_EXCITATION_MODE_IMPULSE:
        {
            return false;
        }
    }

    ASSERT(false);
    return false;
}

static void RestartSpeechState (Fdtd1DRenderSource *source)
{
    ASSERT(source != NULL);

    source->speech_gate = 0.0;
    source->speech_onset_seconds = 0.0;
    source->speech_is_active = false;
}

static f64 ComputeAverageMouthFeedbackSignal (
    const Simulation *simulation,
    const Fdtd1DState *state
)
{
    f64 pressure_sum;
    f64 velocity_sum;
    f64 pressure_weight;
    f64 velocity_weight;
    usize frame_index;

    ASSERT(simulation != NULL);
    ASSERT(state != NULL);

    if ((simulation->probe_buffer == NULL) || (simulation->config.probe_count < 6))
    {
        return 0.0;
    }

    pressure_sum = 0.0;
    velocity_sum = 0.0;

    for (frame_index = 0; frame_index < simulation->config.block_frame_count; frame_index += 1)
    {
        f32 mouth_pressure;
        f32 mouth_velocity;
        usize probe_offset;

        probe_offset = frame_index * (usize) simulation->config.probe_count;
        if (state->left_boundary_type == FDTD_1D_BOUNDARY_TYPE_OPEN)
        {
            mouth_pressure = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_LEFT_MOUTH_PRESSURE];
            mouth_velocity = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_LEFT_MOUTH_VELOCITY];
        }
        else
        {
            mouth_pressure = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_RIGHT_MOUTH_PRESSURE];
            mouth_velocity = simulation->probe_buffer[probe_offset + RENDER_SOURCE_PROBE_INDEX_RIGHT_MOUTH_VELOCITY];
        }

        pressure_sum += (f64) mouth_pressure;
        velocity_sum += (f64) mouth_velocity;
    }

    pressure_sum /= (f64) simulation->config.block_frame_count;
    velocity_sum /= (f64) simulation->config.block_frame_count;

    pressure_weight = FEEDBACK_PRESSURE_COEFFICIENT;
    velocity_weight = FEEDBACK_VELOCITY_COEFFICIENT * state->characteristic_impedance;

    return (pressure_weight * pressure_sum) + (velocity_weight * velocity_sum);
}

static SimulationExcitationType GetContinuousExcitationType (
    Fdtd1DSourceCouplingMode source_coupling_mode,
    Fdtd1DExcitationMode excitation_mode
)
{
    switch (source_coupling_mode)
    {
        case FDTD_1D_SOURCE_COUPLING_MODE_PRESSURE:
        {
            switch (excitation_mode)
            {
                case FDTD_1D_EXCITATION_MODE_CONSTANT: return SIMULATION_EXCITATION_TYPE_CONSTANT;
                case FDTD_1D_EXCITATION_MODE_NOISE: return SIMULATION_EXCITATION_TYPE_NOISE;
                case FDTD_1D_EXCITATION_MODE_IMPULSE:
                case FDTD_1D_EXCITATION_MODE_BIAS_AND_NOISE: break;
            }
        } break;

        case FDTD_1D_SOURCE_COUPLING_MODE_VELOCITY:
        {
            switch (excitation_mode)
            {
                case FDTD_1D_EXCITATION_MODE_CONSTANT: return SIMULATION_EXCITATION_TYPE_VELOCITY_CONSTANT;
                case FDTD_1D_EXCITATION_MODE_NOISE: return SIMULATION_EXCITATION_TYPE_VELOCITY_NOISE;
                case FDTD_1D_EXCITATION_MODE_IMPULSE:
                case FDTD_1D_EXCITATION_MODE_BIAS_AND_NOISE: break;
            }
        } break;
    }

    ASSERT(false);
    return SIMULATION_EXCITATION_TYPE_NONE;
}

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
    source->source_coupling_mode = desc->source_coupling_mode;
    source->drive_amplitude = desc->drive_amplitude;
    source->smoothed_drive_amplitude = desc->drive_amplitude;
    source->windchest_pressure = ClampNonNegativeF64(desc->windchest_pressure);
    source->smoothed_windchest_pressure = source->windchest_pressure;
    source->speech_attack_seconds =
        (desc->speech_attack_seconds > 0.0) ?
            ClampNonNegativeF64(desc->speech_attack_seconds) :
            DEFAULT_SPEECH_ATTACK_SECONDS;
    source->speech_chiff_amount =
        (desc->speech_chiff_amount > 0.0) ?
            ClampNonNegativeF64(desc->speech_chiff_amount) :
            DEFAULT_SPEECH_CHIFF_AMOUNT;
    source->speech_chiff_decay_seconds =
        (desc->speech_chiff_decay_seconds > 0.0) ?
            ClampNonNegativeF64(desc->speech_chiff_decay_seconds) :
            DEFAULT_SPEECH_CHIFF_DECAY_SECONDS;
    source->speech_gate = 0.0;
    source->speech_onset_seconds = 0.0;
    source->speech_is_active = false;
    source->last_requested_drive = 0.0;
    source->last_applied_drive = 0.0;
    source->last_drive_saturation_ratio = 0.0;
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
    RestartSpeechState(source);
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

void Fdtd1DRenderSource_SetSourceCouplingMode (
    Fdtd1DRenderSource *source,
    Fdtd1DSourceCouplingMode source_coupling_mode
)
{
    ASSERT(source != NULL);

    source->source_coupling_mode = source_coupling_mode;
}

void Fdtd1DRenderSource_SetDriveAmplitude (Fdtd1DRenderSource *source, f64 drive_amplitude)
{
    ASSERT(source != NULL);

    source->drive_amplitude = drive_amplitude;
}

void Fdtd1DRenderSource_SetWindchestPressure (Fdtd1DRenderSource *source, f64 windchest_pressure)
{
    ASSERT(source != NULL);

    source->windchest_pressure = ClampNonNegativeF64(windchest_pressure);
}

void Fdtd1DRenderSource_SetSpeechAttackSeconds (Fdtd1DRenderSource *source, f64 speech_attack_seconds)
{
    ASSERT(source != NULL);

    source->speech_attack_seconds = ClampNonNegativeF64(speech_attack_seconds);
}

void Fdtd1DRenderSource_SetSpeechChiffAmount (Fdtd1DRenderSource *source, f64 speech_chiff_amount)
{
    ASSERT(source != NULL);

    source->speech_chiff_amount = ClampNonNegativeF64(speech_chiff_amount);
}

void Fdtd1DRenderSource_SetSpeechChiffDecaySeconds (
    Fdtd1DRenderSource *source,
    f64 speech_chiff_decay_seconds
)
{
    ASSERT(source != NULL);

    source->speech_chiff_decay_seconds = ClampNonNegativeF64(speech_chiff_decay_seconds);
}

void Fdtd1DRenderSource_RestartSpeech (Fdtd1DRenderSource *source)
{
    ASSERT(source != NULL);
    RestartSpeechState(source);
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
    f64 block_seconds;
    f64 effective_drive_amplitude;
    f64 feedback_signal;
    f64 requested_drive_amplitude;
    f64 smoothed_drive_amplitude;
    f64 smoothed_windchest_pressure;
    f64 speech_gate_alpha;
    f64 speech_gated_drive_amplitude;
    f64 speech_chiff_gain;
    f64 speech_chiff_noise_amplitude;
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

    block_seconds = (f64) block_frame_count / (f64) sample_rate;
    source->smoothed_drive_amplitude = ComputeSmoothedDriveAmplitude(
        source->smoothed_drive_amplitude,
        source->drive_amplitude,
        block_seconds
    );
    source->smoothed_windchest_pressure = ComputeSmoothedDriveAmplitude(
        source->smoothed_windchest_pressure,
        source->windchest_pressure,
        block_seconds
    );
    smoothed_drive_amplitude = source->smoothed_drive_amplitude;
    smoothed_windchest_pressure = source->smoothed_windchest_pressure;
    effective_drive_amplitude = smoothed_drive_amplitude * smoothed_windchest_pressure;
    requested_drive_amplitude = effective_drive_amplitude;
    feedback_signal = ComputeAverageMouthFeedbackSignal(simulation, state);
    speech_chiff_noise_amplitude = 0.0;

    if (IsSpeechExcitationMode(source->excitation_mode) && (effective_drive_amplitude > 0.0))
    {
        if (source->speech_is_active == false)
        {
            source->speech_gate = 0.0;
            source->speech_onset_seconds = 0.0;
            source->speech_is_active = true;
        }

        if (source->speech_attack_seconds <= 0.0)
        {
            source->speech_gate = 1.0;
        }
        else
        {
            speech_gate_alpha = 1.0 - exp(-block_seconds / source->speech_attack_seconds);
            source->speech_gate += (1.0 - source->speech_gate) * speech_gate_alpha;
            source->speech_gate = ClampUnitF64(source->speech_gate);
        }

        source->speech_onset_seconds += block_seconds;
        speech_gated_drive_amplitude = effective_drive_amplitude * source->speech_gate;

        speech_chiff_gain = 1.0;
        if ((source->speech_chiff_amount > 0.0) && (source->speech_chiff_decay_seconds > 0.0))
        {
            speech_chiff_gain += source->speech_chiff_amount *
                exp(-source->speech_onset_seconds / source->speech_chiff_decay_seconds);
            speech_chiff_noise_amplitude = requested_drive_amplitude *
                source->speech_chiff_amount *
                exp(-source->speech_onset_seconds / source->speech_chiff_decay_seconds);
        }

        requested_drive_amplitude = speech_gated_drive_amplitude * speech_chiff_gain;
    }
    else
    {
        source->speech_gate = 0.0;
        source->speech_onset_seconds = 0.0;
        source->speech_is_active = false;
    }

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

    if (requested_drive_amplitude > 0.0)
    {
        switch (source->excitation_mode)
        {
            case FDTD_1D_EXCITATION_MODE_IMPULSE:
            {
            } break;

            case FDTD_1D_EXCITATION_MODE_CONSTANT:
            {
                memset(&excitation, 0, sizeof(excitation));
                excitation.type = GetContinuousExcitationType(
                    source->source_coupling_mode,
                    source->excitation_mode
                );
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = requested_drive_amplitude;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);
            } break;

            case FDTD_1D_EXCITATION_MODE_NOISE:
            {
                memset(&excitation, 0, sizeof(excitation));
                excitation.type = GetContinuousExcitationType(
                    source->source_coupling_mode,
                    source->excitation_mode
                );
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = requested_drive_amplitude;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);
            } break;

            case FDTD_1D_EXCITATION_MODE_BIAS_AND_NOISE:
            {
                SimulationExcitationType constant_type;
                SimulationExcitationType noise_type;

                constant_type = GetContinuousExcitationType(
                    source->source_coupling_mode,
                    FDTD_1D_EXCITATION_MODE_CONSTANT
                );
                noise_type = GetContinuousExcitationType(
                    source->source_coupling_mode,
                    FDTD_1D_EXCITATION_MODE_NOISE
                );

                memset(&excitation, 0, sizeof(excitation));
                excitation.type = constant_type;
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = requested_drive_amplitude;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);

                memset(&excitation, 0, sizeof(excitation));
                excitation.type = noise_type;
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = requested_drive_amplitude * BIAS_AND_NOISE_NOISE_SCALE;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);
            } break;

            case FDTD_1D_EXCITATION_MODE_FEEDBACK_MOUTH:
            {
                SimulationExcitationType constant_type;
                SimulationExcitationType noise_type;
                f64 feedback_drive_amplitude;

                constant_type = GetContinuousExcitationType(
                    source->source_coupling_mode,
                    FDTD_1D_EXCITATION_MODE_CONSTANT
                );
                noise_type = GetContinuousExcitationType(
                    source->source_coupling_mode,
                    FDTD_1D_EXCITATION_MODE_NOISE
                );

                feedback_drive_amplitude = requested_drive_amplitude - feedback_signal;
                feedback_drive_amplitude = ClampF64(
                    feedback_drive_amplitude,
                    0.0,
                    MAX_SAFE_DRIVE_AMPLITUDE
                );

                memset(&excitation, 0, sizeof(excitation));
                excitation.type = constant_type;
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = feedback_drive_amplitude;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);

                memset(&excitation, 0, sizeof(excitation));
                excitation.type = noise_type;
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = feedback_drive_amplitude * FEEDBACK_NOISE_SCALE;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);
            } break;

            case FDTD_1D_EXCITATION_MODE_NONLINEAR_MOUTH:
            {
                f64 bounded_drive_amplitude;

                bounded_drive_amplitude = ClampF64(requested_drive_amplitude, 0.0, MAX_SAFE_DRIVE_AMPLITUDE);

                memset(&excitation, 0, sizeof(excitation));
                excitation.type = SIMULATION_EXCITATION_TYPE_NONLINEAR_MOUTH;
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = bounded_drive_amplitude;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);
            } break;

            case FDTD_1D_EXCITATION_MODE_JET_LABIUM:
            {
                f64 applied_drive_amplitude;

                applied_drive_amplitude = ComputeSoftLimitedDrive(
                    requested_drive_amplitude,
                    JET_LABIUM_SOFT_DRIVE_SCALE,
                    JET_LABIUM_HARD_DRIVE_LIMIT
                );
                source->last_requested_drive = requested_drive_amplitude;
                source->last_applied_drive = applied_drive_amplitude;
                source->last_drive_saturation_ratio =
                    (requested_drive_amplitude > 0.0) ?
                        (applied_drive_amplitude / requested_drive_amplitude) :
                        0.0;

                memset(&excitation, 0, sizeof(excitation));
                excitation.type = SIMULATION_EXCITATION_TYPE_JET_LABIUM;
                excitation.target_index = source->startup_impulse_target_index;
                excitation.remaining_frame_count = block_frame_count;
                excitation.value = applied_drive_amplitude;
                excitation.is_active = true;
                Simulation_QueueExcitation(simulation, &excitation);

                if (speech_chiff_noise_amplitude > 0.0)
                {
                    memset(&excitation, 0, sizeof(excitation));
                    excitation.type = SIMULATION_EXCITATION_TYPE_VELOCITY_NOISE;
                    excitation.target_index = source->startup_impulse_target_index;
                    excitation.remaining_frame_count = block_frame_count;
                    excitation.value = ClampF64(
                        speech_chiff_noise_amplitude,
                        0.0,
                        MAX_SAFE_STARTUP_CHIFF_NOISE
                    );
                    excitation.is_active = true;
                    Simulation_QueueExcitation(simulation, &excitation);
                }
            } break;
        }
    }
    else
    {
        source->last_requested_drive = 0.0;
        source->last_applied_drive = 0.0;
        source->last_drive_saturation_ratio = 0.0;
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
