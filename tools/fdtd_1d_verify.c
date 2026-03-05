#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/core/memory_arena.h"
#include "pm_organ/sim/fdtd_1d.h"

#define MODE_COUNT 5

static f64 GetFundamentalFrequencyHz (const Fdtd1DDesc *desc);

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

typedef struct SustainedAnalysisResult
{
    u32 early_start_frame;
    u32 early_end_frame;
    u32 late_start_frame;
    u32 late_end_frame;
    u32 late_window_a_start_frame;
    u32 late_window_a_end_frame;
    u32 late_window_b_start_frame;
    u32 late_window_b_end_frame;
    f64 early_rms;
    f64 late_rms;
    f64 late_dominant_frequency_a_hz;
    f64 late_dominant_frequency_b_hz;
    f64 dominant_frequency_drift_hz;
    f64 dominant_frequency_drift_ratio;
    f64 late_to_early_rms_ratio;
    f64 late_spectral_centroid_hz;
    f64 late_spectral_flatness;
    f64 late_harmonic_energy_ratio;
    f64 late_spectral_flux;
    f64 harmonic_decay_low_band_db;
    f64 harmonic_decay_high_band_db;
    f64 harmonic_decay_slope_db_per_harmonic;
} SustainedAnalysisResult;

typedef struct SpeechAnalysisResult
{
    u32 onset_frame;
    u32 peak_frame;
    u32 attack_10_frame;
    u32 attack_90_frame;
    u32 chiff_start_frame;
    u32 chiff_end_frame;
    u32 sustain_start_frame;
    u32 sustain_end_frame;
    u32 settle_frame;
    f64 peak_abs;
    f64 onset_to_peak_ms;
    f64 attack_10_to_90_ms;
    f64 chiff_rms;
    f64 sustain_rms;
    f64 chiff_to_sustain_rms_ratio;
    f64 settling_time_ms;
    bool has_signal;
    bool attack_was_found;
    bool settling_was_found;
} SpeechAnalysisResult;

typedef enum VerificationExcitationType
{
    VERIFICATION_EXCITATION_TYPE_IMPULSE = 0,
    VERIFICATION_EXCITATION_TYPE_NOISE_BURST,
    VERIFICATION_EXCITATION_TYPE_CONSTANT,
    VERIFICATION_EXCITATION_TYPE_NONLINEAR_MOUTH,
    VERIFICATION_EXCITATION_TYPE_JET_LABIUM,
} VerificationExcitationType;

typedef enum VerificationPreset
{
    VERIFICATION_PRESET_RIGID_RIGID = 0,
    VERIFICATION_PRESET_OPEN_OPEN,
    VERIFICATION_PRESET_OPEN_RIGID,
    VERIFICATION_PRESET_UNIFORM_STOPPED,
    VERIFICATION_PRESET_NARROW_MOUTH_STOPPED,
    VERIFICATION_PRESET_WIDE_MOUTH_STOPPED,
    VERIFICATION_PRESET_OPEN_PIPE,
} VerificationPreset;

typedef struct VerificationSettings
{
    VerificationPreset preset;
    Fdtd1DProbeType probe_type;
    VerificationExcitationType excitation_type;
    Fdtd1DNonlinearMouthParameters nonlinear_mouth_parameters;
    const char *csv_output_path;
    bool source_index_was_overridden;
    bool left_probe_index_was_overridden;
    bool right_probe_index_was_overridden;
    bool run_length_sweep;
    bool run_nonlinear_mouth_sweep;
    bool run_speech_analysis;
    u32 block_count;
    u32 length_sweep_start_cell_count;
    u32 length_sweep_end_cell_count;
    u32 length_sweep_step_cell_count;
    u32 pressure_cell_count;
    u32 source_cell_index;
    u32 left_probe_index;
    u32 right_probe_index;
    f64 nonlinear_mouth_feedback_scale_start;
    f64 nonlinear_mouth_feedback_scale_end;
    f64 nonlinear_mouth_feedback_scale_step;
    f64 uniform_loss;
    f64 uniform_high_frequency_loss;
    f64 uniform_boundary_loss;
    f64 uniform_boundary_high_frequency_loss;
    f64 area_loss_reference_m2;
    f64 area_loss_strength;
    f64 open_end_correction_coefficient;
    f64 open_end_radiation_resistance_scale;
} VerificationSettings;

typedef struct VerificationRunSummary
{
    ModeResult mode_results[MODE_COUNT];
    SustainedAnalysisResult sustained;
    SpeechAnalysisResult speech;
    EnergyResult energy;
    SimulationStats stats;
    f64 left_boundary_emission_rms;
    f64 right_boundary_emission_rms;
    f64 left_boundary_emission_peak_abs;
    f64 right_boundary_emission_peak_abs;
    bool left_boundary_emission_activity_ok;
    bool right_boundary_emission_activity_ok;
    u32 pressure_cell_count;
    f64 tube_length_m;
} VerificationRunSummary;

typedef struct LengthSweepResult
{
    u32 pressure_cell_count;
    f64 tube_length_m;
    f64 expected_fundamental_hz;
    f64 measured_fundamental_hz;
    f64 measured_fundamental_magnitude;
    f64 final_energy_drift;
} LengthSweepResult;

typedef struct NonlinearMouthSweepResult
{
    f64 feedback_scale;
    f64 early_rms;
    f64 late_to_early_rms_ratio;
    f64 dominant_frequency_drift_hz;
    f64 late_spectral_centroid_hz;
    f64 late_spectral_flatness;
    f64 late_harmonic_energy_ratio;
    f64 late_spectral_flux;
    f64 max_output_abs;
    f64 energy_drift;
    bool early_rms_ok;
    bool late_to_early_rms_ok;
    bool dominant_frequency_drift_ok;
    bool energy_drift_ok;
    bool is_stable_candidate;
} NonlinearMouthSweepResult;

static const char *GetProbeTypeName (Fdtd1DProbeType probe_type)
{
    switch (probe_type)
    {
        case FDTD_1D_PROBE_TYPE_PRESSURE: return "pressure";
        case FDTD_1D_PROBE_TYPE_VELOCITY: return "velocity";
        case FDTD_1D_PROBE_TYPE_LEFT_BOUNDARY_EMISSION: return "left-boundary-emission";
        case FDTD_1D_PROBE_TYPE_RIGHT_BOUNDARY_EMISSION: return "right-boundary-emission";
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
        case VERIFICATION_PRESET_UNIFORM_STOPPED: return "uniform-stopped";
        case VERIFICATION_PRESET_NARROW_MOUTH_STOPPED: return "narrow-mouth";
        case VERIFICATION_PRESET_WIDE_MOUTH_STOPPED: return "wide-mouth";
        case VERIFICATION_PRESET_OPEN_PIPE: return "open-pipe";
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

    if (_stricmp(text, "uniform-stopped") == 0)
    {
        *preset = VERIFICATION_PRESET_UNIFORM_STOPPED;
        return true;
    }

    if (_stricmp(text, "narrow-mouth") == 0)
    {
        *preset = VERIFICATION_PRESET_NARROW_MOUTH_STOPPED;
        return true;
    }

    if (_stricmp(text, "wide-mouth") == 0)
    {
        *preset = VERIFICATION_PRESET_WIDE_MOUTH_STOPPED;
        return true;
    }

    if (_stricmp(text, "open-pipe") == 0)
    {
        *preset = VERIFICATION_PRESET_OPEN_PIPE;
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
        case VERIFICATION_EXCITATION_TYPE_CONSTANT: return "constant";
        case VERIFICATION_EXCITATION_TYPE_NONLINEAR_MOUTH: return "nonlinear-mouth";
        case VERIFICATION_EXCITATION_TYPE_JET_LABIUM: return "jet-labium";
    }

    ASSERT(false);
    return "unknown";
}

static bool EvaluateEarlyRmsGate (f64 early_rms)
{
    static const f64 EARLY_RMS_MIN = 0.01;
    return early_rms >= EARLY_RMS_MIN;
}

static bool EvaluateLateToEarlyRmsGate (f64 late_to_early_rms_ratio)
{
    static const f64 LATE_EARLY_RATIO_MIN = 0.5;
    static const f64 LATE_EARLY_RATIO_MAX = 1.5;
    return (late_to_early_rms_ratio >= LATE_EARLY_RATIO_MIN) &&
           (late_to_early_rms_ratio <= LATE_EARLY_RATIO_MAX);
}

static bool EvaluateDominantFrequencyDriftGate (f64 dominant_frequency_drift_ratio)
{
    static const f64 MAX_DOMINANT_FREQUENCY_DRIFT_RATIO = 0.12;
    return dominant_frequency_drift_ratio <= MAX_DOMINANT_FREQUENCY_DRIFT_RATIO;
}

static bool EvaluateEnergyDriftGate (f64 energy_drift)
{
    static const f64 MAX_ABS_ENERGY_DRIFT = 1.0;
    return fabs(energy_drift) <= MAX_ABS_ENERGY_DRIFT;
}

static bool EvaluateBoundaryEmissionActivityGate (Fdtd1DBoundaryType boundary_type, f64 emission_rms)
{
    static const f64 OPEN_BOUNDARY_MIN_RMS = 1.0e-6;
    static const f64 NON_OPEN_BOUNDARY_MAX_RMS = 1.0e-6;

    if (boundary_type == FDTD_1D_BOUNDARY_TYPE_OPEN)
    {
        return emission_rms >= OPEN_BOUNDARY_MIN_RMS;
    }

    return emission_rms <= NON_OPEN_BOUNDARY_MAX_RMS;
}

static bool EvaluateSpeechOnsetToPeakGate (f64 onset_to_peak_ms)
{
    static const f64 MAX_ONSET_TO_PEAK_MS = 700.0;
    return onset_to_peak_ms <= MAX_ONSET_TO_PEAK_MS;
}

static bool EvaluateSpeechAttackGate (bool attack_was_found, f64 attack_10_to_90_ms)
{
    static const f64 MIN_ATTACK_10_TO_90_MS = 2.0;
    static const f64 MAX_ATTACK_10_TO_90_MS = 650.0;

    if (attack_was_found == false)
    {
        return false;
    }

    return (attack_10_to_90_ms >= MIN_ATTACK_10_TO_90_MS) &&
           (attack_10_to_90_ms <= MAX_ATTACK_10_TO_90_MS);
}

static bool EvaluateSpeechChiffToSustainGate (f64 chiff_to_sustain_rms_ratio)
{
    static const f64 MIN_CHIFF_RATIO = 0.2;
    static const f64 MAX_CHIFF_RATIO = 2.5;
    return (chiff_to_sustain_rms_ratio >= MIN_CHIFF_RATIO) &&
           (chiff_to_sustain_rms_ratio <= MAX_CHIFF_RATIO);
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
    static const f64 DEFAULT_SOURCE_RATIO = 6.0 / 128.0;
    static const f64 OPEN_PIPE_SOURCE_RATIO = 16.0 / 128.0;
    static const f64 DEFAULT_LEFT_PROBE_RATIO = 72.0 / 128.0;
    static const f64 DEFAULT_RIGHT_PROBE_RATIO = 104.0 / 128.0;
    u32 pressure_cell_count;
    u32 source_cell_index;
    u32 left_probe_index;
    u32 right_probe_index;

    ASSERT(settings != NULL);
    ASSERT(desc != NULL);
    ASSERT(area_segment_descs != NULL);
    ASSERT(probe_descs != NULL);
    ASSERT(source_descs != NULL);

    pressure_cell_count = settings->pressure_cell_count;
    source_cell_index = settings->source_index_was_overridden ?
        settings->source_cell_index :
        (u32) (DEFAULT_SOURCE_RATIO * (f64) pressure_cell_count);
    if ((settings->source_index_was_overridden == false) &&
        (settings->preset == VERIFICATION_PRESET_OPEN_PIPE))
    {
        source_cell_index = (u32) (OPEN_PIPE_SOURCE_RATIO * (f64) pressure_cell_count);
    }
    left_probe_index = settings->left_probe_index_was_overridden ?
        settings->left_probe_index :
        (u32) (DEFAULT_LEFT_PROBE_RATIO * (f64) pressure_cell_count);
    right_probe_index = settings->right_probe_index_was_overridden ?
        settings->right_probe_index :
        (u32) (DEFAULT_RIGHT_PROBE_RATIO * (f64) pressure_cell_count);

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

    probe_descs[0].type = settings->probe_type;
    probe_descs[0].cell_index = left_probe_index;
    probe_descs[0].output_channel_index = 0;
    probe_descs[0].is_enabled = true;

    probe_descs[1].type = settings->probe_type;
    probe_descs[1].cell_index = right_probe_index;
    probe_descs[1].output_channel_index = 1;
    probe_descs[1].is_enabled = true;

    source_descs[0].cell_index = source_cell_index;
    source_descs[0].is_enabled = true;

    desc->sample_rate = 48000;
    desc->block_frame_count = 64;
    desc->output_channel_count = 2;
    desc->tube_length_m = (f64) pressure_cell_count * (343.0 / (TEST_COURANT_NUMBER * 48000.0));
    desc->wave_speed_m_per_s = 343.0;
    desc->density_kg_per_m3 = 1.225;
    desc->dx = 343.0 / (TEST_COURANT_NUMBER * 48000.0);
    desc->pressure_cell_count = pressure_cell_count;
    desc->velocity_cell_count = pressure_cell_count + 1;
    desc->courant_number = TEST_COURANT_NUMBER;
    desc->uniform_area_m2 = 0.01;
    desc->uniform_loss = settings->uniform_loss;
    desc->uniform_high_frequency_loss = settings->uniform_high_frequency_loss;
    desc->uniform_boundary_loss = settings->uniform_boundary_loss;
    desc->uniform_boundary_high_frequency_loss = settings->uniform_boundary_high_frequency_loss;
    desc->area_loss_reference_m2 = settings->area_loss_reference_m2;
    desc->area_loss_strength = settings->area_loss_strength;
    desc->open_end_correction_coefficient = settings->open_end_correction_coefficient;
    desc->open_end_radiation_resistance_scale = settings->open_end_radiation_resistance_scale;
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

        case VERIFICATION_PRESET_UNIFORM_STOPPED:
        {
            desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->left_boundary.reflection_coefficient = -1.0;
            desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
            desc->right_boundary.reflection_coefficient = 1.0;
        } break;

        case VERIFICATION_PRESET_NARROW_MOUTH_STOPPED:
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

        case VERIFICATION_PRESET_WIDE_MOUTH_STOPPED:
        {
            area_segment_descs[0].start_cell_index = 0;
            area_segment_descs[0].end_cell_index = 20;
            area_segment_descs[0].area_m2 = 0.014;

            desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->left_boundary.reflection_coefficient = -1.0;
            desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_RIGID;
            desc->right_boundary.reflection_coefficient = 1.0;
            desc->area_segment_count = 1;
            desc->area_segment_descs = area_segment_descs;
        } break;

        case VERIFICATION_PRESET_OPEN_PIPE:
        {
            area_segment_descs[0].start_cell_index = 0;
            area_segment_descs[0].end_cell_index = pressure_cell_count / 8;
            area_segment_descs[0].area_m2 = 0.012;
            area_segment_descs[1].start_cell_index = pressure_cell_count - (pressure_cell_count / 8);
            area_segment_descs[1].end_cell_index = pressure_cell_count;
            area_segment_descs[1].area_m2 = 0.012;

            desc->left_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->left_boundary.reflection_coefficient = -1.0;
            desc->right_boundary.type = FDTD_1D_BOUNDARY_TYPE_OPEN;
            desc->right_boundary.reflection_coefficient = -1.0;
            desc->area_segment_count = 2;
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

    if ((_stricmp(text, "left-boundary-emission") == 0) || (_stricmp(text, "left-emission") == 0))
    {
        *probe_type = FDTD_1D_PROBE_TYPE_LEFT_BOUNDARY_EMISSION;
        return true;
    }

    if ((_stricmp(text, "right-boundary-emission") == 0) || (_stricmp(text, "right-emission") == 0))
    {
        *probe_type = FDTD_1D_PROBE_TYPE_RIGHT_BOUNDARY_EMISSION;
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

    if ((_stricmp(text, "constant") == 0) || (_stricmp(text, "dc") == 0))
    {
        *excitation_type = VERIFICATION_EXCITATION_TYPE_CONSTANT;
        return true;
    }

    if ((_stricmp(text, "nonlinear-mouth") == 0) || (_stricmp(text, "mouth") == 0))
    {
        *excitation_type = VERIFICATION_EXCITATION_TYPE_NONLINEAR_MOUTH;
        return true;
    }

    if ((_stricmp(text, "jet-labium") == 0) || (_stricmp(text, "jet") == 0))
    {
        *excitation_type = VERIFICATION_EXCITATION_TYPE_JET_LABIUM;
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

static bool TryParseDoubleValue (const char *text, const char *prefix, f64 *value)
{
    usize prefix_length;
    char *end_pointer;
    double parsed_value;

    ASSERT(text != NULL);
    ASSERT(prefix != NULL);
    ASSERT(value != NULL);

    prefix_length = strlen(prefix);
    if (_strnicmp(text, prefix, prefix_length) != 0)
    {
        return false;
    }

    parsed_value = strtod(text + prefix_length, &end_pointer);
    if ((end_pointer == (text + prefix_length)) || (*end_pointer != '\0'))
    {
        return false;
    }

    *value = (f64) parsed_value;
    return true;
}

static u32 RoundUpToMultiple (u32 value, u32 multiple)
{
    ASSERT(multiple > 0);

    if ((value % multiple) == 0)
    {
        return value;
    }

    return value + (multiple - (value % multiple));
}

static void InitializeVerificationSettings (VerificationSettings *settings)
{
    ASSERT(settings != NULL);

    settings->preset = VERIFICATION_PRESET_RIGID_RIGID;
    settings->probe_type = FDTD_1D_PROBE_TYPE_PRESSURE;
    settings->excitation_type = VERIFICATION_EXCITATION_TYPE_IMPULSE;
    settings->nonlinear_mouth_parameters.max_output = 0.0002f;
    settings->nonlinear_mouth_parameters.noise_scale = 0.04f;
    settings->nonlinear_mouth_parameters.pressure_feedback = 0.35f;
    settings->nonlinear_mouth_parameters.velocity_feedback = 0.10f;
    settings->nonlinear_mouth_parameters.feedback_leak = 0.60f;
    settings->nonlinear_mouth_parameters.saturation_gain = 80.0f;
    settings->nonlinear_mouth_parameters.drive_limit = 0.0005f;
    settings->nonlinear_mouth_parameters.delay_samples = 8;
    settings->csv_output_path = NULL;
    settings->source_index_was_overridden = false;
    settings->left_probe_index_was_overridden = false;
    settings->right_probe_index_was_overridden = false;
    settings->run_length_sweep = false;
    settings->run_nonlinear_mouth_sweep = false;
    settings->run_speech_analysis = false;
    settings->block_count = 256;
    settings->length_sweep_start_cell_count = 64;
    settings->length_sweep_end_cell_count = 256;
    settings->length_sweep_step_cell_count = 32;
    settings->pressure_cell_count = 128;
    settings->source_cell_index = 6;
    settings->left_probe_index = 72;
    settings->right_probe_index = 104;
    settings->nonlinear_mouth_feedback_scale_start = 0.5;
    settings->nonlinear_mouth_feedback_scale_end = 1.5;
    settings->nonlinear_mouth_feedback_scale_step = 0.25;
    settings->uniform_loss = 0.00005;
    settings->uniform_high_frequency_loss = 0.012;
    settings->uniform_boundary_loss = 0.00008;
    settings->uniform_boundary_high_frequency_loss = 0.028;
    settings->area_loss_reference_m2 = 0.01;
    settings->area_loss_strength = 0.35;
    settings->open_end_correction_coefficient = 0.45;
    settings->open_end_radiation_resistance_scale = 1.5;
}

static void ParseArguments (int argc, char **argv, VerificationSettings *settings)
{
    int argument_index;

    ASSERT(settings != NULL);

    for (argument_index = 1; argument_index < argc; argument_index += 1)
    {
        const char *argument;
        f64 parsed_f64;
        u32 parsed_value;

        argument = argv[argument_index];

        if (_strnicmp(argument, "preset=", 7) == 0)
        {
            if (TryParsePresetName(argument + 7, &settings->preset))
            {
                continue;
            }
        }

        if (TryParsePresetName(argument, &settings->preset))
        {
            continue;
        }

        if (TryParseProbeTypeName(argument, &settings->probe_type))
        {
            continue;
        }

        if (_strnicmp(argument, "probe=", 6) == 0)
        {
            if (TryParseProbeTypeName(argument + 6, &settings->probe_type))
            {
                continue;
            }
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

        if (TryParseUnsignedValue(argument, "cells=", &parsed_value))
        {
            if (parsed_value > 7)
            {
                settings->pressure_cell_count = parsed_value;
            }
            continue;
        }

        if (TryParseUnsignedValue(argument, "sweep_start=", &parsed_value))
        {
            settings->length_sweep_start_cell_count = parsed_value;
            continue;
        }

        if (TryParseUnsignedValue(argument, "sweep_end=", &parsed_value))
        {
            settings->length_sweep_end_cell_count = parsed_value;
            continue;
        }

        if (TryParseUnsignedValue(argument, "sweep_step=", &parsed_value))
        {
            if (parsed_value > 0)
            {
                settings->length_sweep_step_cell_count = parsed_value;
            }
            continue;
        }

        if (TryParseUnsignedValue(argument, "source=", &parsed_value))
        {
            settings->source_cell_index = parsed_value;
            settings->source_index_was_overridden = true;
            continue;
        }

        if (TryParseUnsignedValue(argument, "left_probe=", &parsed_value))
        {
            settings->left_probe_index = parsed_value;
            settings->left_probe_index_was_overridden = true;
            continue;
        }

        if (TryParseUnsignedValue(argument, "right_probe=", &parsed_value))
        {
            settings->right_probe_index = parsed_value;
            settings->right_probe_index_was_overridden = true;
            continue;
        }

        if ((_stricmp(argument, "length-sweep") == 0) || (_stricmp(argument, "sweep-length") == 0))
        {
            settings->run_length_sweep = true;
            continue;
        }

        if ((_stricmp(argument, "mouth-sweep") == 0) || (_stricmp(argument, "sweep-mouth") == 0))
        {
            settings->run_nonlinear_mouth_sweep = true;
            settings->excitation_type = VERIFICATION_EXCITATION_TYPE_NONLINEAR_MOUTH;
            continue;
        }

        if ((_stricmp(argument, "speech-analysis") == 0) || (_stricmp(argument, "speech") == 0))
        {
            settings->run_speech_analysis = true;
            continue;
        }

        if (TryParseUnsignedValue(argument, "mouth_delay=", &parsed_value))
        {
            if (parsed_value > 0)
            {
                settings->nonlinear_mouth_parameters.delay_samples = parsed_value;
            }
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_max_output=", &parsed_f64))
        {
            settings->nonlinear_mouth_parameters.max_output = (f32) parsed_f64;
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_noise_scale=", &parsed_f64))
        {
            settings->nonlinear_mouth_parameters.noise_scale = (f32) parsed_f64;
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_pressure_feedback=", &parsed_f64))
        {
            settings->nonlinear_mouth_parameters.pressure_feedback = (f32) parsed_f64;
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_velocity_feedback=", &parsed_f64))
        {
            settings->nonlinear_mouth_parameters.velocity_feedback = (f32) parsed_f64;
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_feedback_leak=", &parsed_f64))
        {
            settings->nonlinear_mouth_parameters.feedback_leak = (f32) parsed_f64;
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_saturation_gain=", &parsed_f64))
        {
            settings->nonlinear_mouth_parameters.saturation_gain = (f32) parsed_f64;
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_drive_limit=", &parsed_f64))
        {
            settings->nonlinear_mouth_parameters.drive_limit = (f32) parsed_f64;
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_feedback_scale_start=", &settings->nonlinear_mouth_feedback_scale_start))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_feedback_scale_end=", &settings->nonlinear_mouth_feedback_scale_end))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "mouth_feedback_scale_step=", &settings->nonlinear_mouth_feedback_scale_step))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "area_loss_reference=", &settings->area_loss_reference_m2))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "area_loss_strength=", &settings->area_loss_strength))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "open_end_correction=", &settings->open_end_correction_coefficient))
        {
            continue;
        }

        if (TryParseDoubleValue(
                argument,
                "open_end_radiation_resistance=",
                &settings->open_end_radiation_resistance_scale
            ))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "uniform_loss=", &settings->uniform_loss))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "hf_loss=", &settings->uniform_high_frequency_loss))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "boundary_loss=", &settings->uniform_boundary_loss))
        {
            continue;
        }

        if (TryParseDoubleValue(argument, "boundary_hf_loss=", &settings->uniform_boundary_high_frequency_loss))
        {
            continue;
        }

        settings->csv_output_path = argument;
    }

    settings->pressure_cell_count = RoundUpToMultiple(settings->pressure_cell_count, 8);
    settings->length_sweep_start_cell_count = RoundUpToMultiple(settings->length_sweep_start_cell_count, 8);
    settings->length_sweep_end_cell_count = RoundUpToMultiple(settings->length_sweep_end_cell_count, 8);
    settings->length_sweep_step_cell_count = RoundUpToMultiple(settings->length_sweep_step_cell_count, 8);
    if (settings->nonlinear_mouth_feedback_scale_step <= 0.0)
    {
        settings->nonlinear_mouth_feedback_scale_step = 0.25;
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

static f64 ComputeWindowRms (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 end_frame
)
{
    f64 sum_squares;
    u32 frame_count;
    u32 frame_index;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);
    ASSERT(start_frame <= end_frame);

    if (capture->frame_count == 0)
    {
        return 0.0;
    }

    if (end_frame >= capture->frame_count)
    {
        end_frame = capture->frame_count - 1;
    }

    if (start_frame > end_frame)
    {
        return 0.0;
    }

    sum_squares = 0.0;
    frame_count = end_frame - start_frame + 1;
    for (frame_index = start_frame; frame_index <= end_frame; frame_index += 1)
    {
        f64 sample;

        sample = (f64) capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        sum_squares += sample * sample;
    }

    return sqrt(sum_squares / (f64) frame_count);
}

static f64 FindDominantFrequencyInRange (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 end_frame,
    f64 sample_rate,
    f64 frequency_start_hz,
    f64 frequency_end_hz,
    f64 frequency_step_hz
)
{
    f64 best_frequency_hz;
    f64 best_magnitude;
    f64 search_frequency_hz;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);
    ASSERT(sample_rate > 0.0);
    ASSERT(frequency_step_hz > 0.0);
    ASSERT(frequency_start_hz <= frequency_end_hz);

    best_frequency_hz = frequency_start_hz;
    best_magnitude = 0.0;

    for (search_frequency_hz = frequency_start_hz;
         search_frequency_hz <= frequency_end_hz;
         search_frequency_hz += frequency_step_hz)
    {
        f64 magnitude;

        magnitude = ComputeSpectrumMagnitude(
            capture,
            analysis_channel_index,
            start_frame,
            end_frame,
            sample_rate,
            search_frequency_hz
        );
        if (magnitude > best_magnitude)
        {
            best_magnitude = magnitude;
            best_frequency_hz = search_frequency_hz;
        }
    }

    return best_frequency_hz;
}

static void ComputeSpectralQualityMetrics (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 end_frame,
    f64 sample_rate,
    f64 frequency_start_hz,
    f64 frequency_end_hz,
    f64 frequency_step_hz,
    f64 *centroid_hz,
    f64 *flatness,
    f64 *total_energy
)
{
    static const f64 EPSILON = 1e-12;
    f64 bin_count;
    f64 energy_sum;
    f64 log_sum;
    f64 weighted_frequency_sum;
    f64 frequency_hz;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);
    ASSERT(sample_rate > 0.0);
    ASSERT(frequency_step_hz > 0.0);
    ASSERT(frequency_start_hz <= frequency_end_hz);
    ASSERT(centroid_hz != NULL);
    ASSERT(flatness != NULL);
    ASSERT(total_energy != NULL);

    bin_count = 0.0;
    energy_sum = 0.0;
    weighted_frequency_sum = 0.0;
    log_sum = 0.0;

    for (frequency_hz = frequency_start_hz;
         frequency_hz <= frequency_end_hz;
         frequency_hz += frequency_step_hz)
    {
        f64 magnitude;
        f64 power;

        magnitude = ComputeSpectrumMagnitude(
            capture,
            analysis_channel_index,
            start_frame,
            end_frame,
            sample_rate,
            frequency_hz
        );
        power = (magnitude * magnitude) + EPSILON;
        energy_sum += power;
        weighted_frequency_sum += frequency_hz * power;
        log_sum += log(power);
        bin_count += 1.0;
    }

    *total_energy = energy_sum;
    if (energy_sum > EPSILON)
    {
        *centroid_hz = weighted_frequency_sum / energy_sum;
    }
    else
    {
        *centroid_hz = frequency_start_hz;
    }

    if (bin_count > 0.0)
    {
        f64 geometric_mean;
        f64 arithmetic_mean;

        geometric_mean = exp(log_sum / bin_count);
        arithmetic_mean = energy_sum / bin_count;
        *flatness = geometric_mean / (arithmetic_mean + EPSILON);
    }
    else
    {
        *flatness = 0.0;
    }
}

static f64 ComputeHarmonicEnergyRatio (
    const SimulationOfflineCapture *capture,
    const Fdtd1DDesc *desc,
    u32 analysis_channel_index,
    u32 start_frame,
    u32 end_frame,
    f64 sample_rate,
    f64 frequency_start_hz,
    f64 frequency_end_hz
)
{
    static const f64 EPSILON = 1e-12;
    f64 expected_frequency_hz;
    f64 fundamental_frequency_hz;
    f64 harmonic_energy_sum;
    f64 total_energy;
    f64 unused_centroid;
    f64 unused_flatness;
    u32 harmonic_index;

    ASSERT(capture != NULL);
    ASSERT(desc != NULL);

    ComputeSpectralQualityMetrics(
        capture,
        analysis_channel_index,
        start_frame,
        end_frame,
        sample_rate,
        frequency_start_hz,
        frequency_end_hz,
        2.0,
        &unused_centroid,
        &unused_flatness,
        &total_energy
    );

    harmonic_energy_sum = 0.0;
    fundamental_frequency_hz = GetFundamentalFrequencyHz(desc);
    for (harmonic_index = 1; harmonic_index <= MODE_COUNT; harmonic_index += 1)
    {
        f64 harmonic_multiplier;
        f64 harmonic_magnitude;

        if (((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN) &&
             (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID)) ||
            ((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID) &&
             (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)))
        {
            harmonic_multiplier = (f64) (2 * (i32) harmonic_index - 1);
        }
        else
        {
            harmonic_multiplier = (f64) harmonic_index;
        }

        expected_frequency_hz = fundamental_frequency_hz * harmonic_multiplier;
        if ((expected_frequency_hz < frequency_start_hz) || (expected_frequency_hz > frequency_end_hz))
        {
            continue;
        }

        harmonic_magnitude = ComputeSpectrumMagnitude(
            capture,
            analysis_channel_index,
            start_frame,
            end_frame,
            sample_rate,
            expected_frequency_hz
        );
        harmonic_energy_sum += harmonic_magnitude * harmonic_magnitude;
    }

    return harmonic_energy_sum / (total_energy + EPSILON);
}

static f64 ComputeSpectralFlux (
    const SimulationOfflineCapture *capture,
    u32 analysis_channel_index,
    u32 start_frame_a,
    u32 end_frame_a,
    u32 start_frame_b,
    u32 end_frame_b,
    f64 sample_rate,
    f64 frequency_start_hz,
    f64 frequency_end_hz,
    f64 frequency_step_hz
)
{
    static const f64 EPSILON = 1e-12;
    f64 energy_sum_a;
    f64 flux_sum;
    f64 frequency_hz;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);

    energy_sum_a = 0.0;
    flux_sum = 0.0;
    for (frequency_hz = frequency_start_hz;
         frequency_hz <= frequency_end_hz;
         frequency_hz += frequency_step_hz)
    {
        f64 magnitude_a;
        f64 magnitude_b;
        f64 power_a;
        f64 power_b;
        f64 delta;

        magnitude_a = ComputeSpectrumMagnitude(
            capture,
            analysis_channel_index,
            start_frame_a,
            end_frame_a,
            sample_rate,
            frequency_hz
        );
        magnitude_b = ComputeSpectrumMagnitude(
            capture,
            analysis_channel_index,
            start_frame_b,
            end_frame_b,
            sample_rate,
            frequency_hz
        );
        power_a = magnitude_a * magnitude_a;
        power_b = magnitude_b * magnitude_b;
        delta = power_b - power_a;
        flux_sum += delta * delta;
        energy_sum_a += power_a;
    }

    return sqrt(flux_sum / (energy_sum_a + EPSILON));
}

static void ComputeHarmonicDecayMetrics (
    const SimulationOfflineCapture *capture,
    const Fdtd1DDesc *desc,
    u32 analysis_channel_index,
    u32 window_a_start_frame,
    u32 window_a_end_frame,
    u32 window_b_start_frame,
    u32 window_b_end_frame,
    f64 *low_band_db,
    f64 *high_band_db,
    f64 *slope_db_per_harmonic
)
{
    static const f64 EPSILON = 1e-12;
    f64 fundamental_frequency_hz;
    f64 x_mean;
    f64 y_mean;
    f64 xx_sum;
    f64 xy_sum;
    f64 low_sum;
    f64 high_sum;
    u32 low_count;
    u32 high_count;
    u32 used_count;
    u32 harmonic_index;

    ASSERT(capture != NULL);
    ASSERT(desc != NULL);
    ASSERT(low_band_db != NULL);
    ASSERT(high_band_db != NULL);
    ASSERT(slope_db_per_harmonic != NULL);

    *low_band_db = 0.0;
    *high_band_db = 0.0;
    *slope_db_per_harmonic = 0.0;

    fundamental_frequency_hz = GetFundamentalFrequencyHz(desc);
    if (fundamental_frequency_hz <= 0.0)
    {
        return;
    }

    x_mean = 0.0;
    y_mean = 0.0;
    xx_sum = 0.0;
    xy_sum = 0.0;
    low_sum = 0.0;
    high_sum = 0.0;
    low_count = 0;
    high_count = 0;
    used_count = 0;

    for (harmonic_index = 1; harmonic_index <= 8; harmonic_index += 1)
    {
        f64 frequency_hz;
        f64 harmonic_multiplier;
        f64 early_magnitude;
        f64 late_magnitude;
        f64 decay_db;
        f64 x;

        if (((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN) &&
             (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID)) ||
            ((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID) &&
             (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)))
        {
            harmonic_multiplier = (f64) (2 * (i32) harmonic_index - 1);
        }
        else
        {
            harmonic_multiplier = (f64) harmonic_index;
        }

        frequency_hz = fundamental_frequency_hz * harmonic_multiplier;
        if (frequency_hz >= (0.5 * (f64) desc->sample_rate - 20.0))
        {
            continue;
        }

        early_magnitude = ComputeSpectrumMagnitude(
            capture,
            analysis_channel_index,
            window_a_start_frame,
            window_a_end_frame,
            (f64) desc->sample_rate,
            frequency_hz
        );
        late_magnitude = ComputeSpectrumMagnitude(
            capture,
            analysis_channel_index,
            window_b_start_frame,
            window_b_end_frame,
            (f64) desc->sample_rate,
            frequency_hz
        );

        decay_db = 20.0 * log10((late_magnitude + EPSILON) / (early_magnitude + EPSILON));
        x = (f64) harmonic_index;
        x_mean += x;
        y_mean += decay_db;
        used_count += 1;

        if (harmonic_index <= 3)
        {
            low_sum += decay_db;
            low_count += 1;
        }
        else
        {
            high_sum += decay_db;
            high_count += 1;
        }
    }

    if (low_count > 0)
    {
        *low_band_db = low_sum / (f64) low_count;
    }
    if (high_count > 0)
    {
        *high_band_db = high_sum / (f64) high_count;
    }

    if (used_count >= 2)
    {
        f64 denominator;

        x_mean /= (f64) used_count;
        y_mean /= (f64) used_count;
        for (harmonic_index = 1; harmonic_index <= 8; harmonic_index += 1)
        {
            f64 frequency_hz;
            f64 harmonic_multiplier;
            f64 early_magnitude;
            f64 late_magnitude;
            f64 decay_db;
            f64 x;
            f64 dx;
            f64 dy;

            if (((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN) &&
                 (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID)) ||
                ((desc->left_boundary.type == FDTD_1D_BOUNDARY_TYPE_RIGID) &&
                 (desc->right_boundary.type == FDTD_1D_BOUNDARY_TYPE_OPEN)))
            {
                harmonic_multiplier = (f64) (2 * (i32) harmonic_index - 1);
            }
            else
            {
                harmonic_multiplier = (f64) harmonic_index;
            }

            frequency_hz = fundamental_frequency_hz * harmonic_multiplier;
            if (frequency_hz >= (0.5 * (f64) desc->sample_rate - 20.0))
            {
                continue;
            }

            early_magnitude = ComputeSpectrumMagnitude(
                capture,
                analysis_channel_index,
                window_a_start_frame,
                window_a_end_frame,
                (f64) desc->sample_rate,
                frequency_hz
            );
            late_magnitude = ComputeSpectrumMagnitude(
                capture,
                analysis_channel_index,
                window_b_start_frame,
                window_b_end_frame,
                (f64) desc->sample_rate,
                frequency_hz
            );
            decay_db = 20.0 * log10((late_magnitude + EPSILON) / (early_magnitude + EPSILON));
            x = (f64) harmonic_index;
            dx = x - x_mean;
            dy = decay_db - y_mean;
            xx_sum += dx * dx;
            xy_sum += dx * dy;
        }

        denominator = xx_sum + EPSILON;
        *slope_db_per_harmonic = xy_sum / denominator;
    }
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

static void AnalyzeSpeechBehavior (
    const SimulationOfflineCapture *capture,
    const Fdtd1DDesc *desc,
    u32 analysis_channel_index,
    SpeechAnalysisResult *result
)
{
    static const f64 SIGNAL_FLOOR = 1e-8;
    static const f64 ONSET_THRESHOLD_RATIO = 0.05;
    static const f64 ATTACK_10_RATIO = 0.10;
    static const f64 ATTACK_90_RATIO = 0.90;
    static const f64 MAX_CHIFF_WINDOW_SECONDS = 0.06;
    static const f64 SETTLE_SUSTAIN_MULTIPLIER = 1.25;
    static const f64 SETTLE_PEAK_RATIO = 0.12;
    u32 consecutive_required;
    u32 frame_index;
    u32 last_frame;
    u32 max_chiff_window_frames;
    u32 settle_consecutive_count;
    f64 attack_10_threshold;
    f64 attack_90_threshold;
    f64 onset_threshold;
    f64 settle_threshold;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(desc != NULL);
    ASSERT(result != NULL);
    ASSERT(analysis_channel_index < capture->channel_count);
    ASSERT(desc->sample_rate > 0);

    memset(result, 0, sizeof(*result));
    if (capture->frame_count == 0)
    {
        return;
    }

    last_frame = capture->frame_count - 1;
    for (frame_index = 0; frame_index < capture->frame_count; frame_index += 1)
    {
        f64 abs_sample;
        f32 sample;

        sample = capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        abs_sample = fabs((f64) sample);
        if (abs_sample > result->peak_abs)
        {
            result->peak_abs = abs_sample;
            result->peak_frame = frame_index;
        }
    }

    if (result->peak_abs <= SIGNAL_FLOOR)
    {
        return;
    }

    result->has_signal = true;
    onset_threshold = ONSET_THRESHOLD_RATIO * result->peak_abs;
    attack_10_threshold = ATTACK_10_RATIO * result->peak_abs;
    attack_90_threshold = ATTACK_90_RATIO * result->peak_abs;
    result->attack_10_frame = result->peak_frame;
    result->attack_90_frame = result->peak_frame;

    for (frame_index = 0; frame_index < capture->frame_count; frame_index += 1)
    {
        f64 abs_sample;
        f32 sample;

        sample = capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        abs_sample = fabs((f64) sample);
        if (abs_sample >= onset_threshold)
        {
            result->onset_frame = frame_index;
            break;
        }
    }

    for (frame_index = result->onset_frame; frame_index <= result->peak_frame; frame_index += 1)
    {
        f64 abs_sample;
        f32 sample;

        sample = capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        abs_sample = fabs((f64) sample);
        if (abs_sample >= attack_10_threshold)
        {
            result->attack_10_frame = frame_index;
            break;
        }
    }

    for (frame_index = result->attack_10_frame; frame_index <= result->peak_frame; frame_index += 1)
    {
        f64 abs_sample;
        f32 sample;

        sample = capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        abs_sample = fabs((f64) sample);
        if (abs_sample >= attack_90_threshold)
        {
            result->attack_90_frame = frame_index;
            result->attack_was_found = true;
            break;
        }
    }

    if (result->peak_frame >= result->onset_frame)
    {
        result->onset_to_peak_ms = 1000.0 *
            ((f64) (result->peak_frame - result->onset_frame) / (f64) desc->sample_rate);
    }
    if (result->attack_was_found && (result->attack_90_frame >= result->attack_10_frame))
    {
        result->attack_10_to_90_ms = 1000.0 *
            ((f64) (result->attack_90_frame - result->attack_10_frame) / (f64) desc->sample_rate);
    }

    max_chiff_window_frames = (u32) (MAX_CHIFF_WINDOW_SECONDS * (f64) desc->sample_rate);
    if (max_chiff_window_frames < 16)
    {
        max_chiff_window_frames = 16;
    }
    if (max_chiff_window_frames > (capture->frame_count / 8))
    {
        max_chiff_window_frames = capture->frame_count / 8;
    }
    if (max_chiff_window_frames < 1)
    {
        max_chiff_window_frames = 1;
    }

    result->chiff_start_frame = result->onset_frame;
    result->chiff_end_frame = result->chiff_start_frame + max_chiff_window_frames - 1;
    if (result->chiff_end_frame > last_frame)
    {
        result->chiff_end_frame = last_frame;
    }

    result->sustain_start_frame = (capture->frame_count * 3) / 4;
    if (result->sustain_start_frame > last_frame)
    {
        result->sustain_start_frame = last_frame;
    }
    result->sustain_end_frame = last_frame;

    result->chiff_rms = ComputeWindowRms(
        capture,
        analysis_channel_index,
        result->chiff_start_frame,
        result->chiff_end_frame
    );
    result->sustain_rms = ComputeWindowRms(
        capture,
        analysis_channel_index,
        result->sustain_start_frame,
        result->sustain_end_frame
    );
    result->chiff_to_sustain_rms_ratio = result->chiff_rms / (result->sustain_rms + 1e-12);

    settle_threshold = SETTLE_SUSTAIN_MULTIPLIER * result->sustain_rms;
    if (settle_threshold < (SETTLE_PEAK_RATIO * result->peak_abs))
    {
        settle_threshold = SETTLE_PEAK_RATIO * result->peak_abs;
    }

    consecutive_required = desc->sample_rate / 200;
    if (consecutive_required < 16)
    {
        consecutive_required = 16;
    }
    settle_consecutive_count = 0;
    for (frame_index = result->peak_frame; frame_index < capture->frame_count; frame_index += 1)
    {
        f64 abs_sample;
        f32 sample;

        sample = capture->samples[(usize) frame_index * (usize) capture->channel_count + analysis_channel_index];
        abs_sample = fabs((f64) sample);
        if (abs_sample <= settle_threshold)
        {
            settle_consecutive_count += 1;
            if (settle_consecutive_count >= consecutive_required)
            {
                result->settle_frame = frame_index + 1 - consecutive_required;
                result->settling_was_found = true;
                break;
            }
        }
        else
        {
            settle_consecutive_count = 0;
        }
    }

    if (result->settling_was_found && (result->settle_frame >= result->onset_frame))
    {
        result->settling_time_ms = 1000.0 *
            ((f64) (result->settle_frame - result->onset_frame) / (f64) desc->sample_rate);
    }
}

static void AnalyzeSustainedBehavior (
    const SimulationOfflineCapture *capture,
    const Fdtd1DDesc *desc,
    u32 analysis_channel_index,
    SustainedAnalysisResult *result
)
{
    static const f64 RMS_EPSILON = 1e-9;

    f64 spectral_energy;
    f64 frequency_end_hz;
    f64 frequency_start_hz;
    f64 fundamental_frequency_hz;
    f64 nyquist_hz;
    u32 mid_frame;

    ASSERT(capture != NULL);
    ASSERT(capture->samples != NULL);
    ASSERT(desc != NULL);
    ASSERT(result != NULL);

    memset(result, 0, sizeof(*result));
    if (capture->frame_count < 16)
    {
        return;
    }

    result->early_start_frame = 0;
    result->early_end_frame = (capture->frame_count / 4) - 1;
    result->late_start_frame = (capture->frame_count * 3) / 4;
    result->late_end_frame = capture->frame_count - 1;
    if (result->late_start_frame >= result->late_end_frame)
    {
        result->late_start_frame = capture->frame_count / 2;
    }

    mid_frame = result->late_start_frame + ((result->late_end_frame - result->late_start_frame) / 2);
    result->late_window_a_start_frame = result->late_start_frame;
    result->late_window_a_end_frame = mid_frame;
    result->late_window_b_start_frame = (mid_frame < result->late_end_frame) ? (mid_frame + 1) : mid_frame;
    result->late_window_b_end_frame = result->late_end_frame;

    result->early_rms = ComputeWindowRms(
        capture,
        analysis_channel_index,
        result->early_start_frame,
        result->early_end_frame
    );
    result->late_rms = ComputeWindowRms(
        capture,
        analysis_channel_index,
        result->late_start_frame,
        result->late_end_frame
    );
    result->late_to_early_rms_ratio = result->late_rms / (result->early_rms + RMS_EPSILON);

    fundamental_frequency_hz = GetFundamentalFrequencyHz(desc);
    nyquist_hz = 0.5 * (f64) desc->sample_rate;
    frequency_start_hz = fundamental_frequency_hz * 0.5;
    frequency_end_hz = fundamental_frequency_hz * 8.0;
    if (frequency_end_hz > (nyquist_hz - 10.0))
    {
        frequency_end_hz = nyquist_hz - 10.0;
    }
    if (frequency_start_hz < 20.0)
    {
        frequency_start_hz = 20.0;
    }
    if (frequency_end_hz <= frequency_start_hz)
    {
        frequency_end_hz = frequency_start_hz + 10.0;
    }

    result->late_dominant_frequency_a_hz = FindDominantFrequencyInRange(
        capture,
        analysis_channel_index,
        result->late_window_a_start_frame,
        result->late_window_a_end_frame,
        (f64) desc->sample_rate,
        frequency_start_hz,
        frequency_end_hz,
        1.0
    );
    result->late_dominant_frequency_b_hz = FindDominantFrequencyInRange(
        capture,
        analysis_channel_index,
        result->late_window_b_start_frame,
        result->late_window_b_end_frame,
        (f64) desc->sample_rate,
        frequency_start_hz,
        frequency_end_hz,
        1.0
    );
    result->dominant_frequency_drift_hz =
        fabs(result->late_dominant_frequency_b_hz - result->late_dominant_frequency_a_hz);
    result->dominant_frequency_drift_ratio = result->dominant_frequency_drift_hz /
        (fundamental_frequency_hz + 1e-12);

    ComputeSpectralQualityMetrics(
        capture,
        analysis_channel_index,
        result->late_start_frame,
        result->late_end_frame,
        (f64) desc->sample_rate,
        frequency_start_hz,
        frequency_end_hz,
        2.0,
        &result->late_spectral_centroid_hz,
        &result->late_spectral_flatness,
        &spectral_energy
    );
    (void) spectral_energy;

    result->late_harmonic_energy_ratio = ComputeHarmonicEnergyRatio(
        capture,
        desc,
        analysis_channel_index,
        result->late_start_frame,
        result->late_end_frame,
        (f64) desc->sample_rate,
        frequency_start_hz,
        frequency_end_hz
    );

    result->late_spectral_flux = ComputeSpectralFlux(
        capture,
        analysis_channel_index,
        result->late_window_a_start_frame,
        result->late_window_a_end_frame,
        result->late_window_b_start_frame,
        result->late_window_b_end_frame,
        (f64) desc->sample_rate,
        frequency_start_hz,
        frequency_end_hz,
        2.0
    );

    ComputeHarmonicDecayMetrics(
        capture,
        desc,
        analysis_channel_index,
        result->late_window_a_start_frame,
        result->late_window_a_end_frame,
        result->late_window_b_start_frame,
        result->late_window_b_end_frame,
        &result->harmonic_decay_low_band_db,
        &result->harmonic_decay_high_band_db,
        &result->harmonic_decay_slope_db_per_harmonic
    );
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

static bool RunVerification (
    const VerificationSettings *settings,
    VerificationRunSummary *summary,
    SimulationOfflineCapture *capture,
    Fdtd1DDesc *solver_desc,
    Fdtd1DProbeDesc *probe_descs,
    Fdtd1DSourceDesc *source_descs
)
{
    static const f64 IMPULSE_AMPLITUDE = 0.25;
    static const f64 SUSTAINED_DRIVE_AMPLITUDE = 0.001;
    static const u32 NOISE_BURST_FRAME_COUNT = 128;

    MemoryArena arena;
    Fdtd1D solver;
    Fdtd1DAreaSegmentDesc area_segment_descs[2];
    Simulation *simulation;
    SimulationExcitation excitation;
    const Fdtd1DState *state;
    usize capture_sample_count;
    u32 block_index;
    bool result;
    f32 *persistent_capture_samples;

    ASSERT(settings != NULL);
    ASSERT(summary != NULL);
    ASSERT(capture != NULL);
    ASSERT(solver_desc != NULL);
    ASSERT(probe_descs != NULL);
    ASSERT(source_descs != NULL);

    result = false;
    persistent_capture_samples = NULL;

    memset(&arena, 0, sizeof(arena));
    memset(&solver, 0, sizeof(solver));
    memset(solver_desc, 0, sizeof(*solver_desc));
    memset(capture, 0, sizeof(*capture));
    memset(&excitation, 0, sizeof(excitation));
    memset(summary, 0, sizeof(*summary));

    if (MemoryArena_Create(&arena, 16 * 1024 * 1024) == false)
    {
        fprintf(stderr, "Failed to create verification arena.\n");
        return false;
    }

    ConfigureSolver(settings, solver_desc, area_segment_descs, probe_descs, source_descs);

    if (Fdtd1D_Initialize(&solver, &arena, solver_desc) == false)
    {
        fprintf(stderr, "Failed to initialize 1D FDTD solver.\n");
        goto cleanup;
    }

    if (Fdtd1D_SetNonlinearMouthParameters(&solver, &settings->nonlinear_mouth_parameters) == false)
    {
        fprintf(stderr, "Failed to configure nonlinear mouth parameters.\n");
        goto cleanup;
    }

    simulation = Fdtd1D_GetSimulation(&solver);

    capture->channel_count = solver_desc->output_channel_count;
    capture->frame_capacity = solver_desc->block_frame_count * settings->block_count;
    capture->frame_count = 0;
    capture_sample_count = (usize) capture->frame_capacity * (usize) capture->channel_count;
    capture->samples = MEMORY_ARENA_PUSH_ARRAY(&arena, capture_sample_count, f32);
    if (capture->samples == NULL)
    {
        fprintf(stderr, "Failed to allocate capture buffer.\n");
        goto cleanup;
    }

    excitation.target_index = 0;
    switch (settings->excitation_type)
    {
        case VERIFICATION_EXCITATION_TYPE_IMPULSE:
        {
            excitation.type = SIMULATION_EXCITATION_TYPE_IMPULSE;
            excitation.remaining_frame_count = 1;
            excitation.value = IMPULSE_AMPLITUDE;
        } break;

        case VERIFICATION_EXCITATION_TYPE_NOISE_BURST:
        {
            excitation.type = SIMULATION_EXCITATION_TYPE_NOISE;
            excitation.remaining_frame_count = NOISE_BURST_FRAME_COUNT;
            excitation.value = IMPULSE_AMPLITUDE;
        } break;

        case VERIFICATION_EXCITATION_TYPE_CONSTANT:
        {
            excitation.type = SIMULATION_EXCITATION_TYPE_VELOCITY_CONSTANT;
            excitation.remaining_frame_count = settings->block_count * solver_desc->block_frame_count;
            excitation.value = SUSTAINED_DRIVE_AMPLITUDE;
        } break;

        case VERIFICATION_EXCITATION_TYPE_NONLINEAR_MOUTH:
        {
            excitation.type = SIMULATION_EXCITATION_TYPE_NONLINEAR_MOUTH;
            excitation.remaining_frame_count = settings->block_count * solver_desc->block_frame_count;
            excitation.value = SUSTAINED_DRIVE_AMPLITUDE;
        } break;

        case VERIFICATION_EXCITATION_TYPE_JET_LABIUM:
        {
            excitation.type = SIMULATION_EXCITATION_TYPE_JET_LABIUM;
            excitation.remaining_frame_count = settings->block_count * solver_desc->block_frame_count;
            excitation.value = SUSTAINED_DRIVE_AMPLITUDE;
        } break;
    }
    excitation.is_active = true;

    if (Simulation_QueueExcitation(simulation, &excitation) == false)
    {
        fprintf(stderr, "Failed to queue verification impulse.\n");
        goto cleanup;
    }

    for (block_index = 0; block_index < settings->block_count; block_index += 1)
    {
        if (Simulation_ProcessBlock(simulation) == false)
        {
            fprintf(stderr, "Offline solver run failed.\n");
            goto cleanup;
        }

        if (Simulation_CaptureOutput(simulation, capture) == false)
        {
            fprintf(stderr, "Failed to capture offline output block.\n");
            goto cleanup;
        }

        state = Fdtd1D_GetState(&solver);
        if (state != NULL)
        {
            f64 block_energy;

            block_energy = ComputeStateEnergy(state);
            if (block_index == 0)
            {
                summary->energy.initial_energy = block_energy;
                summary->energy.minimum_energy = block_energy;
                summary->energy.maximum_energy = block_energy;
            }
            else
            {
                if (block_energy < summary->energy.minimum_energy)
                {
                    summary->energy.minimum_energy = block_energy;
                }

                if (block_energy > summary->energy.maximum_energy)
                {
                    summary->energy.maximum_energy = block_energy;
                }
            }

            summary->energy.final_energy = block_energy;
        }
    }

    AnalyzeModeSeries(
        capture,
        solver_desc,
        0,
        solver_desc->block_frame_count,
        MODE_COUNT,
        summary->mode_results
    );
    AnalyzeSustainedBehavior(capture, solver_desc, 0, &summary->sustained);
    AnalyzeSpeechBehavior(capture, solver_desc, 0, &summary->speech);

    state = Fdtd1D_GetState(&solver);
    if ((state != NULL) && (state->boundary_emission_sample_count > 0))
    {
        f64 inverse_sample_count;

        inverse_sample_count = 1.0 / (f64) state->boundary_emission_sample_count;
        summary->left_boundary_emission_rms = sqrt(
            state->left_boundary_emission_sum_squares * inverse_sample_count
        );
        summary->right_boundary_emission_rms = sqrt(
            state->right_boundary_emission_sum_squares * inverse_sample_count
        );
        summary->left_boundary_emission_peak_abs = state->left_boundary_emission_peak_abs;
        summary->right_boundary_emission_peak_abs = state->right_boundary_emission_peak_abs;
        summary->left_boundary_emission_activity_ok = EvaluateBoundaryEmissionActivityGate(
            solver_desc->left_boundary.type,
            summary->left_boundary_emission_rms
        );
        summary->right_boundary_emission_activity_ok = EvaluateBoundaryEmissionActivityGate(
            solver_desc->right_boundary.type,
            summary->right_boundary_emission_rms
        );
    }

    summary->stats = *Simulation_GetStats(simulation);
    summary->pressure_cell_count = solver_desc->pressure_cell_count;
    summary->tube_length_m = solver_desc->tube_length_m;
    persistent_capture_samples = (f32 *) malloc(sizeof(f32) * capture_sample_count);
    if (persistent_capture_samples == NULL)
    {
        fprintf(stderr, "Failed to allocate persistent capture buffer.\n");
        goto cleanup;
    }

    memcpy(persistent_capture_samples, capture->samples, sizeof(f32) * capture_sample_count);
    capture->samples = persistent_capture_samples;
    result = true;

cleanup:
    Fdtd1D_Shutdown(&solver);
    MemoryArena_Destroy(&arena);
    if (result == false)
    {
        free(persistent_capture_samples);
    }
    return result;
}

int main (int argc, char **argv)
{
    static const f32 FIRST_ARRIVAL_THRESHOLD = 0.001f;
    static const u32 WINDOW_RADIUS = 4;

    VerificationSettings settings;
    VerificationRunSummary summary;
    LengthSweepResult sweep_results[32];
    NonlinearMouthSweepResult nonlinear_mouth_sweep_results[32];
    Fdtd1DDesc solver_desc;
    Fdtd1DProbeDesc probe_descs[2];
    Fdtd1DSourceDesc source_descs[1];
    SimulationOfflineCapture capture;
    ArrivalSummary left_arrival;
    ArrivalSummary right_arrival;
    WindowPeakResult left_reflection_window;
    WindowPeakResult right_reflection_window;
    WindowEnergyResult left_reflection_energy;
    WindowEnergyResult right_reflection_energy;
    ModeResult mode_results[MODE_COUNT];
    f64 expected_left_arrival_seconds;
    f64 expected_right_arrival_seconds;
    f64 expected_left_arrival_frames;
    f64 expected_right_arrival_frames;
    f64 expected_left_reflection_frames;
    f64 expected_right_reflection_frames;
    u32 length_cell_count;
    u32 sweep_result_count;
    u32 block_index;

    InitializeVerificationSettings(&settings);
    ParseArguments(argc, argv, &settings);

    if (settings.run_length_sweep)
    {
        sweep_result_count = 0;
        for (length_cell_count = settings.length_sweep_start_cell_count;
             length_cell_count <= settings.length_sweep_end_cell_count;
             length_cell_count += settings.length_sweep_step_cell_count)
        {
            VerificationSettings sweep_settings;

            if (sweep_result_count >= ARRAY_COUNT(sweep_results))
            {
                break;
            }

            sweep_settings = settings;
            sweep_settings.run_length_sweep = false;
            sweep_settings.pressure_cell_count = length_cell_count;

            if (RunVerification(&sweep_settings, &summary, &capture, &solver_desc, probe_descs, source_descs) == false)
            {
                return 1;
            }

            sweep_results[sweep_result_count].pressure_cell_count = summary.pressure_cell_count;
            sweep_results[sweep_result_count].tube_length_m = summary.tube_length_m;
            sweep_results[sweep_result_count].expected_fundamental_hz =
                summary.mode_results[0].expected_frequency_hz;
            sweep_results[sweep_result_count].measured_fundamental_hz =
                summary.mode_results[0].measured_peak_frequency_hz;
            sweep_results[sweep_result_count].measured_fundamental_magnitude =
                summary.mode_results[0].measured_peak_magnitude;
            sweep_results[sweep_result_count].final_energy_drift =
                summary.energy.final_energy - summary.energy.initial_energy;
            sweep_result_count += 1;
            free(capture.samples);
            capture.samples = NULL;
        }

        printf("FDTD 1D Length Sweep\n\n");
        printf("Setup\n");
        printf("  preset:                 %s\n", GetPresetName(settings.preset));
        printf("  excitation:             %s\n", GetExcitationTypeName(settings.excitation_type));
        printf("  probe_type:             %s\n", GetProbeTypeName(settings.probe_type));
        printf("  blocks:                 %u\n", settings.block_count);
        printf("  sweep_start_cells:      %u\n", settings.length_sweep_start_cell_count);
        printf("  sweep_end_cells:        %u\n", settings.length_sweep_end_cell_count);
        printf("  sweep_step_cells:       %u\n", settings.length_sweep_step_cell_count);
        printf("\n");
        printf("Length Sweep Results\n");
        printf("  cells    length_m   expected_f0   measured_f0   delta_hz   delta_pct   magnitude    energy_drift\n");
        for (block_index = 0; block_index < sweep_result_count; block_index += 1)
        {
            f64 delta_hz;
            f64 delta_percent;

            delta_hz = sweep_results[block_index].measured_fundamental_hz -
                sweep_results[block_index].expected_fundamental_hz;
            delta_percent = 100.0 * (delta_hz / sweep_results[block_index].expected_fundamental_hz);

            printf(
                "  %-8u %-10.6f %-13.2f %-13.2f %-10.2f %-10.2f %-12.6f %.9f\n",
                sweep_results[block_index].pressure_cell_count,
                sweep_results[block_index].tube_length_m,
                sweep_results[block_index].expected_fundamental_hz,
                sweep_results[block_index].measured_fundamental_hz,
                delta_hz,
                delta_percent,
                sweep_results[block_index].measured_fundamental_magnitude,
                sweep_results[block_index].final_energy_drift
            );
        }

        return 0;
    }

    if (settings.run_nonlinear_mouth_sweep)
    {
        f64 feedback_scale;

        sweep_result_count = 0;
        for (feedback_scale = settings.nonlinear_mouth_feedback_scale_start;
             feedback_scale <= settings.nonlinear_mouth_feedback_scale_end;
             feedback_scale += settings.nonlinear_mouth_feedback_scale_step)
        {
            VerificationSettings sweep_settings;

            if (sweep_result_count >= ARRAY_COUNT(nonlinear_mouth_sweep_results))
            {
                break;
            }

            sweep_settings = settings;
            sweep_settings.run_nonlinear_mouth_sweep = false;
            sweep_settings.excitation_type = VERIFICATION_EXCITATION_TYPE_NONLINEAR_MOUTH;
            sweep_settings.nonlinear_mouth_parameters.pressure_feedback =
                settings.nonlinear_mouth_parameters.pressure_feedback * (f32) feedback_scale;
            sweep_settings.nonlinear_mouth_parameters.velocity_feedback =
                settings.nonlinear_mouth_parameters.velocity_feedback * (f32) feedback_scale;

            if (RunVerification(&sweep_settings, &summary, &capture, &solver_desc, probe_descs, source_descs) == false)
            {
                return 1;
            }

            nonlinear_mouth_sweep_results[sweep_result_count].feedback_scale = feedback_scale;
            nonlinear_mouth_sweep_results[sweep_result_count].early_rms =
                summary.sustained.early_rms;
            nonlinear_mouth_sweep_results[sweep_result_count].late_to_early_rms_ratio =
                summary.sustained.late_to_early_rms_ratio;
            nonlinear_mouth_sweep_results[sweep_result_count].dominant_frequency_drift_hz =
                summary.sustained.dominant_frequency_drift_hz;
            nonlinear_mouth_sweep_results[sweep_result_count].late_spectral_centroid_hz =
                summary.sustained.late_spectral_centroid_hz;
            nonlinear_mouth_sweep_results[sweep_result_count].late_spectral_flatness =
                summary.sustained.late_spectral_flatness;
            nonlinear_mouth_sweep_results[sweep_result_count].late_harmonic_energy_ratio =
                summary.sustained.late_harmonic_energy_ratio;
            nonlinear_mouth_sweep_results[sweep_result_count].late_spectral_flux =
                summary.sustained.late_spectral_flux;
            nonlinear_mouth_sweep_results[sweep_result_count].max_output_abs =
                summary.stats.max_abs_output;
            nonlinear_mouth_sweep_results[sweep_result_count].energy_drift =
                summary.energy.final_energy - summary.energy.initial_energy;
            nonlinear_mouth_sweep_results[sweep_result_count].early_rms_ok =
                EvaluateEarlyRmsGate(nonlinear_mouth_sweep_results[sweep_result_count].early_rms);
            nonlinear_mouth_sweep_results[sweep_result_count].late_to_early_rms_ok =
                EvaluateLateToEarlyRmsGate(nonlinear_mouth_sweep_results[sweep_result_count].late_to_early_rms_ratio);
            nonlinear_mouth_sweep_results[sweep_result_count].dominant_frequency_drift_ok =
                EvaluateDominantFrequencyDriftGate(
                    summary.sustained.dominant_frequency_drift_ratio
                );
            nonlinear_mouth_sweep_results[sweep_result_count].energy_drift_ok =
                EvaluateEnergyDriftGate(nonlinear_mouth_sweep_results[sweep_result_count].energy_drift);
            nonlinear_mouth_sweep_results[sweep_result_count].is_stable_candidate =
                nonlinear_mouth_sweep_results[sweep_result_count].early_rms_ok &&
                nonlinear_mouth_sweep_results[sweep_result_count].late_to_early_rms_ok &&
                nonlinear_mouth_sweep_results[sweep_result_count].dominant_frequency_drift_ok &&
                nonlinear_mouth_sweep_results[sweep_result_count].energy_drift_ok;
            sweep_result_count += 1;
            free(capture.samples);
            capture.samples = NULL;
        }

        printf("FDTD 1D Nonlinear Mouth Sweep\n\n");
        printf("Setup\n");
        printf("  preset:                 %s\n", GetPresetName(settings.preset));
        printf("  probe_type:             %s\n", GetProbeTypeName(settings.probe_type));
        printf("  blocks:                 %u\n", settings.block_count);
        printf("  feedback_scale_start:   %.3f\n", settings.nonlinear_mouth_feedback_scale_start);
        printf("  feedback_scale_end:     %.3f\n", settings.nonlinear_mouth_feedback_scale_end);
        printf("  feedback_scale_step:    %.3f\n", settings.nonlinear_mouth_feedback_scale_step);
        printf("  mouth_max_output:       %.6f\n", settings.nonlinear_mouth_parameters.max_output);
        printf("  mouth_noise_scale:      %.6f\n", settings.nonlinear_mouth_parameters.noise_scale);
        printf("  mouth_feedback_leak:    %.6f\n", settings.nonlinear_mouth_parameters.feedback_leak);
        printf("  mouth_saturation_gain:  %.3f\n", settings.nonlinear_mouth_parameters.saturation_gain);
        printf("  mouth_drive_limit:      %.6f\n", settings.nonlinear_mouth_parameters.drive_limit);
        printf("  mouth_delay:            %u\n", settings.nonlinear_mouth_parameters.delay_samples);
        printf("\n");
        printf("Gates\n");
        printf("  early_rms_min:          0.01\n");
        printf("  late/early_rms_ratio:   [0.5, 1.5]\n");
        printf("  dominant_freq_drift_ratio: <= 0.12\n");
        printf("  abs_energy_drift:       <= 1.0\n");
        printf("\n");
        printf("Sweep Results\n");
        printf("  scale    early_rms  late/early_rms   freq_drift_hz   centroid_hz   harmonic_ratio   flatness    flux       max_output_abs   energy_drift    stable\n");
        for (block_index = 0; block_index < sweep_result_count; block_index += 1)
        {
            printf(
                "  %-8.3f %-10.6f %-15.6f %-15.3f %-12.2f %-15.6f %-11.6f %-10.6f %-15.6f %-13.9f %s\n",
                nonlinear_mouth_sweep_results[block_index].feedback_scale,
                nonlinear_mouth_sweep_results[block_index].early_rms,
                nonlinear_mouth_sweep_results[block_index].late_to_early_rms_ratio,
                nonlinear_mouth_sweep_results[block_index].dominant_frequency_drift_hz,
                nonlinear_mouth_sweep_results[block_index].late_spectral_centroid_hz,
                nonlinear_mouth_sweep_results[block_index].late_harmonic_energy_ratio,
                nonlinear_mouth_sweep_results[block_index].late_spectral_flatness,
                nonlinear_mouth_sweep_results[block_index].late_spectral_flux,
                nonlinear_mouth_sweep_results[block_index].max_output_abs,
                nonlinear_mouth_sweep_results[block_index].energy_drift,
                nonlinear_mouth_sweep_results[block_index].is_stable_candidate ? "yes" : "no"
            );
        }

        return 0;
    }

    if (RunVerification(&settings, &summary, &capture, &solver_desc, probe_descs, source_descs) == false)
    {
        return 1;
    }

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
    memcpy(mode_results, summary.mode_results, sizeof(mode_results));

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
        solver_desc.courant_number
    );
    printf("  uniform_loss:           %.6f\n",
        solver_desc.uniform_loss
    );
    printf("  hf_loss:                %.6f\n",
        solver_desc.uniform_high_frequency_loss
    );
    printf("  boundary_loss:          %.6f\n",
        solver_desc.uniform_boundary_loss
    );
    printf("  boundary_hf_loss:       %.6f\n",
        solver_desc.uniform_boundary_high_frequency_loss
    );
    printf("  area_loss_reference:    %.6f\n",
        solver_desc.area_loss_reference_m2
    );
    printf("  area_loss_strength:     %.6f\n",
        solver_desc.area_loss_strength
    );
    printf("  open_end_correction:    %.6f\n",
        solver_desc.open_end_correction_coefficient
    );
    printf("  open_end_rad_resist:    %.6f\n",
        solver_desc.open_end_radiation_resistance_scale
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
    printf("  speech_analysis:       %s\n",
        settings.run_speech_analysis ? "enabled" : "disabled"
    );
    printf("  mouth_feedback_leak:   %.6f\n",
        settings.nonlinear_mouth_parameters.feedback_leak
    );
    printf("  source_index:          %u\n",
        source_descs[0].cell_index
    );
    printf("  left_probe_index:      %u\n",
        probe_descs[0].cell_index
    );
    printf("  right_probe_index:     %u\n",
        probe_descs[1].cell_index
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
        (unsigned long long) summary.stats.processed_block_count
    );
    printf("  processed frames:       %llu\n",
        (unsigned long long) summary.stats.processed_frame_count
    );
    printf("  processed seconds:      %.6f\n",
        summary.stats.processed_seconds
    );
    printf("  max output abs:         %.6f\n",
        summary.stats.max_abs_output
    );
    printf("  max probe abs:          %.6f\n",
        summary.stats.max_abs_probe
    );
    printf("  saw NaN:                %s\n",
        summary.stats.saw_nan ? "yes" : "no"
    );
    printf("  saw Inf:                %s\n",
        summary.stats.saw_inf ? "yes" : "no"
    );
    printf("\n");

    printf("Boundary Emission\n");
    printf("  left rms:               %.9f\n",
        summary.left_boundary_emission_rms
    );
    printf("  left peak abs:          %.9f\n",
        summary.left_boundary_emission_peak_abs
    );
    printf("  left activity gate:     %s\n",
        summary.left_boundary_emission_activity_ok ? "pass" : "warn"
    );
    printf("  right rms:              %.9f\n",
        summary.right_boundary_emission_rms
    );
    printf("  right peak abs:         %.9f\n",
        summary.right_boundary_emission_peak_abs
    );
    printf("  right activity gate:    %s\n",
        summary.right_boundary_emission_activity_ok ? "pass" : "warn"
    );
    printf("\n");

    printf("Energy\n");
    printf("  initial:                %.9f\n",
        summary.energy.initial_energy
    );
    printf("  minimum:                %.9f\n",
        summary.energy.minimum_energy
    );
    printf("  maximum:                %.9f\n",
        summary.energy.maximum_energy
    );
    printf("  final:                  %.9f\n",
        summary.energy.final_energy
    );
    printf("  drift:                  %.9f\n",
        summary.energy.final_energy - summary.energy.initial_energy
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

    printf("Sustained Analysis\n");
    printf("  early_rms_window:       [%u, %u]\n",
        summary.sustained.early_start_frame,
        summary.sustained.early_end_frame
    );
    printf("  late_rms_window:        [%u, %u]\n",
        summary.sustained.late_start_frame,
        summary.sustained.late_end_frame
    );
    printf("  early_rms:              %.9f\n",
        summary.sustained.early_rms
    );
    printf("  late_rms:               %.9f\n",
        summary.sustained.late_rms
    );
    printf("  late/early_rms_ratio:   %.6f\n",
        summary.sustained.late_to_early_rms_ratio
    );
    printf("  late_freq_window_a:     [%u, %u]\n",
        summary.sustained.late_window_a_start_frame,
        summary.sustained.late_window_a_end_frame
    );
    printf("  late_freq_window_b:     [%u, %u]\n",
        summary.sustained.late_window_b_start_frame,
        summary.sustained.late_window_b_end_frame
    );
    printf("  dominant_freq_a_hz:     %.2f\n",
        summary.sustained.late_dominant_frequency_a_hz
    );
    printf("  dominant_freq_b_hz:     %.2f\n",
        summary.sustained.late_dominant_frequency_b_hz
    );
    printf("  dominant_freq_drift_hz: %.2f\n",
        summary.sustained.dominant_frequency_drift_hz
    );
    printf("  dominant_freq_drift_ratio: %.6f\n",
        summary.sustained.dominant_frequency_drift_ratio
    );
    printf("  late_spectral_centroid: %.2f\n",
        summary.sustained.late_spectral_centroid_hz
    );
    printf("  late_harmonic_ratio:    %.6f\n",
        summary.sustained.late_harmonic_energy_ratio
    );
    printf("  harmonic_late_delta_low_db:  %.3f\n",
        summary.sustained.harmonic_decay_low_band_db
    );
    printf("  harmonic_late_delta_high_db: %.3f\n",
        summary.sustained.harmonic_decay_high_band_db
    );
    printf("  harmonic_late_delta_slope:   %.3f dB/harmonic\n",
        summary.sustained.harmonic_decay_slope_db_per_harmonic
    );
    printf("  late_spectral_flatness: %.6f\n",
        summary.sustained.late_spectral_flatness
    );
    printf("  late_spectral_flux:     %.6f\n",
        summary.sustained.late_spectral_flux
    );
    printf("  early_rms_gate:         %s\n",
        EvaluateEarlyRmsGate(summary.sustained.early_rms) ? "pass" : "fail"
    );
    printf("  late/early_rms_gate:    %s\n",
        EvaluateLateToEarlyRmsGate(summary.sustained.late_to_early_rms_ratio) ? "pass" : "fail"
    );
    printf("  freq_drift_gate:        %s\n",
        EvaluateDominantFrequencyDriftGate(summary.sustained.dominant_frequency_drift_ratio) ? "pass" : "fail"
    );
    printf("  energy_drift_gate:      %s\n",
        EvaluateEnergyDriftGate(summary.energy.final_energy - summary.energy.initial_energy) ? "pass" : "fail"
    );
    printf("\n");

    if (settings.run_speech_analysis)
    {
        printf("Speech/Onset Analysis\n");
        printf("  analysis_channel:       0\n");
        if (summary.speech.has_signal == false)
        {
            printf("  signal_detected:        no\n");
            printf("\n");
        }
        else
        {
            printf("  signal_detected:        yes\n");
            printf("  peak_abs:               %.9f\n",
                summary.speech.peak_abs
            );
            printf("  onset_frame:            %u\n",
                summary.speech.onset_frame
            );
            printf("  peak_frame:             %u\n",
                summary.speech.peak_frame
            );
            printf("  onset_to_peak_ms:       %.3f\n",
                summary.speech.onset_to_peak_ms
            );
            printf("  attack_10_frame:        %u\n",
                summary.speech.attack_10_frame
            );
            printf("  attack_90_frame:        %u\n",
                summary.speech.attack_90_frame
            );
            printf("  attack_10_to_90_ms:     ");
            if (summary.speech.attack_was_found)
            {
                printf("%.3f\n", summary.speech.attack_10_to_90_ms);
            }
            else
            {
                printf("not found\n");
            }
            printf("  chiff_window:           [%u, %u]\n",
                summary.speech.chiff_start_frame,
                summary.speech.chiff_end_frame
            );
            printf("  sustain_window:         [%u, %u]\n",
                summary.speech.sustain_start_frame,
                summary.speech.sustain_end_frame
            );
            printf("  chiff_rms:              %.9f\n",
                summary.speech.chiff_rms
            );
            printf("  sustain_rms:            %.9f\n",
                summary.speech.sustain_rms
            );
            printf("  chiff/sustain_rms:      %.6f\n",
                summary.speech.chiff_to_sustain_rms_ratio
            );
            printf("  settling_frame:         ");
            if (summary.speech.settling_was_found)
            {
                printf("%u\n", summary.speech.settle_frame);
            }
            else
            {
                printf("not found\n");
            }
            printf("  settling_time_ms:       ");
            if (summary.speech.settling_was_found)
            {
                printf("%.3f\n", summary.speech.settling_time_ms);
            }
            else
            {
                printf("not found\n");
            }
            printf("  onset_to_peak_gate:     %s\n",
                EvaluateSpeechOnsetToPeakGate(summary.speech.onset_to_peak_ms) ? "pass" : "fail"
            );
            printf("  attack_10_to_90_gate:   %s\n",
                EvaluateSpeechAttackGate(summary.speech.attack_was_found, summary.speech.attack_10_to_90_ms) ? "pass" : "fail"
            );
            printf("  chiff/sustain_gate:     %s\n",
                EvaluateSpeechChiffToSustainGate(summary.speech.chiff_to_sustain_rms_ratio) ? "pass" : "fail"
            );
            printf("  settling_gate:          %s\n",
                summary.speech.settling_was_found ? "pass" : "warn"
            );
            printf("\n");
        }
    }

    if (settings.csv_output_path != NULL)
    {
        if (WriteCaptureCsv(settings.csv_output_path, &capture))
        {
            printf("csv_written=%s\n", settings.csv_output_path);
        }
        else
        {
            fprintf(stderr, "Failed to write CSV output: %s\n", settings.csv_output_path);
            return 1;
        }
    }

    free(capture.samples);

    return 0;
}
