#ifndef PM_ORGAN_SIM_FDTD_1D_RENDER_SOURCE_H
#define PM_ORGAN_SIM_FDTD_1D_RENDER_SOURCE_H

#include "pm_organ/core/memory_arena.h"
#include "pm_organ/core/types.h"
#include "pm_organ/sim/fdtd_1d.h"

typedef enum Fdtd1DOutputExtractionMode
{
    FDTD_1D_OUTPUT_EXTRACTION_MODE_RAW_PROBES = 0,
    FDTD_1D_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION,
    FDTD_1D_OUTPUT_EXTRACTION_MODE_LISTENER_MODEL,
} Fdtd1DOutputExtractionMode;

typedef enum Fdtd1DExcitationMode
{
    FDTD_1D_EXCITATION_MODE_IMPULSE = 0,
    FDTD_1D_EXCITATION_MODE_CONSTANT,
    FDTD_1D_EXCITATION_MODE_NOISE,
    FDTD_1D_EXCITATION_MODE_BIAS_AND_NOISE,
    FDTD_1D_EXCITATION_MODE_FEEDBACK_MOUTH,
    FDTD_1D_EXCITATION_MODE_NONLINEAR_MOUTH,
    FDTD_1D_EXCITATION_MODE_JET_LABIUM,
} Fdtd1DExcitationMode;

typedef enum Fdtd1DSourceCouplingMode
{
    FDTD_1D_SOURCE_COUPLING_MODE_PRESSURE = 0,
    FDTD_1D_SOURCE_COUPLING_MODE_VELOCITY,
} Fdtd1DSourceCouplingMode;

typedef struct Fdtd1DRenderSourceDesc
{
    Fdtd1DDesc solver_desc;
    Fdtd1DExcitationMode excitation_mode;
    Fdtd1DSourceCouplingMode source_coupling_mode;
    f64 drive_amplitude;
    f64 windchest_pressure;
    f64 speech_attack_seconds;
    f64 speech_chiff_amount;
    f64 speech_chiff_decay_seconds;
    f64 listener_distance_m;
    f64 listener_mouth_pressure_mix;
    f64 listener_crossfeed;
    f64 listener_lowpass_cutoff_hz;
    Fdtd1DOutputExtractionMode output_extraction_mode;
    bool startup_impulse_is_enabled;
    u32 startup_impulse_target_index;
    f64 startup_impulse_amplitude;
} Fdtd1DRenderSourceDesc;

typedef struct Fdtd1DRenderSource
{
    Fdtd1D solver;
    Fdtd1DExcitationMode excitation_mode;
    Fdtd1DSourceCouplingMode source_coupling_mode;
    f64 drive_amplitude;
    f64 smoothed_drive_amplitude;
    f64 windchest_pressure;
    f64 smoothed_windchest_pressure;
    f64 speech_attack_seconds;
    f64 speech_chiff_amount;
    f64 speech_chiff_decay_seconds;
    f64 listener_distance_m;
    f64 listener_mouth_pressure_mix;
    f64 listener_crossfeed;
    f64 listener_lowpass_cutoff_hz;
    f64 speech_gate;
    f64 speech_onset_seconds;
    bool speech_is_active;
    f64 last_requested_drive;
    f64 last_applied_drive;
    f64 last_drive_saturation_ratio;
    f32 listener_left_state;
    f32 listener_right_state;
    Fdtd1DOutputExtractionMode output_extraction_mode;
    bool startup_impulse_is_pending;
    u32 startup_impulse_target_index;
    f64 startup_impulse_amplitude;
} Fdtd1DRenderSource;

bool Fdtd1DRenderSource_Initialize (
    Fdtd1DRenderSource *source,
    MemoryArena *arena,
    const Fdtd1DRenderSourceDesc *desc
);
void Fdtd1DRenderSource_Shutdown (Fdtd1DRenderSource *source);
void Fdtd1DRenderSource_TriggerStartupImpulse (Fdtd1DRenderSource *source);
void Fdtd1DRenderSource_SetOutputExtractionMode (
    Fdtd1DRenderSource *source,
    Fdtd1DOutputExtractionMode output_extraction_mode
);
void Fdtd1DRenderSource_SetExcitationMode (
    Fdtd1DRenderSource *source,
    Fdtd1DExcitationMode excitation_mode
);
void Fdtd1DRenderSource_SetSourceCouplingMode (
    Fdtd1DRenderSource *source,
    Fdtd1DSourceCouplingMode source_coupling_mode
);
void Fdtd1DRenderSource_SetDriveAmplitude (Fdtd1DRenderSource *source, f64 drive_amplitude);
void Fdtd1DRenderSource_SetWindchestPressure (Fdtd1DRenderSource *source, f64 windchest_pressure);
void Fdtd1DRenderSource_SetSpeechAttackSeconds (Fdtd1DRenderSource *source, f64 speech_attack_seconds);
void Fdtd1DRenderSource_SetSpeechChiffAmount (Fdtd1DRenderSource *source, f64 speech_chiff_amount);
void Fdtd1DRenderSource_SetSpeechChiffDecaySeconds (
    Fdtd1DRenderSource *source,
    f64 speech_chiff_decay_seconds
);
void Fdtd1DRenderSource_SetListenerDistance (Fdtd1DRenderSource *source, f64 listener_distance_m);
void Fdtd1DRenderSource_SetListenerMouthPressureMix (Fdtd1DRenderSource *source, f64 listener_mouth_pressure_mix);
void Fdtd1DRenderSource_SetListenerCrossfeed (Fdtd1DRenderSource *source, f64 listener_crossfeed);
void Fdtd1DRenderSource_SetListenerLowpassCutoff (
    Fdtd1DRenderSource *source,
    f64 listener_lowpass_cutoff_hz
);
void Fdtd1DRenderSource_RestartSpeech (Fdtd1DRenderSource *source);
void Fdtd1DRenderSource_Render (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
);

#endif // PM_ORGAN_SIM_FDTD_1D_RENDER_SOURCE_H
