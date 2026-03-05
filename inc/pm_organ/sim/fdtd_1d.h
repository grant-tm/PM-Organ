#ifndef PM_ORGAN_SIM_FDTD_1D_H
#define PM_ORGAN_SIM_FDTD_1D_H

#include "pm_organ/core/memory_arena.h"
#include "pm_organ/core/types.h"
#include "pm_organ/sim/simulation.h"

typedef enum Fdtd1DBoundaryType
{
    FDTD_1D_BOUNDARY_TYPE_RIGID = 0,
    FDTD_1D_BOUNDARY_TYPE_OPEN,
    FDTD_1D_BOUNDARY_TYPE_REFLECTION_COEFFICIENT,
} Fdtd1DBoundaryType;

typedef struct Fdtd1DBoundaryDesc
{
    Fdtd1DBoundaryType type;
    f64 reflection_coefficient;
} Fdtd1DBoundaryDesc;

typedef enum Fdtd1DProbeType
{
    FDTD_1D_PROBE_TYPE_PRESSURE = 0,
    FDTD_1D_PROBE_TYPE_VELOCITY,
    FDTD_1D_PROBE_TYPE_LEFT_BOUNDARY_EMISSION,
    FDTD_1D_PROBE_TYPE_RIGHT_BOUNDARY_EMISSION,
} Fdtd1DProbeType;

typedef struct Fdtd1DProbeDesc
{
    Fdtd1DProbeType type;
    u32 cell_index;
    u32 output_channel_index;
    bool is_enabled;
} Fdtd1DProbeDesc;

typedef struct Fdtd1DSourceDesc
{
    u32 cell_index;
    bool is_enabled;
} Fdtd1DSourceDesc;

typedef struct Fdtd1DAreaSegmentDesc
{
    u32 start_cell_index;
    u32 end_cell_index;
    f64 area_m2;
} Fdtd1DAreaSegmentDesc;

typedef struct Fdtd1DNonlinearMouthParameters
{
    f32 max_output;
    f32 noise_scale;
    f32 pressure_feedback;
    f32 velocity_feedback;
    f32 feedback_leak;
    f32 saturation_gain;
    f32 drive_limit;
    u32 delay_samples;
} Fdtd1DNonlinearMouthParameters;

typedef struct Fdtd1DJetLabiumParameters
{
    f32 max_output;
    f32 noise_scale;
    f32 pressure_feedback;
    f32 velocity_feedback;
    f32 feedback_leak;
    f32 jet_smoothing;
    f32 labium_split_gain;
    f32 saturation_gain;
    f32 drive_limit;
    u32 delay_samples;
} Fdtd1DJetLabiumParameters;

typedef struct Fdtd1DDesc
{
    u32 sample_rate;
    u32 block_frame_count;
    u32 output_channel_count;

    f64 tube_length_m;
    f64 wave_speed_m_per_s;
    f64 density_kg_per_m3;
    f64 dx;

    u32 pressure_cell_count;
    u32 velocity_cell_count;
    f64 courant_number;

    f64 uniform_area_m2;
    f64 uniform_loss;
    f64 uniform_high_frequency_loss;
    f64 uniform_boundary_loss;
    f64 uniform_boundary_high_frequency_loss;
    f64 area_loss_reference_m2;
    f64 area_loss_strength;
    f64 open_end_correction_coefficient;
    f64 open_end_radiation_resistance_scale;

    u32 area_segment_count;
    const Fdtd1DAreaSegmentDesc *area_segment_descs;

    Fdtd1DBoundaryDesc left_boundary;
    Fdtd1DBoundaryDesc right_boundary;

    u32 probe_count;
    const Fdtd1DProbeDesc *probe_descs;

    u32 source_count;
    const Fdtd1DSourceDesc *source_descs;
} Fdtd1DDesc;

typedef struct Fdtd1DState
{
    u32 pressure_cell_count;
    u32 velocity_cell_count;
    u32 probe_count;
    u32 source_count;

    f64 dt;
    f64 dx;
    f64 wave_speed_m_per_s;
    f64 density_kg_per_m3;
    f64 courant_number;
    f64 characteristic_impedance;

    f32 *pressure;
    f32 *velocity;

    f32 *area_pressure;
    f32 *area_velocity;

    f32 *pressure_loss;
    f32 *velocity_loss;
    f32 *pressure_previous;
    f32 *velocity_previous;
    f32 pressure_high_frequency_loss;
    f32 velocity_high_frequency_loss;
    f32 boundary_loss;
    f32 boundary_high_frequency_loss;

    f32 *pressure_update_coeff;
    f32 *velocity_update_coeff;

    Fdtd1DProbeType *probe_types;
    u32 *probe_cell_indices;
    u32 *probe_output_channels;

    u32 *source_cell_indices;
    f32 *source_jet_state;
    f32 *source_jet_regime_state;
    f32 *mouth_feedback_delay_buffer;
    u32 *mouth_feedback_delay_lengths;
    u32 *mouth_feedback_delay_indices;
    u32 mouth_feedback_delay_capacity;
    Fdtd1DNonlinearMouthParameters nonlinear_mouth;
    Fdtd1DJetLabiumParameters jet_labium;

    Fdtd1DBoundaryType left_boundary_type;
    Fdtd1DBoundaryType right_boundary_type;
    f32 left_reflection_coefficient;
    f32 right_reflection_coefficient;
    f32 left_open_reflection_filter_a1;
    f32 left_open_reflection_filter_b0;
    f32 left_open_reflection_filter_b1;
    f32 right_open_reflection_filter_a1;
    f32 right_open_reflection_filter_b0;
    f32 right_open_reflection_filter_b1;
    f32 left_previous_outgoing_pressure;
    f32 left_previous_incoming_pressure;
    f32 right_previous_outgoing_pressure;
    f32 right_previous_incoming_pressure;
    f32 left_boundary_outgoing_wave;
    f32 left_boundary_incoming_wave;
    f32 right_boundary_outgoing_wave;
    f32 right_boundary_incoming_wave;
    f32 left_boundary_emitted_sample;
    f32 right_boundary_emitted_sample;
    f64 left_boundary_emission_sum_squares;
    f64 right_boundary_emission_sum_squares;
    f32 left_boundary_emission_peak_abs;
    f32 right_boundary_emission_peak_abs;
    u64 boundary_emission_sample_count;
    u32 noise_state;
} Fdtd1DState;

typedef struct Fdtd1D
{
    Simulation simulation;
    Fdtd1DDesc desc;
    Fdtd1DState *state;
} Fdtd1D;

bool Fdtd1D_ValidateDesc (const Fdtd1DDesc *desc);
bool Fdtd1D_Initialize (Fdtd1D *solver, MemoryArena *arena, const Fdtd1DDesc *desc);
void Fdtd1D_Shutdown (Fdtd1D *solver);
void Fdtd1D_Reset (Fdtd1D *solver);
bool Fdtd1D_SetNonlinearMouthParameters (
    Fdtd1D *solver,
    const Fdtd1DNonlinearMouthParameters *parameters
);
bool Fdtd1D_SetJetLabiumParameters (
    Fdtd1D *solver,
    const Fdtd1DJetLabiumParameters *parameters
);
Simulation *Fdtd1D_GetSimulation (Fdtd1D *solver);
const Fdtd1DState *Fdtd1D_GetState (const Fdtd1D *solver);

#endif // PM_ORGAN_SIM_FDTD_1D_H
