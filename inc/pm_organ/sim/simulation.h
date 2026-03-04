#ifndef PM_ORGAN_SIM_SIMULATION_H
#define PM_ORGAN_SIM_SIMULATION_H

#include "pm_organ/core/memory_arena.h"
#include "pm_organ/core/types.h"

typedef struct Simulation Simulation;
typedef struct SimulationProcessContext SimulationProcessContext;

typedef enum SimulationExcitationType
{
    SIMULATION_EXCITATION_TYPE_NONE = 0,
    SIMULATION_EXCITATION_TYPE_IMPULSE,
    SIMULATION_EXCITATION_TYPE_CONSTANT,
    SIMULATION_EXCITATION_TYPE_NOISE,
    SIMULATION_EXCITATION_TYPE_VELOCITY_IMPULSE,
    SIMULATION_EXCITATION_TYPE_VELOCITY_CONSTANT,
    SIMULATION_EXCITATION_TYPE_VELOCITY_NOISE,
    SIMULATION_EXCITATION_TYPE_NONLINEAR_MOUTH,
    SIMULATION_EXCITATION_TYPE_CUSTOM,
} SimulationExcitationType;

typedef struct SimulationConfig
{
    u32 sample_rate;
    u32 block_frame_count;
    u32 output_channel_count;
    u32 probe_count;
    u32 parameter_count;
    u32 excitation_capacity;
    u32 scratch_sample_count;
    usize model_state_size;
} SimulationConfig;

typedef struct SimulationProbe
{
    u32 read_index;
    u32 output_channel_index;
    bool is_enabled;
} SimulationProbe;

typedef struct SimulationParameter
{
    f64 current_value;
    f64 default_value;
    f64 minimum_value;
    f64 maximum_value;
    bool is_dirty;
} SimulationParameter;

typedef struct SimulationExcitation
{
    SimulationExcitationType type;
    u32 target_index;
    u32 remaining_frame_count;
    f64 value;
    bool is_active;
} SimulationExcitation;

typedef struct SimulationStats
{
    u64 processed_block_count;
    u64 processed_frame_count;
    f64 processed_seconds;
    f32 max_abs_output;
    f32 max_abs_probe;
    bool saw_nan;
    bool saw_inf;
} SimulationStats;

typedef struct SimulationOfflineCapture
{
    f32 *samples;
    u32 channel_count;
    u32 frame_capacity;
    u32 frame_count;
} SimulationOfflineCapture;

struct SimulationProcessContext
{
    f32 *output_buffer;
    f32 *scratch_buffer;
    f32 *probe_buffer;
    SimulationProbe *probes;
    SimulationParameter *parameters;
    SimulationExcitation *excitations;
    u32 block_frame_count;
    u32 output_channel_count;
    u32 probe_count;
    u32 parameter_count;
    u32 excitation_count;
    u32 sample_rate;
};

typedef bool SimulationValidateConfigCallback (const SimulationConfig *config);
typedef bool SimulationInitializeStateCallback (Simulation *simulation);
typedef void SimulationResetStateCallback (Simulation *simulation);
typedef void SimulationProcessBlockCallback (Simulation *simulation, SimulationProcessContext *process_context);

typedef struct SimulationInterface
{
    SimulationValidateConfigCallback *validate_config;
    SimulationInitializeStateCallback *initialize_state;
    SimulationResetStateCallback *reset_state;
    SimulationProcessBlockCallback *process_block;
} SimulationInterface;

struct Simulation
{
    SimulationConfig config;
    SimulationInterface interface;
    void *model_state;
    f32 *output_buffer;
    f32 *scratch_buffer;
    f32 *probe_buffer;
    SimulationProbe *probes;
    SimulationParameter *parameters;
    SimulationExcitation *excitations;
    u32 excitation_count;
    SimulationStats stats;
    bool is_initialized;
};

bool Simulation_ValidateConfig (const SimulationConfig *config);
bool Simulation_Initialize (
    Simulation *simulation,
    MemoryArena *arena,
    const SimulationConfig *config,
    const SimulationInterface *simulation_interface
);
void Simulation_Shutdown (Simulation *simulation);
void Simulation_Reset (Simulation *simulation);
bool Simulation_SetProbe (Simulation *simulation, u32 probe_index, const SimulationProbe *probe);
bool Simulation_SetParameter (Simulation *simulation, u32 parameter_index, const SimulationParameter *parameter);
bool Simulation_QueueExcitation (Simulation *simulation, const SimulationExcitation *excitation);
void Simulation_ClearExcitations (Simulation *simulation);
bool Simulation_ProcessBlock (Simulation *simulation);
bool Simulation_CaptureOutput (Simulation *simulation, SimulationOfflineCapture *capture);
bool Simulation_RunOfflineBlocks (Simulation *simulation, u32 block_count, SimulationOfflineCapture *capture);
void Simulation_ResetStats (Simulation *simulation);
const SimulationStats *Simulation_GetStats (const Simulation *simulation);

#endif // PM_ORGAN_SIM_SIMULATION_H
