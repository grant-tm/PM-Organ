#ifndef PM_ORGAN_SIM_FDTD_1D_RENDER_SOURCE_H
#define PM_ORGAN_SIM_FDTD_1D_RENDER_SOURCE_H

#include "pm_organ/core/memory_arena.h"
#include "pm_organ/core/types.h"
#include "pm_organ/sim/fdtd_1d.h"

typedef enum Fdtd1DOutputExtractionMode
{
    FDTD_1D_OUTPUT_EXTRACTION_MODE_RAW_PROBES = 0,
    FDTD_1D_OUTPUT_EXTRACTION_MODE_MOUTH_RADIATION,
} Fdtd1DOutputExtractionMode;

typedef enum Fdtd1DExcitationMode
{
    FDTD_1D_EXCITATION_MODE_IMPULSE = 0,
    FDTD_1D_EXCITATION_MODE_CONSTANT,
    FDTD_1D_EXCITATION_MODE_NOISE,
} Fdtd1DExcitationMode;

typedef struct Fdtd1DRenderSourceDesc
{
    Fdtd1DDesc solver_desc;
    Fdtd1DExcitationMode excitation_mode;
    f64 drive_amplitude;
    Fdtd1DOutputExtractionMode output_extraction_mode;
    bool startup_impulse_is_enabled;
    u32 startup_impulse_target_index;
    f64 startup_impulse_amplitude;
} Fdtd1DRenderSourceDesc;

typedef struct Fdtd1DRenderSource
{
    Fdtd1D solver;
    Fdtd1DExcitationMode excitation_mode;
    f64 drive_amplitude;
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
void Fdtd1DRenderSource_SetDriveAmplitude (Fdtd1DRenderSource *source, f64 drive_amplitude);
void Fdtd1DRenderSource_Render (
    void *user_data,
    f32 *output,
    f32 *scratch_buffer,
    u32 block_frame_count,
    u32 channel_count,
    u32 sample_rate
);

#endif // PM_ORGAN_SIM_FDTD_1D_RENDER_SOURCE_H
