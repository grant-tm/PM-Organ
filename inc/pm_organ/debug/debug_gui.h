#ifndef PM_ORGAN_DEBUG_DEBUG_GUI_H
#define PM_ORGAN_DEBUG_DEBUG_GUI_H

#include "pm_organ/core/memory_arena.h"
#include "pm_organ/core/types.h"
#include "pm_organ/platform/window.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct DebugGui
{
    void *internal_state;
} DebugGui;

typedef struct DebugGuiFrameDesc
{
    bool fdtd_source_is_active;
    bool output_is_muted;
    f32 drive_amplitude;
    f32 windchest_pressure;
    f32 speech_attack_seconds;
    f32 speech_chiff_amount;
    f32 speech_chiff_decay_seconds;
    f32 effective_drive_requested;
    f32 effective_drive_applied;
    f32 effective_drive_saturation_ratio;
    f32 master_gain;
    u32 active_preset_index;
    u32 active_excitation_mode;
    u32 excitation_mode_count;
    u32 active_source_coupling_mode;
    u32 source_coupling_mode_count;
    u32 active_output_extraction_mode;
    u32 output_extraction_mode_count;
    u32 preset_count;
    const char *const *excitation_mode_names;
    const char *const *source_coupling_mode_names;
    const char *const *output_extraction_mode_names;
    const char *const *preset_names;
    f64 delta_seconds;
} DebugGuiFrameDesc;

typedef struct DebugGuiFrameActions
{
    bool request_trigger_impulse;
    bool request_kill_output;
    bool request_use_fdtd_source;
    bool request_use_test_tone;
    bool request_select_excitation_mode;
    bool request_select_source_coupling_mode;
    bool request_select_output_extraction_mode;
    bool request_select_preset;
    bool request_set_drive_amplitude;
    bool request_set_windchest_pressure;
    bool request_set_speech_attack_seconds;
    bool request_set_speech_chiff_amount;
    bool request_set_speech_chiff_decay_seconds;
    bool request_set_master_gain;
    bool request_set_output_muted;
    u32 selected_excitation_mode;
    u32 selected_source_coupling_mode;
    u32 selected_output_extraction_mode;
    u32 selected_preset_index;
    f32 drive_amplitude;
    f32 windchest_pressure;
    f32 speech_attack_seconds;
    f32 speech_chiff_amount;
    f32 speech_chiff_decay_seconds;
    f32 master_gain;
    bool output_is_muted;
} DebugGuiFrameActions;

bool DebugGui_Initialize (DebugGui *gui, MemoryArena *arena, Window *window);
void DebugGui_Shutdown (DebugGui *gui);
void DebugGui_BeginFrame (DebugGui *gui);
void DebugGui_Draw (DebugGui *gui, const DebugGuiFrameDesc *frame_desc, DebugGuiFrameActions *frame_actions);
void DebugGui_Render (DebugGui *gui);
bool DebugGui_WindowMessageCallback (
    void *user_data,
    void *native_handle,
    u32 message,
    usize w_param,
    usize l_param,
    usize *result
);

#if defined(__cplusplus)
}
#endif

#endif // PM_ORGAN_DEBUG_DEBUG_GUI_H
