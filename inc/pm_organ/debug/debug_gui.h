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
    u32 active_preset_index;
    u32 preset_count;
    const char *const *preset_names;
    f64 delta_seconds;
} DebugGuiFrameDesc;

typedef struct DebugGuiFrameActions
{
    bool request_trigger_impulse;
    bool request_use_fdtd_source;
    bool request_use_test_tone;
    bool request_select_preset;
    u32 selected_preset_index;
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
