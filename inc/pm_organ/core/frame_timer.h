#ifndef PM_ORGAN_CORE_FRAME_TIMER_H
#define PM_ORGAN_CORE_FRAME_TIMER_H

#include "pm_organ/core/types.h"

typedef struct FrameTimer
{
    i64 app_start_ticks;
    i64 frame_start_ticks;
    i64 previous_frame_ticks;
    i64 frame_index;
    f64 delta_seconds;
    f64 elapsed_seconds;
    f64 target_frame_seconds;
} FrameTimer;

void FrameTimer_Initialize (FrameTimer *timer);
void FrameTimer_SetTargetHz (FrameTimer *timer, f64 target_hz);
void FrameTimer_BeginFrame (FrameTimer *timer);
void FrameTimer_EndFrame (FrameTimer *timer);

#endif // PM_ORGAN_CORE_FRAME_TIMER_H
