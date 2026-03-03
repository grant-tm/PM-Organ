#include "pm_organ/core/assert.h"
#include "pm_organ/core/frame_timer.h"
#include "pm_organ/platform/time.h"

void FrameTimer_Initialize (FrameTimer *timer)
{
    i64 now_ticks;

    ASSERT(timer != NULL);

    now_ticks = PlatformTime_GetTicks();

    timer->app_start_ticks = now_ticks;
    timer->frame_start_ticks = now_ticks;
    timer->previous_frame_ticks = now_ticks;
    timer->frame_index = 0;
    timer->delta_seconds = 0.0;
    timer->elapsed_seconds = 0.0;
    timer->target_frame_seconds = 0.0;
}

void FrameTimer_SetTargetHz (FrameTimer *timer, f64 target_hz)
{
    ASSERT(timer != NULL);
    ASSERT(target_hz >= 0.0);

    if (target_hz == 0.0)
    {
        timer->target_frame_seconds = 0.0;
        return;
    }

    timer->target_frame_seconds = 1.0 / target_hz;
}

void FrameTimer_BeginFrame (FrameTimer *timer)
{
    i64 now_ticks;

    ASSERT(timer != NULL);

    now_ticks = PlatformTime_GetTicks();

    timer->frame_start_ticks = now_ticks;
    timer->delta_seconds = PlatformTime_GetSecondsElapsed(timer->previous_frame_ticks, now_ticks);
    timer->elapsed_seconds = PlatformTime_GetSecondsElapsed(timer->app_start_ticks, now_ticks);
    timer->previous_frame_ticks = now_ticks;
}

void FrameTimer_EndFrame (FrameTimer *timer)
{
    f64 remaining_seconds;
    u32 sleep_milliseconds;

    ASSERT(timer != NULL);

    if (timer->target_frame_seconds <= 0.0)
    {
        timer->frame_index += 1;
        return;
    }

    remaining_seconds = timer->target_frame_seconds - PlatformTime_GetSecondsSince(timer->frame_start_ticks);
    while (remaining_seconds > 0.0015)
    {
        sleep_milliseconds = (u32) (remaining_seconds * 1000.0) - 1;
        sleep_milliseconds = (sleep_milliseconds > 0) ? sleep_milliseconds : 1;

        PlatformTime_SleepMilliseconds(sleep_milliseconds);
        remaining_seconds = timer->target_frame_seconds - PlatformTime_GetSecondsSince(timer->frame_start_ticks);
    }

    while (PlatformTime_GetSecondsSince(timer->frame_start_ticks) < timer->target_frame_seconds)
    {
    }

    timer->elapsed_seconds = PlatformTime_GetSecondsElapsed(timer->app_start_ticks, PlatformTime_GetTicks());
    timer->frame_index += 1;
}
