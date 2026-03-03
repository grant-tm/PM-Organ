#include <windows.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/platform/time.h"

static LARGE_INTEGER g_qpc_frequency;
static bool g_time_is_initialized = false;

bool PlatformTime_Initialize (void)
{
    if (g_time_is_initialized)
    {
        return true;
    }

    if (QueryPerformanceFrequency(&g_qpc_frequency) == 0)
    {
        g_qpc_frequency.QuadPart = 0;
        return false;
    }

    ASSERT(g_qpc_frequency.QuadPart > 0);

    g_time_is_initialized = true;
    return true;
}

i64 PlatformTime_GetTicks (void)
{
    LARGE_INTEGER counter;

    ASSERT(g_time_is_initialized == true);

    QueryPerformanceCounter(&counter);
    return counter.QuadPart;
}

f64 PlatformTime_GetSeconds (void)
{
    return PlatformTime_GetSecondsElapsed(0, PlatformTime_GetTicks());
}

f64 PlatformTime_GetSecondsElapsed (i64 start_ticks, i64 end_ticks)
{
    i64 delta_ticks;

    ASSERT(g_time_is_initialized == true);

    delta_ticks = end_ticks - start_ticks;
    return (f64) delta_ticks / (f64) g_qpc_frequency.QuadPart;
}

f64 PlatformTime_GetSecondsSince (i64 start_ticks)
{
    return PlatformTime_GetSecondsElapsed(start_ticks, PlatformTime_GetTicks());
}

void PlatformTime_SleepMilliseconds (u32 milliseconds)
{
    Sleep(milliseconds);
}
