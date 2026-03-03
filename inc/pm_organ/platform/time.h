#ifndef PM_ORGAN_PLATFORM_TIME_H
#define PM_ORGAN_PLATFORM_TIME_H

#include "pm_organ/core/types.h"

bool PlatformTime_Initialize (void);
i64 PlatformTime_GetTicks (void);
f64 PlatformTime_GetSeconds (void);
f64 PlatformTime_GetSecondsElapsed (i64 start_ticks, i64 end_ticks);
f64 PlatformTime_GetSecondsSince (i64 start_ticks);
void PlatformTime_SleepMilliseconds (u32 milliseconds);

#endif // PM_ORGAN_PLATFORM_TIME_H
