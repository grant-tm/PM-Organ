#ifndef PM_ORGAN_PLATFORM_WINDOW_H
#define PM_ORGAN_PLATFORM_WINDOW_H

#include "pm_organ/core/types.h"

typedef struct WindowDesc
{
    const char *title;
    i32 client_width;
    i32 client_height;
} WindowDesc;

typedef struct Window
{
    void *native_handle;
    i32 client_width;
    i32 client_height;
    bool is_running;
} Window;

bool PlatformWindow_Create (Window *window, const WindowDesc *desc);
void PlatformWindow_Destroy (Window *window);
void PlatformWindow_PumpMessages (Window *window);

#endif // PM_ORGAN_PLATFORM_WINDOW_H
