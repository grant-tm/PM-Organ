#ifndef PM_ORGAN_PLATFORM_WINDOW_H
#define PM_ORGAN_PLATFORM_WINDOW_H

#include "pm_organ/core/types.h"

typedef bool WindowMessageCallback (
    void *user_data,
    void *native_handle,
    u32 message,
    usize w_param,
    usize l_param,
    usize *result
);

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
    WindowMessageCallback *message_callback;
    void *message_callback_user_data;
    bool is_running;
} Window;

bool PlatformWindow_Create (Window *window, const WindowDesc *desc);
void PlatformWindow_Destroy (Window *window);
void PlatformWindow_PumpMessages (Window *window);
void PlatformWindow_SetMessageCallback (
    Window *window,
    WindowMessageCallback *message_callback,
    void *user_data
);
void PlatformWindow_SetTitle (Window *window, const char *title);

#endif // PM_ORGAN_PLATFORM_WINDOW_H
