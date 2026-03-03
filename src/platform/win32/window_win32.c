#include <windows.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/platform/window.h"

static const char WINDOW_CLASS_NAME[] = "PMOrganWindowClass";

static LRESULT CALLBACK WindowProc (HWND hwnd, UINT message, WPARAM w_param, LPARAM l_param)
{
    Window *window;

    window = (Window *) GetWindowLongPtrA(hwnd, GWLP_USERDATA);

    switch (message)
    {
        case WM_NCCREATE:
        {
            CREATESTRUCTA *create_struct;

            create_struct = (CREATESTRUCTA *) l_param;
            SetWindowLongPtrA(hwnd, GWLP_USERDATA, (LONG_PTR) create_struct->lpCreateParams);
            return DefWindowProcA(hwnd, message, w_param, l_param);
        }

        case WM_SIZE:
        {
            if (window != NULL)
            {
                window->client_width = LOWORD(l_param);
                window->client_height = HIWORD(l_param);
            }
        } break;

        case WM_CLOSE:
        {
            if (window != NULL)
            {
                window->is_running = false;
            }

            DestroyWindow(hwnd);
            return 0;
        }

        case WM_DESTROY:
        {
            if (window != NULL)
            {
                window->is_running = false;
                window->native_handle = NULL;
            }

            PostQuitMessage(0);
            return 0;
        }
    }

    return DefWindowProcA(hwnd, message, w_param, l_param);
}

static bool RegisterWindowClass (HINSTANCE instance)
{
    WNDCLASSEXA window_class;
    ATOM atom;

    window_class.cbSize = sizeof(window_class);
    window_class.style = CS_HREDRAW | CS_VREDRAW;
    window_class.lpfnWndProc = WindowProc;
    window_class.cbClsExtra = 0;
    window_class.cbWndExtra = 0;
    window_class.hInstance = instance;
    window_class.hIcon = LoadIconA(NULL, IDI_APPLICATION);
    window_class.hCursor = LoadCursorA(NULL, IDC_ARROW);
    window_class.hbrBackground = (HBRUSH) (COLOR_WINDOW + 1);
    window_class.lpszMenuName = NULL;
    window_class.lpszClassName = WINDOW_CLASS_NAME;
    window_class.hIconSm = window_class.hIcon;

    atom = RegisterClassExA(&window_class);
    if (atom == 0)
    {
        if (GetLastError() == ERROR_CLASS_ALREADY_EXISTS)
        {
            return true;
        }

        return false;
    }

    return true;
}

bool PlatformWindow_Create (Window *window, const WindowDesc *desc)
{
    HINSTANCE instance;
    RECT client_rect;
    DWORD style;
    HWND hwnd;

    ASSERT(window != NULL);
    ASSERT(desc != NULL);
    ASSERT(desc->title != NULL);
    ASSERT(desc->client_width > 0);
    ASSERT(desc->client_height > 0);

    instance = GetModuleHandleA(NULL);
    if (RegisterWindowClass(instance) == false)
    {
        return false;
    }

    client_rect.left = 0;
    client_rect.top = 0;
    client_rect.right = desc->client_width;
    client_rect.bottom = desc->client_height;

    style = WS_OVERLAPPEDWINDOW;
    AdjustWindowRect(&client_rect, style, FALSE);

    window->client_width = desc->client_width;
    window->client_height = desc->client_height;
    window->is_running = true;
    window->native_handle = NULL;

    hwnd = CreateWindowExA(
        0,
        WINDOW_CLASS_NAME,
        desc->title,
        style,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        client_rect.right - client_rect.left,
        client_rect.bottom - client_rect.top,
        NULL,
        NULL,
        instance,
        window
    );

    if (hwnd == NULL)
    {
        window->is_running = false;
        return false;
    }

    window->native_handle = hwnd;

    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);

    return true;
}

void PlatformWindow_Destroy (Window *window)
{
    HWND hwnd;

    ASSERT(window != NULL);

    hwnd = (HWND) window->native_handle;
    if (hwnd != NULL)
    {
        DestroyWindow(hwnd);
    }

    window->native_handle = NULL;
    window->is_running = false;
}

void PlatformWindow_PumpMessages (Window *window)
{
    MSG message;

    ASSERT(window != NULL);

    while (PeekMessageA(&message, NULL, 0, 0, PM_REMOVE))
    {
        if (message.message == WM_QUIT)
        {
            window->is_running = false;
        }

        TranslateMessage(&message);
        DispatchMessageA(&message);
    }
}
