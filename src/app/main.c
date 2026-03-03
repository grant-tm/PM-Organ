#include <windows.h>

#include "pm_organ/app/app.h"

int WINAPI WinMain (HINSTANCE instance, HINSTANCE previous_instance, LPSTR command_line, int show_code)
{
    (void) instance;
    (void) previous_instance;
    (void) command_line;
    (void) show_code;

    return App_Run();
}
