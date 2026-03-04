cmake_minimum_required(VERSION 3.24)

project(PM_Organ C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(pm_organ WIN32
    src/app/app.c
    src/audio/audio_device.c
    src/audio/audio_engine.c
    src/audio/test_tone_source.c
    src/app/main.c
    src/core/frame_timer.c
    src/core/memory_arena.c
    src/sim/fdtd_1d.c
    src/sim/simulation.c
    src/platform/win32/audio_device_wasapi.c
    src/platform/win32/time_win32.c
    src/platform/win32/window_win32.c
)

target_include_directories(pm_organ PRIVATE
    inc
)

target_link_libraries(pm_organ PRIVATE
    avrt
    ole32
    uuid
)

target_compile_definitions(pm_organ PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

if (MSVC)
    target_compile_options(pm_organ PRIVATE
        /W4
        /WX
        /wd4201
    )
else ()
    target_compile_options(pm_organ PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
    )
endif ()
