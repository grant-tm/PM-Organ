cmake_minimum_required(VERSION 3.24)

project(PM_Organ C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

function(pm_organ_configure_target target_name)
    target_include_directories(${target_name} PRIVATE
        inc
    )

    target_compile_definitions(${target_name} PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )

    if (MSVC)
        target_compile_options(${target_name} PRIVATE
            /W4
            /WX
            /wd4201
        )
    else ()
        target_compile_options(${target_name} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Werror
        )
    endif ()
endfunction ()

add_executable(pm_organ WIN32
    src/app/app.c
    src/audio/audio_device.c
    src/audio/audio_engine.c
    src/audio/test_tone_source.c
    src/app/main.c
    src/core/frame_timer.c
    src/core/memory_arena.c
    src/sim/fdtd_1d.c
    src/sim/fdtd_1d_render_source.c
    src/sim/simulation.c
    src/platform/win32/audio_device_wasapi.c
    src/platform/win32/time_win32.c
    src/platform/win32/window_win32.c
)

pm_organ_configure_target(pm_organ)

target_link_libraries(pm_organ PRIVATE
    avrt
    ole32
    uuid
)

add_executable(fdtd_1d_verify
    tools/fdtd_1d_verify.c
    src/core/memory_arena.c
    src/sim/fdtd_1d.c
    src/sim/simulation.c
)

pm_organ_configure_target(fdtd_1d_verify)
