cmake_minimum_required(VERSION 3.24)

project(PM_Organ C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

add_library(imgui STATIC
    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_dx11.cpp
    external/imgui/backends/imgui_impl_win32.cpp
)

target_include_directories(imgui PUBLIC
    external/imgui
    external/imgui/backends
)

target_compile_definitions(imgui PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

if (MSVC)
    target_compile_options(imgui PRIVATE
        /W3
        /EHsc
    )
else ()
    target_compile_options(imgui PRIVATE
        -w
    )
endif ()

add_executable(pm_organ WIN32
    src/app/app.c
    src/audio/audio_device.c
    src/audio/audio_engine.c
    src/audio/test_tone_source.c
    src/debug/debug_gui_imgui.cpp
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
    d3d11
    dxgi
    imgui
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
