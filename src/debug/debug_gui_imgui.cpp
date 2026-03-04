#include <d3d11.h>
#include <dxgi.h>
#include <string.h>

#include "imgui.h"
#include "backends/imgui_impl_dx11.h"
#include "backends/imgui_impl_win32.h"

#include "pm_organ/core/assert.h"
#include "pm_organ/debug/debug_gui.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param);

struct DebugGuiInternal
{
    HWND hwnd;
    ID3D11Device *device;
    ID3D11DeviceContext *device_context;
    IDXGISwapChain *swap_chain;
    ID3D11RenderTargetView *render_target_view;
    i32 cached_width;
    i32 cached_height;
};

static void CleanupRenderTarget (DebugGuiInternal *internal)
{
    ASSERT(internal != nullptr);

    if (internal->render_target_view != nullptr)
    {
        internal->render_target_view->Release();
        internal->render_target_view = nullptr;
    }
}

static bool CreateRenderTarget (DebugGuiInternal *internal)
{
    HRESULT result;
    ID3D11Texture2D *back_buffer;

    ASSERT(internal != nullptr);
    ASSERT(internal->swap_chain != nullptr);
    ASSERT(internal->device != nullptr);

    back_buffer = nullptr;
    result = internal->swap_chain->GetBuffer(0, IID_PPV_ARGS(&back_buffer));
    if (FAILED(result))
    {
        return false;
    }

    result = internal->device->CreateRenderTargetView(back_buffer, nullptr, &internal->render_target_view);
    back_buffer->Release();
    return SUCCEEDED(result);
}

static bool ResizeSwapChain (DebugGuiInternal *internal, i32 width, i32 height)
{
    HRESULT result;

    ASSERT(internal != nullptr);

    if ((width <= 0) || (height <= 0))
    {
        return true;
    }

    if ((width == internal->cached_width) &&
        (height == internal->cached_height))
    {
        return true;
    }

    CleanupRenderTarget(internal);
    result = internal->swap_chain->ResizeBuffers(
        0,
        (UINT) width,
        (UINT) height,
        DXGI_FORMAT_UNKNOWN,
        0
    );
    if (FAILED(result))
    {
        return false;
    }

    internal->cached_width = width;
    internal->cached_height = height;

    return CreateRenderTarget(internal);
}

static bool CreateDeviceD3D (DebugGuiInternal *internal, HWND hwnd)
{
    static const D3D_FEATURE_LEVEL FEATURE_LEVELS[2] =
    {
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_0,
    };

    D3D_FEATURE_LEVEL feature_level;
    DXGI_SWAP_CHAIN_DESC swap_chain_desc;
    HRESULT result;
    UINT create_device_flags;
    IDXGIFactory *factory;

    ASSERT(internal != nullptr);

    memset(&swap_chain_desc, 0, sizeof(swap_chain_desc));
    swap_chain_desc.BufferCount = 2;
    swap_chain_desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    swap_chain_desc.BufferDesc.RefreshRate.Numerator = 60;
    swap_chain_desc.BufferDesc.RefreshRate.Denominator = 1;
    swap_chain_desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swap_chain_desc.OutputWindow = hwnd;
    swap_chain_desc.SampleDesc.Count = 1;
    swap_chain_desc.Windowed = TRUE;
    swap_chain_desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    swap_chain_desc.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

    create_device_flags = 0;
    result = D3D11CreateDeviceAndSwapChain(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        create_device_flags,
        FEATURE_LEVELS,
        2,
        D3D11_SDK_VERSION,
        &swap_chain_desc,
        &internal->swap_chain,
        &internal->device,
        &feature_level,
        &internal->device_context
    );
    if (result == DXGI_ERROR_UNSUPPORTED)
    {
        result = D3D11CreateDeviceAndSwapChain(
            nullptr,
            D3D_DRIVER_TYPE_WARP,
            nullptr,
            create_device_flags,
            FEATURE_LEVELS,
            2,
            D3D11_SDK_VERSION,
            &swap_chain_desc,
            &internal->swap_chain,
            &internal->device,
            &feature_level,
            &internal->device_context
        );
    }

    if (FAILED(result))
    {
        return false;
    }

    factory = nullptr;
    if (SUCCEEDED(internal->swap_chain->GetParent(IID_PPV_ARGS(&factory))))
    {
        factory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);
        factory->Release();
    }

    return CreateRenderTarget(internal);
}

static void CleanupDeviceD3D (DebugGuiInternal *internal)
{
    ASSERT(internal != nullptr);

    CleanupRenderTarget(internal);

    if (internal->swap_chain != nullptr)
    {
        internal->swap_chain->Release();
        internal->swap_chain = nullptr;
    }

    if (internal->device_context != nullptr)
    {
        internal->device_context->Release();
        internal->device_context = nullptr;
    }

    if (internal->device != nullptr)
    {
        internal->device->Release();
        internal->device = nullptr;
    }
}

extern "C" bool DebugGui_Initialize (DebugGui *gui, MemoryArena *arena, Window *window)
{
    DebugGuiInternal *internal;
    ImGuiIO *io;

    ASSERT(gui != nullptr);
    ASSERT(arena != nullptr);
    ASSERT(window != nullptr);
    ASSERT(window->native_handle != nullptr);

    memset(gui, 0, sizeof(*gui));

    internal = (DebugGuiInternal *) MemoryArena_PushZeroSize(arena, sizeof(DebugGuiInternal), alignof(DebugGuiInternal));
    if (internal == nullptr)
    {
        return false;
    }

    memset(internal, 0, sizeof(*internal));
    internal->hwnd = (HWND) window->native_handle;
    internal->cached_width = window->client_width;
    internal->cached_height = window->client_height;
    gui->internal_state = internal;

    if (CreateDeviceD3D(internal, internal->hwnd) == false)
    {
        CleanupDeviceD3D(internal);
        gui->internal_state = nullptr;
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = &ImGui::GetIO();
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    if (ImGui_ImplWin32_Init((void *) internal->hwnd) == false)
    {
        ImGui::DestroyContext();
        CleanupDeviceD3D(internal);
        gui->internal_state = nullptr;
        return false;
    }

    if (ImGui_ImplDX11_Init(internal->device, internal->device_context) == false)
    {
        ImGui_ImplWin32_Shutdown();
        ImGui::DestroyContext();
        CleanupDeviceD3D(internal);
        gui->internal_state = nullptr;
        return false;
    }

    return true;
}

extern "C" void DebugGui_Shutdown (DebugGui *gui)
{
    DebugGuiInternal *internal;

    ASSERT(gui != nullptr);

    internal = (DebugGuiInternal *) gui->internal_state;
    if (internal == nullptr)
    {
        return;
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    CleanupDeviceD3D(internal);
    gui->internal_state = nullptr;
}

extern "C" void DebugGui_BeginFrame (DebugGui *gui)
{
    DebugGuiInternal *internal;

    ASSERT(gui != nullptr);

    internal = (DebugGuiInternal *) gui->internal_state;
    ASSERT(internal != nullptr);

    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

extern "C" void DebugGui_Draw (DebugGui *gui, const DebugGuiFrameDesc *frame_desc, DebugGuiFrameActions *frame_actions)
{
    DebugGuiInternal *internal;
    usize preset_index;

    ASSERT(gui != nullptr);
    ASSERT(frame_desc != nullptr);
    ASSERT(frame_actions != nullptr);
    ASSERT(frame_desc->preset_names != nullptr);
    ASSERT(frame_desc->excitation_mode_names != nullptr);
    ASSERT(frame_desc->output_extraction_mode_names != nullptr);

    internal = (DebugGuiInternal *) gui->internal_state;
    ASSERT(internal != nullptr);

    memset(frame_actions, 0, sizeof(*frame_actions));

    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360.0f, 0.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("PM-Organ Controls"))
    {
        ImGui::Text("Source");
        if (ImGui::RadioButton("FDTD", frame_desc->fdtd_source_is_active))
        {
            frame_actions->request_use_fdtd_source = true;
        }

        ImGui::SameLine();
        if (ImGui::RadioButton("Test Tone", frame_desc->fdtd_source_is_active == false))
        {
            frame_actions->request_use_test_tone = true;
        }

        if (ImGui::Button("Trigger Impulse"))
        {
            frame_actions->request_trigger_impulse = true;
        }

        ImGui::Separator();
        ImGui::Text("Excitation");
        for (preset_index = 0; preset_index < frame_desc->excitation_mode_count; preset_index += 1)
        {
            bool is_selected;

            is_selected = (preset_index == frame_desc->active_excitation_mode);
            if (ImGui::Selectable(frame_desc->excitation_mode_names[preset_index], is_selected))
            {
                frame_actions->request_select_excitation_mode = true;
                frame_actions->selected_excitation_mode = (u32) preset_index;
            }
        }

        frame_actions->drive_amplitude = frame_desc->drive_amplitude;
        if (ImGui::SliderFloat("Drive Amplitude", &frame_actions->drive_amplitude, 0.0f, 0.02f, "%.5f"))
        {
            frame_actions->request_set_drive_amplitude = true;
        }

        ImGui::Separator();
        ImGui::Text("Source Coupling");
        for (preset_index = 0; preset_index < frame_desc->source_coupling_mode_count; preset_index += 1)
        {
            bool is_selected;

            is_selected = (preset_index == frame_desc->active_source_coupling_mode);
            if (ImGui::Selectable(frame_desc->source_coupling_mode_names[preset_index], is_selected))
            {
                frame_actions->request_select_source_coupling_mode = true;
                frame_actions->selected_source_coupling_mode = (u32) preset_index;
            }
        }

        ImGui::Separator();
        ImGui::Text("Output Extraction");
        for (preset_index = 0; preset_index < frame_desc->output_extraction_mode_count; preset_index += 1)
        {
            bool is_selected;

            is_selected = (preset_index == frame_desc->active_output_extraction_mode);
            if (ImGui::Selectable(frame_desc->output_extraction_mode_names[preset_index], is_selected))
            {
                frame_actions->request_select_output_extraction_mode = true;
                frame_actions->selected_output_extraction_mode = (u32) preset_index;
            }
        }

        ImGui::Separator();
        ImGui::Text("Geometry Presets");
        for (preset_index = 0; preset_index < frame_desc->preset_count; preset_index += 1)
        {
            bool is_selected;

            is_selected = (preset_index == frame_desc->active_preset_index);
            if (ImGui::Selectable(frame_desc->preset_names[preset_index], is_selected))
            {
                frame_actions->request_select_preset = true;
                frame_actions->selected_preset_index = (u32) preset_index;
            }
        }

        ImGui::Separator();
        ImGui::Text("Frame: %.3f ms (%.1f FPS)",
            frame_desc->delta_seconds * 1000.0,
            (frame_desc->delta_seconds > 0.0) ? (1.0 / frame_desc->delta_seconds) : 0.0
        );
        ImGui::Text("Hotkeys: Space retrigger, T source, G preset");

    }
    ImGui::End();
}

extern "C" void DebugGui_Render (DebugGui *gui)
{
    static const FLOAT CLEAR_COLOR[4] = { 0.07f, 0.08f, 0.09f, 1.0f };

    DebugGuiInternal *internal;

    ASSERT(gui != nullptr);

    internal = (DebugGuiInternal *) gui->internal_state;
    ASSERT(internal != nullptr);

    ImGui::Render();

    internal->device_context->OMSetRenderTargets(1, &internal->render_target_view, nullptr);
    internal->device_context->ClearRenderTargetView(internal->render_target_view, CLEAR_COLOR);
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
    internal->swap_chain->Present(1, 0);
}

extern "C" bool DebugGui_WindowMessageCallback (
    void *user_data,
    void *native_handle,
    u32 message,
    usize w_param,
    usize l_param,
    usize *result
)
{
    DebugGui *gui;
    DebugGuiInternal *internal;
    LRESULT callback_result;

    ASSERT(user_data != nullptr);
    ASSERT(result != nullptr);

    gui = (DebugGui *) user_data;
    internal = (DebugGuiInternal *) gui->internal_state;
    if (internal == nullptr)
    {
        return false;
    }

    callback_result = ImGui_ImplWin32_WndProcHandler(
        (HWND) native_handle,
        (UINT) message,
        (WPARAM) w_param,
        (LPARAM) l_param
    );
    if (callback_result != 0)
    {
        *result = (usize) callback_result;
        return true;
    }

    if (message == WM_SIZE)
    {
        if (w_param != SIZE_MINIMIZED)
        {
            ResizeSwapChain(internal, (i32) LOWORD(l_param), (i32) HIWORD(l_param));
        }
    }

    return false;
}
