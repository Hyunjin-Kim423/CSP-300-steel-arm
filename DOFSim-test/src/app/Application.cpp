#include "app/Application.hpp"

#include "imgui.h"
#include "imgui_impl_win32.h"

#include <array>
#include <cmath>

// The backend intentionally keeps this declaration out of its header so that
// including the header never forces a dependency on windows.h.
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

Application::Application()
    : m_window(1280, 720, L"Vulkan 2DOF / 3DOF Inverse Kinematics")
    , m_vulkan(m_window.nativeHandle(), m_window.instanceHandle())
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsLight();
    ImGui_ImplWin32_Init(m_window.nativeHandle());
    const SIZE size = m_window.clientSize();
    m_vulkan.initializeImGui(size.cx, size.cy);
    m_window.setMessageHandler([](HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
    {
        return ImGui_ImplWin32_WndProcHandler(hwnd, message, wParam, lParam) != 0;
    });
}

Application::~Application()
{
    m_vulkan.shutdownImGui();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void Application::run()
{
    while (!m_window.shouldClose())
    {
        m_window.pollEvents();
        const SIZE size = m_window.clientSize();
        if (size.cx <= 0 || size.cy <= 0)
        {
            continue;
        }

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        buildInterface();
        drawRobot();
        ImGui::Render();
        m_vulkan.render(ImGui::GetDrawData(), size.cx, size.cy);
    }
}

void Application::buildInterface()
{
    ImGui::SetNextWindowPos({20.0f, 20.0f}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({360.0f, 0.0f}, ImGuiCond_Always);
    ImGui::Begin("Planar Robot Arm", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
    ImGui::Separator();
    ImGui::TextUnformatted("Degrees of freedom");
    int dof = m_arm.degreesOfFreedom();
    if (ImGui::RadioButton("2 DOF", dof == 2))
    {
        m_arm.setDegreesOfFreedom(2);
        solveTarget();
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("3 DOF", dof == 3))
    {
        m_arm.setDegreesOfFreedom(3);
        solveTarget();
    }

    std::array<double, 3> linkLengths = m_arm.linkLengths();
    bool lengthChanged = false;
    for (int index = 0; index < m_arm.degreesOfFreedom(); ++index)
    {
        float value = static_cast<float>(linkLengths[static_cast<std::size_t>(index)]);
        const std::string label = "Link " + std::to_string(index + 1) + " length";
        if (ImGui::SliderFloat(label.c_str(), &value, 60.0f, 260.0f, "%.0f px"))
        {
            linkLengths[static_cast<std::size_t>(index)] = value;
            lengthChanged = true;
        }
    }
    if (lengthChanged)
    {
        m_arm.setLinkLengths(linkLengths);
        solveTarget();
    }

    ImGui::Separator();
    ImGui::TextWrapped("%s", m_status.c_str());
    if (m_hasTarget)
    {
        const ImVec2 screenRoot{ImGui::GetIO().DisplaySize.x * 0.5f, ImGui::GetIO().DisplaySize.y * 0.5f};
        ImGui::Text("Target relative to root: (%.0f, %.0f) px", m_target.x - screenRoot.x, m_target.y - screenRoot.y);
    }
    ImGui::End();

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
    {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        m_target = {mouse.x, mouse.y};
        m_hasTarget = true;
        solveTarget();
    }
}

void Application::solveTarget()
{
    if (!m_hasTarget)
    {
        return;
    }
    const ImVec2 screenRoot{ImGui::GetIO().DisplaySize.x * 0.5f, ImGui::GetIO().DisplaySize.y * 0.5f};
    const Vec2 targetFromRoot{m_target.x - screenRoot.x, m_target.y - screenRoot.y};
    m_targetReachable = m_arm.solve(targetFromRoot);
    if (m_targetReachable)
    {
        m_status = "Target reached by inverse kinematics.";
        return;
    }

    double maximumReach = 0.0;
    const auto& lengths = m_arm.linkLengths();
    for (int index = 0; index < m_arm.degreesOfFreedom(); ++index)
        maximumReach += lengths[static_cast<std::size_t>(index)];
    m_status = "Unreachable: the target is farther than the arm's available length (or inside its minimum radius).";
}

void Application::drawRobot()
{
    const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    const Vec2 root{displaySize.x * 0.5, displaySize.y * 0.5};
    ImDrawList* drawList = ImGui::GetBackgroundDrawList();

    constexpr float gridStep = 50.0f;
    const ImU32 gridColor = IM_COL32(220, 224, 232, 255);
    for (float x = std::fmod(static_cast<float>(root.x), gridStep); x < displaySize.x; x += gridStep)
        drawList->AddLine({x, 0.0f}, {x, displaySize.y}, gridColor, 1.0f);
    for (float y = std::fmod(static_cast<float>(root.y), gridStep); y < displaySize.y; y += gridStep)
        drawList->AddLine({0.0f, y}, {displaySize.x, y}, gridColor, 1.0f);
    drawList->AddLine({0.0f, static_cast<float>(root.y)}, {displaySize.x, static_cast<float>(root.y)}, IM_COL32(180, 185, 198, 255), 1.5f);
    drawList->AddLine({static_cast<float>(root.x), 0.0f}, {static_cast<float>(root.x), displaySize.y}, IM_COL32(180, 185, 198, 255), 1.5f);

    if (m_hasTarget)
    {
        drawList->AddCircleFilled({static_cast<float>(m_target.x), static_cast<float>(m_target.y)}, 9.0f, IM_COL32(230, 44, 44, 255), 24);
        drawList->AddCircle({static_cast<float>(m_target.x), static_cast<float>(m_target.y)}, 13.0f, IM_COL32(190, 20, 20, 255), 24, 2.0f);
    }

    const auto joints = m_arm.jointPositions(root);
    for (int index = 0; index < m_arm.degreesOfFreedom(); ++index)
    {
        const Vec2& a = joints[static_cast<std::size_t>(index)];
        const Vec2& b = joints[static_cast<std::size_t>(index + 1)];
        drawList->AddLine({static_cast<float>(a.x), static_cast<float>(a.y)}, {static_cast<float>(b.x), static_cast<float>(b.y)}, IM_COL32(10, 10, 10, 255), 7.0f);
    }
    for (int index = 0; index <= m_arm.degreesOfFreedom(); ++index)
    {
        const Vec2& joint = joints[static_cast<std::size_t>(index)];
        drawList->AddCircleFilled({static_cast<float>(joint.x), static_cast<float>(joint.y)}, 11.0f, IM_COL32(0, 0, 0, 255), 24);
    }

    const Vec2& end = joints[static_cast<std::size_t>(m_arm.degreesOfFreedom())];
    drawList->AddCircle({static_cast<float>(end.x), static_cast<float>(end.y)}, 15.0f, m_targetReachable ? IM_COL32(50, 110, 255, 255) : IM_COL32(255, 150, 0, 255), 24, 2.0f);
}
