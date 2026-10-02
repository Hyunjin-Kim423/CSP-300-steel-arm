#pragma once

#include "core/Window.hpp"
#include "renderer/VulkanContext.hpp"
#include "robot/Kinematics.hpp"

#include <string>

class Application
{
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void run();

private:
    void buildInterface();
    void drawRobot();
    void solveTarget();

    Window m_window;
    VulkanContext m_vulkan;
    PlanarArm m_arm;
    Vec2 m_target{};
    bool m_hasTarget = false;
    bool m_targetReachable = true;
    std::string m_status = "Click the open canvas to choose a target.";
};
