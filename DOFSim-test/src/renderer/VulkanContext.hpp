#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef VK_USE_PLATFORM_WIN32_KHR
#define VK_USE_PLATFORM_WIN32_KHR
#endif
#include <windows.h>
#include <vulkan/vulkan.h>

#include "imgui_impl_vulkan.h"

#include <cstdint>
#include <optional>

struct ImDrawData;

// Owns the complete Vulkan presentation path used by Dear ImGui. The arm and
// all UI shapes are ImGui draw commands, rasterized by its Vulkan backend.
class VulkanContext
{
public:
    VulkanContext(HWND windowHandle, HINSTANCE instanceHandle);
    ~VulkanContext();

    VulkanContext(const VulkanContext&) = delete;
    VulkanContext& operator=(const VulkanContext&) = delete;

    void initializeImGui(int width, int height);
    void shutdownImGui();
    void render(ImDrawData* drawData, int width, int height);
    void waitIdle() const;

private:
    struct QueueFamilyIndices
    {
        std::optional<std::uint32_t> graphicsFamily;
        std::optional<std::uint32_t> presentFamily;

        bool complete() const
        {
            return graphicsFamily.has_value() && presentFamily.has_value();
        }
    };

    void createInstance();
    void createSurface(HWND windowHandle, HINSTANCE instanceHandle);
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createDescriptorPool();
    void createOrResizeSwapchain(int width, int height);
    void uploadFonts();
    void cleanup();

    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device) const;
    bool isDeviceSuitable(VkPhysicalDevice device) const;
    bool supportsSwapchainExtension(VkPhysicalDevice device) const;

    VkInstance m_instance = VK_NULL_HANDLE;
    VkSurfaceKHR m_surface = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;
    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;

    VkQueue m_graphicsQueue = VK_NULL_HANDLE;
    VkQueue m_presentQueue = VK_NULL_HANDLE;
    std::uint32_t m_graphicsQueueFamily = 0;

    ImGui_ImplVulkanH_Window m_mainWindow{};
    bool m_imguiInitialized = false;
    int m_swapchainWidth = 0;
    int m_swapchainHeight = 0;
    std::uint32_t m_minImageCount = 2;
};
