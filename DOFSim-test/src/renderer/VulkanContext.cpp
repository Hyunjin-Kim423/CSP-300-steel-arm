#include "renderer/VulkanContext.hpp"

#include "imgui.h"

#include <array>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void check(VkResult result)
{
    if (result != VK_SUCCESS)
        throw std::runtime_error("Vulkan call failed (VkResult " + std::to_string(result) + ").");
}
}

VulkanContext::VulkanContext(HWND hwnd, HINSTANCE instance)
{
    try { createInstance(); createSurface(hwnd, instance); pickPhysicalDevice(); createLogicalDevice(); }
    catch (...) { cleanup(); throw; }
}

VulkanContext::~VulkanContext() { cleanup(); }

void VulkanContext::initializeImGui(int width, int height)
{
    createDescriptorPool();
    createOrResizeSwapchain(width, height);
    ImGui_ImplVulkan_InitInfo info{};
    info.Instance = m_instance; info.PhysicalDevice = m_physicalDevice; info.Device = m_device;
    info.QueueFamily = m_graphicsQueueFamily; info.Queue = m_graphicsQueue;
    info.DescriptorPool = m_descriptorPool; info.MinImageCount = m_minImageCount;
    info.ImageCount = m_mainWindow.ImageCount; info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    info.CheckVkResultFn = check;
    if (!ImGui_ImplVulkan_Init(&info, m_mainWindow.RenderPass))
        throw std::runtime_error("Dear ImGui Vulkan backend initialization failed.");
    m_imguiInitialized = true;
    uploadFonts();
}

void VulkanContext::shutdownImGui()
{
    if (m_imguiInitialized)
    {
        waitIdle();
        ImGui_ImplVulkan_Shutdown();
        m_imguiInitialized = false;
    }
}

void VulkanContext::render(ImDrawData* drawData, int width, int height)
{
    if (!m_imguiInitialized || width <= 0 || height <= 0) return;
    if (width != m_swapchainWidth || height != m_swapchainHeight)
    {
        waitIdle();
        createOrResizeSwapchain(width, height);
        ImGui_ImplVulkan_SetMinImageCount(m_minImageCount);
    }

    ImGui_ImplVulkanH_Window& window = m_mainWindow;
    ImGui_ImplVulkanH_FrameSemaphores& semaphores = window.FrameSemaphores[window.SemaphoreIndex];
    VkResult result = vkAcquireNextImageKHR(m_device, window.Swapchain, UINT64_MAX,
        semaphores.ImageAcquiredSemaphore, VK_NULL_HANDLE, &window.FrameIndex);
    if (result == VK_ERROR_OUT_OF_DATE_KHR) { createOrResizeSwapchain(width, height); return; }
    if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) check(result);

    ImGui_ImplVulkanH_Frame& frame = window.Frames[window.FrameIndex];
    check(vkWaitForFences(m_device, 1, &frame.Fence, VK_TRUE, UINT64_MAX));
    check(vkResetFences(m_device, 1, &frame.Fence));
    check(vkResetCommandPool(m_device, frame.CommandPool, 0));
    VkCommandBufferBeginInfo begin{}; begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    check(vkBeginCommandBuffer(frame.CommandBuffer, &begin));
    VkRenderPassBeginInfo pass{}; pass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    pass.renderPass = window.RenderPass; pass.framebuffer = frame.Framebuffer;
    pass.renderArea.extent = {static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height)};
    pass.clearValueCount = 1; pass.pClearValues = &window.ClearValue;
    vkCmdBeginRenderPass(frame.CommandBuffer, &pass, VK_SUBPASS_CONTENTS_INLINE);
    ImGui_ImplVulkan_RenderDrawData(drawData, frame.CommandBuffer);
    vkCmdEndRenderPass(frame.CommandBuffer);
    check(vkEndCommandBuffer(frame.CommandBuffer));

    const VkPipelineStageFlags stage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.waitSemaphoreCount = 1; submit.pWaitSemaphores = &semaphores.ImageAcquiredSemaphore;
    submit.pWaitDstStageMask = &stage; submit.commandBufferCount = 1; submit.pCommandBuffers = &frame.CommandBuffer;
    submit.signalSemaphoreCount = 1; submit.pSignalSemaphores = &semaphores.RenderCompleteSemaphore;
    check(vkQueueSubmit(m_graphicsQueue, 1, &submit, frame.Fence));
    VkPresentInfoKHR present{}; present.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present.waitSemaphoreCount = 1; present.pWaitSemaphores = &semaphores.RenderCompleteSemaphore;
    present.swapchainCount = 1; present.pSwapchains = &window.Swapchain; present.pImageIndices = &window.FrameIndex;
    result = vkQueuePresentKHR(m_presentQueue, &present);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) createOrResizeSwapchain(width, height);
    else if (result != VK_SUCCESS) check(result);
    window.SemaphoreIndex = (window.SemaphoreIndex + 1) % window.ImageCount;
}

void VulkanContext::waitIdle() const { if (m_device != VK_NULL_HANDLE) vkDeviceWaitIdle(m_device); }

void VulkanContext::cleanup()
{
    waitIdle();
    shutdownImGui();
    if (m_mainWindow.Swapchain != VK_NULL_HANDLE && m_device != VK_NULL_HANDLE)
        ImGui_ImplVulkanH_DestroyWindow(m_instance, m_device, &m_mainWindow, nullptr);
    if (m_descriptorPool != VK_NULL_HANDLE && m_device != VK_NULL_HANDLE)
        vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr);
    if (m_device != VK_NULL_HANDLE) vkDestroyDevice(m_device, nullptr);
    if (m_surface != VK_NULL_HANDLE && m_instance != VK_NULL_HANDLE) vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
    if (m_instance != VK_NULL_HANDLE) vkDestroyInstance(m_instance, nullptr);
    m_device = VK_NULL_HANDLE; m_surface = VK_NULL_HANDLE; m_instance = VK_NULL_HANDLE;
}

void VulkanContext::createInstance()
{
    VkApplicationInfo app{}; app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "Vulkan Planar Robot Arm"; app.apiVersion = VK_API_VERSION_1_0;
    constexpr std::array<const char*, 2> extensions{VK_KHR_SURFACE_EXTENSION_NAME, VK_KHR_WIN32_SURFACE_EXTENSION_NAME};
    VkInstanceCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    info.pApplicationInfo = &app; info.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    info.ppEnabledExtensionNames = extensions.data(); check(vkCreateInstance(&info, nullptr, &m_instance));
}

void VulkanContext::createSurface(HWND hwnd, HINSTANCE instance)
{
    VkWin32SurfaceCreateInfoKHR info{}; info.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
    info.hwnd = hwnd; info.hinstance = instance; check(vkCreateWin32SurfaceKHR(m_instance, &info, nullptr, &m_surface));
}

void VulkanContext::pickPhysicalDevice()
{
    std::uint32_t count = 0; check(vkEnumeratePhysicalDevices(m_instance, &count, nullptr));
    if (count == 0) throw std::runtime_error("No Vulkan-capable GPU was found.");
    std::vector<VkPhysicalDevice> devices(count); check(vkEnumeratePhysicalDevices(m_instance, &count, devices.data()));
    for (VkPhysicalDevice device : devices) if (isDeviceSuitable(device)) { m_physicalDevice = device; break; }
    if (m_physicalDevice == VK_NULL_HANDLE) throw std::runtime_error("No GPU supports the Vulkan swapchain extension.");
}

void VulkanContext::createLogicalDevice()
{
    const QueueFamilyIndices indices = findQueueFamilies(m_physicalDevice);
    const std::set<std::uint32_t> families{indices.graphicsFamily.value(), indices.presentFamily.value()};
    constexpr float priority = 1.0f; std::vector<VkDeviceQueueCreateInfo> queues;
    for (std::uint32_t family : families) { VkDeviceQueueCreateInfo q{}; q.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        q.queueFamilyIndex = family; q.queueCount = 1; q.pQueuePriorities = &priority; queues.push_back(q); }
    constexpr const char* extensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    VkDeviceCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    info.queueCreateInfoCount = static_cast<std::uint32_t>(queues.size()); info.pQueueCreateInfos = queues.data();
    info.enabledExtensionCount = 1; info.ppEnabledExtensionNames = extensions; check(vkCreateDevice(m_physicalDevice, &info, nullptr, &m_device));
    m_graphicsQueueFamily = indices.graphicsFamily.value();
    vkGetDeviceQueue(m_device, m_graphicsQueueFamily, 0, &m_graphicsQueue);
    vkGetDeviceQueue(m_device, indices.presentFamily.value(), 0, &m_presentQueue);
}

void VulkanContext::createDescriptorPool()
{
    constexpr std::array<VkDescriptorPoolSize, 11> sizes{{
        {VK_DESCRIPTOR_TYPE_SAMPLER, 1000}, {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
        {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
        {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}}};
    VkDescriptorPoolCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT; info.maxSets = 11000;
    info.poolSizeCount = static_cast<std::uint32_t>(sizes.size()); info.pPoolSizes = sizes.data();
    check(vkCreateDescriptorPool(m_device, &info, nullptr, &m_descriptorPool));
}

void VulkanContext::createOrResizeSwapchain(int width, int height)
{
    if (m_mainWindow.Swapchain != VK_NULL_HANDLE)
        ImGui_ImplVulkanH_DestroyWindow(m_instance, m_device, &m_mainWindow, nullptr);
    m_mainWindow.Surface = m_surface;
    constexpr VkFormat formats[] = {VK_FORMAT_B8G8R8A8_UNORM, VK_FORMAT_R8G8B8A8_UNORM, VK_FORMAT_B8G8R8A8_SRGB, VK_FORMAT_R8G8B8A8_SRGB};
    m_mainWindow.SurfaceFormat = ImGui_ImplVulkanH_SelectSurfaceFormat(m_physicalDevice, m_surface, formats, IM_ARRAYSIZE(formats), VK_COLOR_SPACE_SRGB_NONLINEAR_KHR);
    constexpr VkPresentModeKHR modes[] = {VK_PRESENT_MODE_FIFO_KHR};
    m_mainWindow.PresentMode = ImGui_ImplVulkanH_SelectPresentMode(m_physicalDevice, m_surface, modes, IM_ARRAYSIZE(modes));
    m_mainWindow.ClearEnable = true;
    m_mainWindow.ClearValue.color = {{0.94f, 0.95f, 0.98f, 1.0f}};
    ImGui_ImplVulkanH_CreateOrResizeWindow(m_instance, m_physicalDevice, m_device, &m_mainWindow,
        m_graphicsQueueFamily, nullptr, width, height, m_minImageCount);
    m_swapchainWidth = width; m_swapchainHeight = height;
}

void VulkanContext::uploadFonts()
{
    ImGui_ImplVulkanH_Frame& frame = m_mainWindow.Frames[m_mainWindow.FrameIndex];
    check(vkResetCommandPool(m_device, frame.CommandPool, 0));
    VkCommandBufferBeginInfo begin{}; begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; check(vkBeginCommandBuffer(frame.CommandBuffer, &begin));
    ImGui_ImplVulkan_CreateFontsTexture(frame.CommandBuffer); check(vkEndCommandBuffer(frame.CommandBuffer));
    VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &frame.CommandBuffer;
    check(vkQueueSubmit(m_graphicsQueue, 1, &submit, VK_NULL_HANDLE)); check(vkQueueWaitIdle(m_graphicsQueue));
    ImGui_ImplVulkan_DestroyFontUploadObjects();
}

VulkanContext::QueueFamilyIndices VulkanContext::findQueueFamilies(VkPhysicalDevice device) const
{
    QueueFamilyIndices indices; std::uint32_t count = 0; vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count); vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());
    for (std::uint32_t i = 0; i < count; ++i) { if ((families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) indices.graphicsFamily = i;
        VkBool32 present = VK_FALSE; vkGetPhysicalDeviceSurfaceSupportKHR(device, i, m_surface, &present);
        if (present == VK_TRUE) indices.presentFamily = i; if (indices.complete()) break; }
    return indices;
}

bool VulkanContext::isDeviceSuitable(VkPhysicalDevice device) const { return findQueueFamilies(device).complete() && supportsSwapchainExtension(device); }

bool VulkanContext::supportsSwapchainExtension(VkPhysicalDevice device) const
{
    std::uint32_t count = 0; vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count); vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data());
    for (const auto& extension : extensions) if (std::strcmp(extension.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) return true;
    return false;
}
