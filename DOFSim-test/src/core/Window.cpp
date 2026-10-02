#include "core/Window.hpp"

#include <stdexcept>

namespace
{
constexpr wchar_t kWindowClassName[] = L"Vulkan2DOFWindowClass";
}

Window::Window(int width, int height, const wchar_t* title)
    : m_instance(GetModuleHandleW(nullptr))
{
    WNDCLASSEXW windowClass{};
    windowClass.cbSize = sizeof(WNDCLASSEXW);
    windowClass.style = CS_HREDRAW | CS_VREDRAW;
    windowClass.lpfnWndProc = &Window::windowProc;
    windowClass.hInstance = m_instance;
    windowClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    windowClass.lpszClassName = kWindowClassName;

    if (RegisterClassExW(&windowClass) == 0)
    {
        const DWORD error = GetLastError();
        if (error != ERROR_CLASS_ALREADY_EXISTS)
        {
            throw std::runtime_error("Failed to register the Win32 window class.");
        }
    }

    RECT clientRect{0, 0, width, height};
    AdjustWindowRect(&clientRect, WS_OVERLAPPEDWINDOW, FALSE);

    m_hwnd = CreateWindowExW(
        0,
        kWindowClassName,
        title,
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        clientRect.right - clientRect.left,
        clientRect.bottom - clientRect.top,
        nullptr,
        nullptr,
        m_instance,
        this);

    if (m_hwnd == nullptr)
    {
        UnregisterClassW(kWindowClassName, m_instance);
        throw std::runtime_error("Failed to create the Win32 window.");
    }

    ShowWindow(m_hwnd, SW_SHOW);
    UpdateWindow(m_hwnd);
}

Window::~Window()
{
    if (m_hwnd != nullptr && IsWindow(m_hwnd))
    {
        DestroyWindow(m_hwnd);
    }

    if (m_instance != nullptr)
    {
        UnregisterClassW(kWindowClassName, m_instance);
    }
}

void Window::pollEvents()
{
    MSG message{};
    while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
    {
        if (message.message == WM_QUIT)
        {
            m_shouldClose = true;
            continue;
        }

        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
}

void Window::setMessageHandler(std::function<bool(HWND, UINT, WPARAM, LPARAM)> handler)
{
    m_messageHandler = std::move(handler);
}

SIZE Window::clientSize() const
{
    RECT rect{};
    GetClientRect(m_hwnd, &rect);
    return {rect.right - rect.left, rect.bottom - rect.top};
}

LRESULT CALLBACK Window::windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    Window* window = reinterpret_cast<Window*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    if (message == WM_NCCREATE)
    {
        const auto* createInfo = reinterpret_cast<CREATESTRUCTW*>(lParam);
        window = static_cast<Window*>(createInfo->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(window));
    }

    if (window != nullptr && window->m_messageHandler != nullptr &&
        window->m_messageHandler(hwnd, message, wParam, lParam))
    {
        return 1;
    }

    switch (message)
    {
    case WM_CLOSE:
        if (window != nullptr)
        {
            window->m_shouldClose = true;
        }
        DestroyWindow(hwnd);
        return 0;

    case WM_DESTROY:
        if (window != nullptr)
        {
            window->m_hwnd = nullptr;
            window->m_shouldClose = true;
        }
        PostQuitMessage(0);
        return 0;

    default:
        return DefWindowProcW(hwnd, message, wParam, lParam);
    }
}
