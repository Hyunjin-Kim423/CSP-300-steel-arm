#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <functional>

class Window
{
public:
    Window(int width, int height, const wchar_t* title);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool shouldClose() const { return m_shouldClose; }
    void pollEvents();
    void setMessageHandler(std::function<bool(HWND, UINT, WPARAM, LPARAM)> handler);

    HWND nativeHandle() const { return m_hwnd; }
    HINSTANCE instanceHandle() const { return m_instance; }
    SIZE clientSize() const;

private:
    static LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

    HINSTANCE m_instance = nullptr;
    HWND m_hwnd = nullptr;
    bool m_shouldClose = false;
    std::function<bool(HWND, UINT, WPARAM, LPARAM)> m_messageHandler;
};
