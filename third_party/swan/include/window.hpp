#pragma once

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Window
{
public:
    Window(const int width, const int height, const char* title);
    ~Window();

    void resizeWindowCallback(GLFWwindow* window, int width, int height);
    void setBackgroundColor(const GLclampf R, const GLclampf G, const GLclampf B, const GLclampf A);
    bool shouldClose() const;
    void wipeOut() const;
    void display() const;
    void processUserInputs();
    int getWidth() const;
    int getHeight() const;
    void setDepthEnabled(const bool enabled = true);

protected:
    // window
    GLFWwindow* m_window = NULL;
    GLclampf m_R = 1.f;
    GLclampf m_G = 1.f;
    GLclampf m_B = 1.f;
    GLclampf m_A = 1.f;
    bool m_depthEnabled = false;
    int m_width;
    int m_height;
};