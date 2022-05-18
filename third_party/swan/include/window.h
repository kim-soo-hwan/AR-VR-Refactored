#ifndef __WINDOW_H__
#define __WINDOW_H__

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Window
{
public:
    Window(const int width, const int height, const char* title);
    virtual ~Window();

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
    GLFWwindow* window_ = NULL;
    GLclampf R_ = 1.f;
    GLclampf G_ = 1.f;
    GLclampf B_ = 1.f;
    GLclampf A_ = 1.f;
    bool depthEnabled_ = false;
    int width_;
    int height_;
};

#endif // __WINDOW_H__