#ifndef __WINDOW_H__
#define __WINDOW_H__

// std
#include <memory>
using namespace std;

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// swan
#include <camera.h>

class Window
{
public:
    // constructor
    Window(const int width, const int height, const char* title);

    // destructor
    virtual ~Window();

    // setter
    void setBackgroundColor(const GLclampf R, const GLclampf G, const GLclampf B, const GLclampf A);
    void setDepthEnabled(const bool enabled = true);

    // infinite loop
    bool shouldClose() const;
    void wipeOut() const;
    void display() const;
    void processUserInputs();

    // gettor
    int getWidth() const;
    int getHeight() const;
    float getRatio() const;

    // call-back functions
    void resizeWindowCallback(GLFWwindow* window, int width, int height);
    void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    void mouseMovementCallback(GLFWwindow* window, double posX, double posY) const;
    void mouseScrollCallback(GLFWwindow* window, double offsetX, double offsetY) const;

    // camera
    void setCamera(const shared_ptr<Camera> &camera);

protected:
    // window
    GLFWwindow* window_ = NULL;

    // size
    int width_;
    int height_;

    // background color
    GLclampf R_ = 1.f;
    GLclampf G_ = 1.f;
    GLclampf B_ = 1.f;
    GLclampf A_ = 1.f;

    // 3D
    bool depthEnabled_ = false;

    // camera
    shared_ptr<Camera> camera_;

    // mouse
    bool leftMouseButtonPressed_ = false;
    bool rightMouseButtonPressed_ = false;

    const float MOUSE_ROTATION_SENSITIVITY = 0.1f;
    const float MOUSE_TRANSLATION_SENSITIVITY = 0.01f;
};

#endif // __WINDOW_H__