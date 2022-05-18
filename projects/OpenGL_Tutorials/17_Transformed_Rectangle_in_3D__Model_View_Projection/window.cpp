#include <iostream>
#include <functional>
using namespace std;

#include "window.hpp"

// call back functions
// ref) https://blog.mbedded.ninja/programming/languages/c-plus-plus/passing-a-cpp-member-function-to-a-c-callback/
// ref) https://www.youtube.com/watch?v=j-pD1mlsOqE
template <typename T>
struct Callback;

// callback class
template <typename Ret, typename... Params>
struct Callback<Ret(Params...)>
{
    template <typename... Args>
    static Ret callback(Args... args)
    {
        return func(args...);
    }

    // static member functor
    static std::function<Ret(Params...)> func;
};

// static member functor
template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

void Window::resizeWindowCallback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new dimensions
    glViewport(0, 0, width, height);

    m_width = width;
    m_height = height;
}

Window::Window(const int width, const int height, const char* title)
: m_width(width),
  m_height(height)
{
    // GLFW: initialize and configure (OpenGL 3.3 core)
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    // uncomment this to fix compilation on OS X
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // GLFW: create a window
    m_window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (m_window == NULL)
    {
        cout << "Error: Failed to create a GLFW window" << endl;
        glfwTerminate();
        return;
    }

    // GLFW: make the OpenGL context of this window current
    glfwMakeContextCurrent(m_window);

    // GLFW: set callback functions
    Callback<void(GLFWwindow*, int, int)>::func = std::bind(&Window::resizeWindowCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    GLFWframebuffersizefun resizeWindowCallback = static_cast<GLFWframebuffersizefun>(Callback<void(GLFWwindow*, int, int)>::callback);
    glfwSetFramebufferSizeCallback(m_window, resizeWindowCallback);

    // GLAD: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        cout << "Error: Failed to initialize GLAD" << endl;
        glfwTerminate();
        m_window = NULL;
        return;
    }

    cout << "OpenGL " << GLVersion.major << "." << GLVersion.minor << endl;
}

Window::~Window()
{
    // GLFW: clear all GLFW resources
    if (m_window)
    {
        glfwDestroyWindow(m_window);
        glfwTerminate();
    }
}

void Window::setBackgroundColor(const GLclampf R, const GLclampf G, const GLclampf B, const GLclampf A)
{
    m_R = R;
    m_G = G;
    m_B = B;
    m_A = A;
}

bool Window::shouldClose() const
{
    if (m_window == NULL) return true;

    return glfwWindowShouldClose(m_window);
}

void Window::wipeOut() const
{
    // wipe out
    glClearColor(m_R, m_G, m_B, m_A);   // a state-setting function

    // a state-using function
    if (m_depthEnabled) glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
    else                glClear(GL_COLOR_BUFFER_BIT);
}

void Window::display() const
{
    // GLFW: swap buffers
    glfwSwapBuffers(m_window);
}

void Window::processUserInputs()
{
    // GLFW: poll IO events (keyboard and mouse events)
    glfwPollEvents();

    // ESC
    if (glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(m_window, true);
}

int Window::getWidth() const
{
    return m_width;
}

int Window::getHeight() const
{
    return m_height;
}