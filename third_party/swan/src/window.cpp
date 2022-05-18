// STL
#include <iostream>
#include <functional>
using namespace std;

// GLM
#include <glm/glm.hpp>

// Our Library
#include <window.h>

// external variables
//extern glm::vec3 g_camerPosition;
//extern glm::vec3 G_WORLD_UP_VECTOR;
//extern glm::vec3 G_WORLD_FORWARD_VECTOR;
//const glm::vec3 G_WORLD_RIGHT_VECTOR = glm::cross(G_WORLD_FORWARD_VECTOR, G_WORLD_UP_VECTOR);

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

    width_ = width;
    height_ = height;
}

Window::Window(const int width, const int height, const char* title)
: width_(width),
  height_(height)
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
    window_ = glfwCreateWindow(width, height, title, NULL, NULL);
    if (window_ == NULL)
    {
        cout << "Error: Failed to create a GLFW window" << endl;
        glfwTerminate();
        return;
    }

    // GLFW: make the OpenGL context of this window current
    glfwMakeContextCurrent(window_);

    // GLFW: set callback functions
    Callback<void(GLFWwindow*, int, int)>::func = std::bind(&Window::resizeWindowCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    GLFWframebuffersizefun resizeWindowCallback = static_cast<GLFWframebuffersizefun>(Callback<void(GLFWwindow*, int, int)>::callback);
    glfwSetFramebufferSizeCallback(window_, resizeWindowCallback);

    // GLAD: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        cout << "Error: Failed to initialize GLAD" << endl;
        glfwTerminate();
        window_ = NULL;
        return;
    }

    cout << "OpenGL " << GLVersion.major << "." << GLVersion.minor << endl;
}

Window::~Window()
{
    // GLFW: clear all GLFW resources
    if (window_)
    {
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

void Window::setBackgroundColor(const GLclampf R, const GLclampf G, const GLclampf B, const GLclampf A)
{
    R_ = R;
    G_ = G;
    B_ = B;
    A_ = A;
}

bool Window::shouldClose() const
{
    if (window_ == NULL) return true;

    return glfwWindowShouldClose(window_);
}

void Window::wipeOut() const
{
    // wipe out
    glClearColor(R_, G_, B_, A_);   // a state-setting function

    // a state-using function
    if (depthEnabled_) glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
    else               glClear(GL_COLOR_BUFFER_BIT);
}

void Window::display() const
{
    // GLFW: swap buffers
    glfwSwapBuffers(window_);
}

void Window::processUserInputs()
{
    // GLFW: poll IO events (keyboard and mouse events)
    glfwPollEvents();

    // ESC
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window_, true);

    // time
    static float lastTime = (float)glfwGetTime();
    const float currentTime = (float)glfwGetTime();
    const float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    // camera speed
    const float cameraSpeed = 2.5f;
    const float displacement = cameraSpeed * deltaTime;

    // // move forward
    // if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS)
    //     g_camerPosition += displacement * G_WORLD_FORWARD_VECTOR;

    // // move backward
    // if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS)
    //     g_camerPosition -= displacement * G_WORLD_FORWARD_VECTOR;

    // // move right
    // if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS)
    //     g_camerPosition += displacement * G_WORLD_RIGHT_VECTOR;

    // // move left
    // if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS)
    //     g_camerPosition -= displacement * G_WORLD_RIGHT_VECTOR;
}

int Window::getWidth() const
{
    return width_;
}

int Window::getHeight() const
{
    return height_;
}

void Window::setDepthEnabled(const bool enabled)
{
    depthEnabled_ = enabled;
    if (depthEnabled_) glEnable(GL_DEPTH_TEST);
    else               glDisable(GL_DEPTH_TEST);
}