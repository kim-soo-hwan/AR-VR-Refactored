// std
#include <iostream>
#include <functional>
#include <algorithm>    // min, max
using namespace std;
using std::cout;

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>    // half_pi()

// swan
#include <window.h>

// external variables
//extern glm::vec3 g_camerPosition;
//extern glm::vec3 G_WORLD_UP_VECTOR;
//extern glm::vec3 G_WORLD_FORWARD_VECTOR;
//const glm::vec3 G_WORLD_RIGHT_VECTOR = glm::cross(G_WORLD_FORWARD_VECTOR, G_WORLD_UP_VECTOR);

// call back functions
// ref) https://blog.mbedded.ninja/programming/languages/c-plus-plus/passing-a-cpp-member-function-to-a-c-callback/
// ref) https://www.youtube.com/watch?v=j-pD1mlsOqE
template <unsigned int N, typename T>
struct Callback;

// callback class
template <unsigned int N, typename Ret, typename... Params>
struct Callback<N, Ret(Params...)>
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
template <unsigned int N, typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<N, Ret(Params...)>::func;

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void Window::resizeWindowCallback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new dimensions
    glViewport(0, 0, width, height);

    width_ = width;
    height_ = height;
}

// glfw: whenever the mouse clicks, this callback is called
void Window::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        leftMouseButtonPressed_ = true;
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
    {
        leftMouseButtonPressed_ = false;
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
    {
        rightMouseButtonPressed_ = true;
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
    {
        rightMouseButtonPressed_ = false;
    }
}

// glfw: whenever the mouse moves, this callback is called
void Window::mouseMovementCallback(GLFWwindow* window, double posX, double posY) const
{
    // camera is required
    if (!camera_) return;

    // last mouse position
    static float lastX = posX;
    static float lastY = posY;

    // offset
    const float offsetX = posX - lastX;
    const float offsetY = posY - lastY;
    lastX = posX;
    lastY = posY;

    // left mouse button: pan/tilt
    if (leftMouseButtonPressed_)
    {
        // pan and tilt angles in radian
        float panAngle  = MOUSE_ROTATION_SENSITIVITY * offsetX; // pan: turn left
        float tiltAngle = MOUSE_ROTATION_SENSITIVITY * offsetY; // tilt: turn up

        // limit: pan/tilt in [-pi/2, pi/2]
        const float HALF_PI = glm::half_pi<float>();
        panAngle  = std::min(std::max(panAngle,  -HALF_PI), HALF_PI);
        tiltAngle = std::min(std::max(tiltAngle, -HALF_PI), HALF_PI);

        // convert degrees to radians
        camera_->pan(panAngle);
        camera_->tilt(tiltAngle);
    }

    // right mouse button: move right/left or up/down
    if (rightMouseButtonPressed_)
    {
        camera_->moveLeft(MOUSE_TRANSLATION_SENSITIVITY * offsetX); // move left
        camera_->moveUp  (MOUSE_TRANSLATION_SENSITIVITY * offsetY); // move up
    }
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
void Window::mouseScrollCallback(GLFWwindow* window, double offsetX, double offsetY) const
{
    // move forward and backward
    if (camera_) camera_->moveForward(offsetY);
}

Window::Window(const int width, const int height, const char* title)
: width_(width),
  height_(height),
  title_(title)
{
    // GLFW: initialize and configure (OpenGL 3.3 core)
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
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
    Callback<0, void(GLFWwindow*, int, int)>::func = std::bind(&Window::resizeWindowCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    GLFWframebuffersizefun resizeWindowCallback = static_cast<GLFWframebuffersizefun>(Callback<0, void(GLFWwindow*, int, int)>::callback);
    glfwSetFramebufferSizeCallback(window_, resizeWindowCallback);

    Callback<1, void(GLFWwindow*, int, int, int)>::func = std::bind(&Window::mouseButtonCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    GLFWmousebuttonfun mouseButtonCallback = static_cast<GLFWmousebuttonfun>(Callback<1, void(GLFWwindow*, int, int, int)>::callback);
    glfwSetMouseButtonCallback(window_, mouseButtonCallback);

    Callback<2, void(GLFWwindow*, double, double)>::func = std::bind(&Window::mouseMovementCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    GLFWcursorposfun mouseMovementCallback = static_cast<GLFWcursorposfun>(Callback<2, void(GLFWwindow*, double, double)>::callback);
    glfwSetCursorPosCallback(window_, mouseMovementCallback);

    Callback<3, void(GLFWwindow*, double, double)>::func = std::bind(&Window::mouseScrollCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    GLFWscrollfun mouseScrollCallback = static_cast<GLFWscrollfun>(Callback<3, void(GLFWwindow*, double, double)>::callback);
    glfwSetScrollCallback(window_, mouseScrollCallback);

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

    // frame per second
    static double previousTime = glfwGetTime();
    static int frameCount = 0;

    // for every 100 frames
    if (frameCount > 100)
    {
        // elapsed time
        const double currentTime = glfwGetTime();
        const double elapsedTime = currentTime - previousTime;
        const double fps = (double)frameCount / elapsedTime;

        // you can't display faster than the refresh rate of the monitor
        // (around 60Hz or 60FPS)
        char fpsMessage[128];
        sprintf(fpsMessage, " @ %.2f FPS (Max 60 FPS)", fps);
        const string title = title_ + fpsMessage;
        glfwSetWindowTitle(window_, title.c_str());

        // next
        previousTime = currentTime;
        frameCount = 0;
    }
    frameCount++;

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
    
    // camera
    if (!camera_) return;

    // time
    static float lastTime = (float)glfwGetTime();
    const float currentTime = (float)glfwGetTime();
    const float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    // camera speed
    const float cameraSpeed = 2.5f;
    const float displacement = cameraSpeed * deltaTime;

    // move camera
    if (glfwGetKey(window_, GLFW_KEY_F) == GLFW_PRESS) camera_->moveForward(displacement);  // move forward
    if (glfwGetKey(window_, GLFW_KEY_B) == GLFW_PRESS) camera_->moveBackward(displacement); // move backward
    if (glfwGetKey(window_, GLFW_KEY_H) == GLFW_PRESS) camera_->moveLeft(displacement);     // move left
    if (glfwGetKey(window_, GLFW_KEY_L) == GLFW_PRESS) camera_->moveRight(displacement);    // move right
    if (glfwGetKey(window_, GLFW_KEY_J) == GLFW_PRESS) camera_->moveDown(displacement);     // move up
    if (glfwGetKey(window_, GLFW_KEY_K) == GLFW_PRESS) camera_->moveUp(displacement);       // move down

    // print view
    if (glfwGetKey(window_, GLFW_KEY_V) == GLFW_PRESS)
    {
        glm::mat4 view = camera_->getViewMatrix();
        for(int row = 0; row < 4; row++)
        {
            for(int col = 0; col < 4; col++)
            {
                cout << view[col][row] << ", ";
            }
            cout << endl;
        }
        cout << endl;
    }
}

int Window::getWidth() const
{
    return width_;
}

int Window::getHeight() const
{
    return height_;
}

float Window::getRatio() const
{
    return (float)width_/(float)height_;
}

void Window::enable(const GLenum capability)
{
    if (capability == GL_DEPTH_TEST) depthEnabled_ = true;
    glEnable(capability);
}

void Window::disable(const GLenum capability)
{
    if (capability == GL_DEPTH_TEST) depthEnabled_ = false;
    glDisable(capability);
}

// camera
void Window::setCamera(const shared_ptr<Camera> &camera)
{
    camera_ = camera;
}