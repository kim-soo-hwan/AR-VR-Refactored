// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <stdio.h>

int main()
{
    // GLFW: initialize
    if (!glfwInit())
    {
        printf("Failed to initialize GLFW\n");
        return -1;
    }

    // GLFW: configure (OpenGL 3.3 core)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    // uncomment this to fix compilation on OS X
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // GLFW: create a window
    GLFWwindow* window = glfwCreateWindow(800, 600, "Hello World", NULL, NULL);
    if (!window)
    {
        printf("Failed to create GLFW window\n");
        glfwTerminate();
        return -1;
    }

    // GLFW: make the OpenGL context of this window current
    glfwMakeContextCurrent(window);

    // GLAD: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        printf("Failed to initialize GLAD\n");
        glfwTerminate();
        return -1;
    }
    printf("OpenGL %d.%d\n", GLVersion.major, GLVersion.minor);

    // render loop
    while (!glfwWindowShouldClose(window))
    {
        // render here
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f); // state-setting
        glClear(GL_COLOR_BUFFER_BIT);         // state-using

        // GLFW: swap buffers
        glfwSwapBuffers(window);

        // GLFW: poll IO events (keyboard and mouse events)
        glfwPollEvents();
    }

    // GLFW: clear all GLFW resources
    glfwTerminate();
    return 0;
}