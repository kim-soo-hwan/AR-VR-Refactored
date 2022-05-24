#include <cmath>
using namespace std;

#include <window.h>
#include <mesh.h>
#include <shader.h>

int main()
{
    // OpenGL window
    Window window(800, 600, "Greenish Triangle");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);
    
    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "default.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "uniform_color.fs");
    shaderProgram->attachAndLinkShaders();

    // vertex input
    GLfloat vertices[] = {
        -0.5f, -0.5f, 0.0f, // left  
         0.5f, -0.5f, 0.0f, // right 
         0.0f,  0.5f, 0.0f  // top   
    };

    // mesh
    Mesh mesh(3);
    mesh.setVertexAttributes(vertices, 3);
    mesh.setShaderProgram(shaderProgram);

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // get the running time in seconds
        GLfloat timeValue = glfwGetTime();

        // vary the green color in [0, 1]
        GLfloat greenValue = (sin(timeValue) / 2.0f) + 0.5f;

        // set the uniform variable
        shaderProgram->set("ourColor", 0.0f, greenValue, 0.0f, 1.0f);

        // draw triangles
        mesh.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}