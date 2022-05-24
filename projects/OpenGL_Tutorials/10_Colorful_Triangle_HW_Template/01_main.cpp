#include <window.h>
#include <mesh.h>
#include <shader.h>

int main()
{
    // OpenGL window
    Window window(800, 600, "Colorful Triangle");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "pass_color.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "default.fs");
    shaderProgram->attachAndLinkShaders();

    // vertex input
    GLfloat vertices[] = {
        // positions            // colors
         0.5f, -0.5f, 0.0f,     1.0f, 0.0f, 0.0f,   // bottom right
        -0.5f, -0.5f, 0.0f,     0.0f, 1.0f, 0.0f,   // bottom left
         0.0f,  0.5f, 0.0f,     0.0f, 0.0f, 1.0f,   // top
    };

    // mesh
    Mesh mesh(3);
    mesh.setVertexAttributes(vertices, 3, 3);
    mesh.setShaderProgram(shaderProgram);
    
    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // draw triangles
        mesh.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}