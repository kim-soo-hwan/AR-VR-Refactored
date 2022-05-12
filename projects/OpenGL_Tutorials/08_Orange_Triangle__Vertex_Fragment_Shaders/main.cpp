#include <window.h>
#include <mesh.h>
#include <shader.h>

int main()
{
    // OpenGL window
    Window window(800, 600, "Orange Triangle");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    //shaderProgram->createShader(GL_VERTEX_SHADER,   "default.vs");
    //shaderProgram->createShader(GL_FRAGMENT_SHADER, "orange.fs");
    //shaderProgram->attachAndLinkShaders();
    shaderProgram->attachAndLinkShaders(GL_VERTEX_SHADER,   "default.vs",
                                        GL_FRAGMENT_SHADER, "orange.fs");

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

        // draw triangles
        mesh.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}