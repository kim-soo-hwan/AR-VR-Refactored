#include "window.hpp"
#include "mesh.hpp"
#include "shader.hpp"

int main()
{
    // OpenGL window
    Window window(800, 600, "Orange Triangle");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // vertex input
    GLfloat vertices[] = {
        -0.5f, -0.5f, 0.0f, // left  
         0.5f, -0.5f, 0.0f, // right 
         0.0f,  0.5f, 0.0f  // top   
    };

    // mesh
    Mesh mesh(3);
    mesh.setVertexPositions(3, vertices);

    // shader
    Shader shader("default.vert", "orange.frag");

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // activate the shader
        shader.use();

        // draw triangles
        mesh.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}