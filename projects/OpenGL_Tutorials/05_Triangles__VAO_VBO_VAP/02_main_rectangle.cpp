#include <window.h>
#include <mesh.h>

#ifdef __APPLE__
#include <utility.h>
#endif

int main()
{
    // OpenGL window
    Window window(800, 600, "Rectangle");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // vertex input
    float vertices[] = {
         0.5f,  0.5f, 0.0f,  // top right
         0.5f, -0.5f, 0.0f,  // bottom right
        -0.5f,  0.5f, 0.0f,  // top left 

         0.5f, -0.5f, 0.0f,  // bottom right
        -0.5f, -0.5f, 0.0f,  // bottom left
        -0.5f,  0.5f, 0.0f,  // top left 
    };

    // mesh
    Mesh mesh(6);
    mesh.setVertexAttributes(vertices, 3);

#ifdef __APPLE__
    // shader
    uint32_t shaderProgram = generateAndUseDefaultShaderProgram();
#endif

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

#ifdef __APPLE__
    // shader
    deleteDefaultShaderProgram(shaderProgram);
#endif

    return 0;
}