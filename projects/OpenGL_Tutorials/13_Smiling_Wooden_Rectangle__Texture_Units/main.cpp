#include <window.h>
#include <mesh.h>
#include <shader.h>
#include <texture.h>

int main()
{
    // OpenGL window
    Window window(800, 600, "Smiling Wooden Rectangle");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "pass_texCoords.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "two_textures.fs");
    shaderProgram->attachAndLinkShaders();

    // vertex input
    GLfloat positions[] = {
         0.5f,  0.5f, 0.0f, // top right
         0.5f, -0.5f, 0.0f, // bottom right
        -0.5f, -0.5f, 0.0f, // bottom left
        -0.5f,  0.5f, 0.0f, // top left 
    };

    GLfloat texCoords[] = {
        1.0f, 1.0f, // top right
        1.0f, 0.0f, // bottom right
        0.0f, 0.0f, // bottom left
        0.0f, 1.0f, // top left
    };

    // index input
    GLuint indices[] = {    // note that the index starts from 0!
        0, 1, 3,            // first triangle
        1, 2, 3             // second triangle
    };

    // mesh
    Mesh mesh(4);
    mesh.setVertexAttributes(positions, 3);
    mesh.setVertexAttributes(texCoords, 2);
    mesh.setElementIndices(indices, 6);
    mesh.setShaderProgram(shaderProgram);

    // Texture
    shaderProgram->addTextureUnit("texture1", "container.jpg");
    shaderProgram->addTextureUnit("texture2", "awesomeface.png");

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