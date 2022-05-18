#include <window.h>
#include <mesh.h>
#include <shader.h>
#include <texture.h>
#include <model.h>

int main()
{
    // OpenGL window
    Window window(800, 600, "Rotating Rectangle");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShader(GL_VERTEX_SHADER,   "transform_pass_texCoords.vs");
    shaderProgram->createShader(GL_FRAGMENT_SHADER, "two_textures.fs");
    shaderProgram->attachAndLinkShaders();

    // texture
    shaderProgram->addTextureUnit("texture1", "container.jpg",   GL_RGB,  GL_RGB,  false);
    shaderProgram->addTextureUnit("texture2", "awesomeface.png", GL_RGBA, GL_RGBA, true);

    // vertex input: O_p
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

    // model
    Model rectangle(4);
    rectangle.setVertexAttributes(positions, 3);
    rectangle.setVertexAttributes(texCoords, 2);
    rectangle.setElementIndices(indices, 6);
    rectangle.setShaderProgram(shaderProgram);

    // transform
    rectangle.setModelMatrixName("transform");

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // reset
        rectangle.resetModelMatrix();

        // scale down by one half
        rectangle.scale(0.5f, 0.5f, 0.5f);

        // translate
        rectangle.translate(0.5f, -0.5f, 0.0f);

        // rotate by time degrees about the z-axis (unit vector)
        rectangle.rotate(50.f * glfwGetTime(), 0.0f, 0.0f, 1.0f);

        // draw        
        rectangle.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}