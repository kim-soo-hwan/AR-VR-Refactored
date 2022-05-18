#include <window.h>
#include <mesh.h>
#include <shader.h>
#include <texture.h>
#include <model.h>
#include <scene.h>
#include <camera.h>

int main()
{
    // OpenGL window
    Window window(800, 600, "Transformed Rectangle in 3D");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShader(GL_VERTEX_SHADER,   "model_view_projection_pass_texCoords.vs");
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
    shared_ptr<Model> rectangle = make_shared<Model>(4);
    rectangle->setVertexAttributes(positions, 3);
    rectangle->setVertexAttributes(texCoords, 2);
    rectangle->setElementIndices(indices, 6);
    rectangle->setShaderProgram(shaderProgram);
    rectangle->rotate(-55.f, 1.f, 0.f, 0.f); // model: tilt down by 55 degrees
    rectangle->setTransformationMatrixName("model_view_projection");

    // camera
    Camera camera(45.f, window.getRatio(), 0.1f, 100.f);
    camera.translate(0.f, 0.f, -3.f); // view: move the camera backward
    camera.addModel(rectangle);

    // scene
    Scene scene;
    scene.addModel(rectangle);
    scene.setCamera(camera);

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // capture
        scene.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}