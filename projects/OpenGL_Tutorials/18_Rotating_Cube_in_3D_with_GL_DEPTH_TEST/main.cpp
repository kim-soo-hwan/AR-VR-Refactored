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
    Window window(800, 600, "Rotating Cube in 3D");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);
    window.enable(GL_DEPTH_TEST);

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "model_view_projection_pass_texCoords.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "two_textures.fs");
    shaderProgram->attachAndLinkShaders();

    // texture
    shaderProgram->addTextureUnit("texture1", "container.jpg");
    shaderProgram->addTextureUnit("texture2", "awesomeface.png");

    // vertex input: O_p
    GLfloat vertices[] = {
        // bottom
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,   // 0
         0.5f, -0.5f, -0.5f,  1.0f, 0.0f,   // 1
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,   // 2
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,   // 2
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,   // 3
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,   // 0

        // top
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,   // 4
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,   // 5
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,   // 6
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,   // 6
        -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,   // 7
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,   // 4

        // back
        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,   // 7
        -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,   // 3
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,   // 0
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,   // 0
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,   // 4
        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,   // 7

        // front
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,   // 6
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,   // 2
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,   // 1
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,   // 1
         0.5f, -0.5f,  0.5f,  0.0f, 0.0f,   // 5
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,   // 6

        // left
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,   // 0
         0.5f, -0.5f, -0.5f,  1.0f, 1.0f,   // 1
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,   // 5
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,   // 5
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,   // 4
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,   // 0

        // right
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,   // 3
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,   // 2
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,   // 6
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,   // 6
        -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,   // 7
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,   // 3
     };

    // model
    shared_ptr<Model> cube = make_shared<Model>(36);
    cube->setVertexAttributes(vertices, 3, 2);
    cube->setShaderProgram(shaderProgram);
    cube->setModelViewProjectionMatrixName("model_view_projection");

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 100.f);
    camera->translate_L(0.f, 0.f, -3.f); // view: move the camera backward

    // scene
    Scene scene;
    scene.addModel(cube);
    scene.setCamera(camera);

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // model
        cube->resetTransformationMatrix();
        cube->rotate_R(glfwGetTime(), 0.5f, 1.f, 0.f);

        // draw
        scene.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}