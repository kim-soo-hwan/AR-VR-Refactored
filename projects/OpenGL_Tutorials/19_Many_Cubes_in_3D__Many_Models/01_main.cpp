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
    Window window(800, 600, "Many Cube in 3D");
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

    // translation vectors: G_t_O
    GLfloat cubePositions[10][3] = {
        { 0.0f,  0.0f,   0.0f},
        { 2.0f,  5.0f, -15.0f},
        {-1.5f, -2.2f,  -2.5f},
        {-3.8f, -2.0f, -12.3f},
        { 2.4f, -0.4f,  -3.5f},
        {-1.7f,  3.0f,  -7.5f},
        { 1.3f, -2.0f,  -2.5f},
        { 1.5f,  2.0f,  -2.5f},
        { 1.5f,  0.2f,  -1.5f},
        {-1.3f,  1.0f,  -1.5f}
    };

    // scene
    Scene scene;

    // model
    for (int i = 0; i < 10; i++)
    {
        // vetices
        shared_ptr<Model> cube = make_shared<Model>(36);
        cube->setVertexAttributes(vertices, 3, 2);

        // shader
        cube->setShaderProgram(shaderProgram);

        // transform
        cube->translate_R(cubePositions[i][0], cubePositions[i][1], cubePositions[i][2]);
        cube->rotate_R(i * glm::radians(20.f), 1.0f, 0.3f, 0.5f);
        cube->setModelViewProjectionMatrixName("model_view_projection");

        // add
        scene.addModel(cube);
    }

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 100.f);
    camera->translate_L(0.f, 0.f, -3.f); // view: move the camera backward
    scene.setCamera(camera);

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // draw        
        for (unsigned int i = 0; i < 10; i++)
        {
            // draw
            scene.draw();
        }

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}