// std
#include <iostream>
using namespace std;

// Intel RealSense
#include <librealsense2/rs.hpp>

// swan
#include <window.h>
#include <mesh.h>
#include <shader.h>
#include <texture.h>

// ref) texture class in https://github.com/IntelRealSense/librealsense/blob/master/examples/example.hpp
int main() try
{
    // Intel RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    pipe.start(); // start streaming with the default recommended configuration

    // OpenGL window
    Window window(640, 480, "Intel RealSense Image Viewer");
    window.setBackgroundColor(0.f, 0.f, 0.f, 1.0f);

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "pass_texCoords.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "single_texture.fs");
    shaderProgram->attachAndLinkShaders();

    // texture
    shared_ptr<Texture> texture = make_shared<Texture>();
    shaderProgram->setTexture(texture);

    // vertex input
    GLfloat positions[] = {
         1.f,  1.f, 0.0f, // top right
         1.f, -1.f, 0.0f, // bottom right
        -1.f, -1.f, 0.0f, // bottom left
        -1.f,  1.f, 0.0f, // top left 
    };

    GLfloat texCoords[] = {
        1.0f, 0.0f, // bottom right
        1.0f, 1.0f, // top right
        0.0f, 1.0f, // top left
        0.0f, 0.0f, // bottom left
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

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // capture
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();

        // texture
        auto format = color.get_profile().format();
        auto width  = color.get_width();
        auto height = color.get_height();
        //cout << "width: " << width << ", height: " << height << ", format: " << format << endl;

        switch (format)
        {
        case rs2_format::RS2_FORMAT_RGB8:
            texture->setImage(GL_RGB, width, height, GL_RGB, (const unsigned char*)color.get_data()); // default
            break;
        case rs2_format::RS2_FORMAT_RGBA8:
            texture->setImage(GL_RGBA, width, height, GL_RGBA, (const unsigned char*)color.get_data());
            break;
        default:
            throw std::runtime_error("The requested format is not supported by this demo!");
        }

        // draw        
        mesh.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}