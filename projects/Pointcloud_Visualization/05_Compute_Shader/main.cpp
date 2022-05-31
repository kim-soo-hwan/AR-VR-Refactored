#include <iostream>
#include <ifstream>
using namespace std;

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// swan
#include <window.h>
#include <mesh.h>
#include <shader.h>
#include <texture.h>
#include <model.h>
#include <scene.h>
#include <camera.h>
#include <axes.h>

int main()
{
    // OpenGL window
    Window window(800, 600, "Pointcloud Visualization");
    window.setBackgroundColor(0.f, 0.f, 0.f, 1.0f);
    window.enable(GL_DEPTH_TEST);
        
    // create the image texture
    int tex_w = 512, tex_h = 512; // dimensions of the image
    GLuint tex_output;
    glGenTextures(1, &tex_output);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex_output);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, tex_w, tex_h, 0, GL_RGBA, GL_FLOAT, NULL);
    glBindImageTexture(0, tex_output, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    // determine the work group size
    int work_grp_cnt[3];
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &work_grp_cnt[0]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &work_grp_cnt[1]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &work_grp_cnt[2]);
    cout << "max global (total) work group counts x:" << work_grp_cnt[0] << ", y:" << work_grp_cnt[1] << ", z:" << work_grp_cnt[2] << endl;

    int work_grp_size[3];
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &work_grp_size[0]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &work_grp_size[1]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &work_grp_size[2]);
    cout << "max local (in one shader) work group sizes x:" << work_grp_size[0] << ", y:" << work_grp_size[1] << ", z:" << work_grp_size[2] << endl;

    int work_grp_inv;
    glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &work_grp_inv);
    cout << "max local work group invocations " << work_grp_inv << endl;

    // shader
    // shader input file streams
    ifstream shaderFile;

    // ensure ifstream objects can throw exceptions
    shaderFile.exceptions(ifstream::failbit | ifstream::badbit);

    // open files
    shaderFile.open("ray_tracing.cs");

    // read file's buffer contents into streams
    stringstream shaderStream;
    shaderStream << shaderFile.rdbuf();

    // close file handlers
    shaderFile.close();

    // convert stream into string
    string shaderCode = shaderStream.str();

    // shader
    GLuint ray_shader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(ray_shader, 1, &shaderCode.c_str(), NULL);
    glCompileShader(ray_shader);

    // shader program
    GLuint ray_program = glCreateProgram();
    glAttachShader(ray_program, ray_shader);
    glLinkProgram(ray_program);
    
    // dispatch the shaders
    // drawing loop
    while(!glfwWindowShouldClose(window))
    {

        { // launch compute shaders!
            glUseProgram(ray_program);
            glDispatchCompute((GLuint)tex_w, (GLuint)tex_h, 1);
        }
        
        // make sure writing to image has finished before read
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
        
        { // normal drawing pass
            glClear(GL_COLOR_BUFFER_BIT);
            glUseProgram(quad_program);
            glBindVertexArray(quad_vao);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tex_output);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        }
        
        glfwPollEvents();
        if (GLFW_PRESS == glfwGetKey(window, GLFW_KEY_ESCAPE)) {
            glfwSetWindowShouldClose(window, 1);
        }
        glfwSwapBuffers(window);
    }

    return 0;
}