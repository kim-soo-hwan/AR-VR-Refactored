#pragma once

// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stdio.h>

// vertex shader
const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 vertexPosition;\n"
"void main()\n"
"{\n"
"   gl_Position = vec4(vertexPosition.x, vertexPosition.y, vertexPosition.z, 1.0);\n"
"}\0";

// fragment shader
const char* fragmentShaderSource = "#version 330 core\n"
"out vec4 fragmentColor;\n"
"void main()\n"
"{\n"
"   fragmentColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
"}\n\0";

GLuint getDefaultShader()
{
    // create and compile the vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // check for shader compile errors
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        printf("ERROR::SHADER::VERTEX::COMPILATION_FAILED\n");
        printf("%s\n", infoLog);
    }

    // create and compile the fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // check for shader compile errors
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        printf("ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n");
        printf("%s\n", infoLog);
    }

    // create a shader program
    GLuint shaderProgram = glCreateProgram();

    // attach the vertex and fragment shaders into the shader program
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // check for shader program link errors
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        printf("ERROR::SHADER::PROGRAM::LINKING_FAILED\n");
        printf("%s\n", infoLog);
    }

    // delete the vertex and fragment shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProgram;
}