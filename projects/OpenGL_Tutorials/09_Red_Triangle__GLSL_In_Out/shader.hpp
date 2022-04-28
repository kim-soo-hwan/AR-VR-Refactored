#pragma once

#include <string>
using namespace std;

// include glad to get all the required OpenGL headers
#include <glad/glad.h> 

class Shader
{
public:
    // constructor reads and builds the shader
    Shader(const char* vertexPath, const char* fragmentPath);

    // activate the shader program
    void use() const;

protected:
    void checkErrors(GLuint shader, const string &type);

protected:
    // shader program
    GLuint m_shaderProgram;
};