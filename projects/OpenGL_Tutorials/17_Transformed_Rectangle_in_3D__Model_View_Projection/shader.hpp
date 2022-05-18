#pragma once

// STL
#include <string>
#include <vector>
#include <functional>   // std::reference_wrapper
using namespace std;

// include glad to get all the required OpenGL headers
#include <glad/glad.h> 

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// texture
#include "texture.hpp"

class Shader
{
public:
    // constructor reads and builds the shader
    Shader(const char* vertexPath, const char* fragmentPath);

    // activate the shader program
    void use() const;
    void begin() const;
    void end() const;

    // utility uniform functions
    void set(const string& name, GLint value) const;
    void set(const string& name, GLuint value) const;
    void set(const string& name, GLfloat value) const;
    void set(const string& name, GLfloat v0, GLfloat v1) const;
    void set(const string& name, GLfloat v0, GLfloat v1, GLfloat v2) const;
    void set(const string& name, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3) const;
    void set(const string& name, const unsigned int dim, GLsizei count, const GLfloat* value) const;
    void set(const string& name, const glm::mat4& T) const;

    void set(const string& name, const Texture& texture);

protected:
    void checkErrors(GLuint shader, const string& type);

protected:
    // shader program
    GLuint m_shaderProgram;

    // textures
    vector<reference_wrapper<const Texture>> m_textures;
};