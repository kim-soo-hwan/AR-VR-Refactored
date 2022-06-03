#ifndef __SHADER_H__
#define __SHADER_H__

// std
#include <string>
#include <vector>
#include <memory>   // shared_prt, make_shared
#include <utility>  // pair, make_pair
using namespace std;

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>

// swan
#include <texture.h>

class Shader
{
public:
    // constructors
    Shader(const GLenum shaderType);
    Shader(const GLenum shaderType, const string &filePath);

    // destructor
    virtual ~Shader();

    // load and compile the shader code
    bool loadAndCompile(const string &filePath);

    // compile the shader code
    bool compile(const string &shaderCode);

    // getter
    GLuint id() const;

protected:
    // load the shader code from file
    bool load(const string &filePath, string &shaderCode) const;

protected:
    GLuint id_;     // shader id
};


class ShaderProgram
{
public:
    // constructor
    ShaderProgram();

    // destructor
    virtual ~ShaderProgram();

    // shader
    bool createShaderFromFile(const GLenum shaderType, const string &filePath);
    bool createShaderFromString(const GLenum shaderType, const string &code);
    void addShader(const shared_ptr<Shader> &shader);

    // attach and link shaders
    bool attachAndLinkShaders() const;

    // use the shader program
    void use() const;

    // set uniform variables
    void set(const string& name, const GLint value) const;
    void set(const string& name, const GLuint value) const;
    void set(const string& name, const GLfloat value) const;
    void set(const string& name, const GLfloat v0, const GLfloat v1) const;
    void set(const string& name, const GLfloat v0, const GLfloat v1, const GLfloat v2) const;
    void set(const string& name, const GLfloat v0, const GLfloat v1, const GLfloat v2, const GLfloat v3) const;
    void set(const string& name, const unsigned int dim, GLsizei count, const GLfloat* value) const;
    void set(const string& name, const glm::mat4& T) const;
    void set(const string& name, const glm::mat3x4& T) const;

    // set texture
    void setTexture(const shared_ptr<Texture> &texture);
    bool setTexture(const string& filePath, const bool flipVertically = true);
    bool setTexture(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically = true);
    bool addTextureUnit(const string &name, const shared_ptr<Texture> &texture);
    bool addTextureUnit(const string &name, const string& filePath, const bool flipVertically = true);
    bool addTextureUnit(const string &name, const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically = true);

protected:
    // shaders
    vector<shared_ptr<Shader>> shaders_;

    // texture
    shared_ptr<Texture> texture_;

    // texture units
    GLint MAX_NUM_TEXTURE_UNITS_;
    vector<shared_ptr<Texture>> textureUnits_;

    // shader program
    GLuint id_;
};

#endif // __SHADER_H__