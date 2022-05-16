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

// swan
#include <texture.h>

class Shader
{
public:
    // constructor
    Shader(const GLenum shaderType);

    // destructor
    virtual ~Shader();

    // load and compile the shader code
    bool loadAndCompile(const string &filePath);

    // getter
    GLuint id() const;

protected:
    // load the shader code from file
    bool load(const string &filePath, string &shaderCode) const;

    // compile the shader code
    bool compile(const string &shaderCode);

protected:
    GLuint _id;     // shader id
};


class ShaderProgram
{
public:
    // constructor
    ShaderProgram();

    // destructor
    virtual ~ShaderProgram();

    // create a shader from file
    bool createShader(const GLenum shaderType, const string &filePath);

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

    // set texture
    void setTexture(const shared_ptr<Texture> &texture);
    bool addTextureUnit(const string &name, const shared_ptr<Texture> &texture);

protected:
    // shaders
    vector<shared_ptr<Shader>> _shaders;

    // texture
    shared_ptr<Texture> _texture;

    // texture units
    GLint MAX_NUM_TEXTURE_UNITS;
    vector<shared_ptr<Texture>> _textureUnits;

    // shader program
    GLuint _id;
};

#endif // __SHADER_H__