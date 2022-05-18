#pragma once

#include <string>
using namespace std;

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Texture
{
public:
    Texture();
    Texture(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically = false);
    ~Texture();

    void bind() const;
    void bind(const GLuint textureUnit) const;
    void unbind() const;
    bool loadImage(const string & filePath, const GLint internalFormat, const GLenum format, const bool flipVertically = false);
    void setImage(const GLint internalformat, const GLsizei width, const GLsizei height, const GLenum format, const unsigned char* data) const;
    void generateMipmap() const;
    void setWrapping(const GLint s, const GLint t) const;
    void setFiltering(const GLint min, const GLint mag) const;
    GLuint getID() const;

protected:
    GLuint m_texture;
};

