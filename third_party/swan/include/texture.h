#ifndef _TEXTURE_H_
#define _TEXTURE_H_

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
    // constructors
    Texture();
    Texture(const string& filePath, const bool flipVertically = true);
    Texture(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically = true);

    // destructor
    virtual ~Texture();

    // bind and unbind
    void bind() const;
    void unbind() const;

    // image
    bool loadImage(const string& filePath, const bool flipVertically = true) const;
    bool loadImage(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically = true) const;
    void setImage(const GLint internalformat, const GLsizei width, const GLsizei height, const GLenum format, const unsigned char* data) const;

    // settings
    void generateMipmap() const;
    void setWrapping(const GLint s, const GLint t) const;
    void setFiltering(const GLint min, const GLint mag) const;

    // id
    GLuint getID() const;

protected:

protected:
    GLuint id_;
};

#endif // _TEXTURE_H_