#include <iostream>
using namespace std;

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "texture.hpp"

Texture::Texture()
{
    glGenTextures(1, &m_texture);

    // default setting
    setWrapping(GL_REPEAT, GL_REPEAT);
    setFiltering(GL_LINEAR, GL_LINEAR);
}

Texture::Texture(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically)
    : Texture()
{
    loadImage(filePath, internalFormat, format, flipVertically);
    generateMipmap();
}

Texture::~Texture()
{
    glDeleteTextures(1, &m_texture);
}

void Texture::bind() const
{
    glBindTexture(GL_TEXTURE_2D, m_texture);
}

void Texture::bind(const GLuint textureUnit) const
{
    glActiveTexture(GL_TEXTURE0 + textureUnit);
    bind();
}

void Texture::unbind() const
{
    glBindTexture(GL_TEXTURE_2D, 0);
}

bool Texture::loadImage(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically)
{
    stbi_set_flip_vertically_on_load(flipVertically);

    int width, height, nChannels;
    unsigned char* data = stbi_load(filePath.c_str(), &width, &height, &nChannels, 0);

    if (data == nullptr)
    {
        cerr << "Error: Failed to load texture at " << filePath << endl;
        return false;
    }

    // specify the texture image data
    bind();
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    unbind();

    // free data
    stbi_image_free(data);

    return true;
}

void Texture::setImage(const GLint internalformat, const GLsizei width, const GLsizei height, const GLenum format, const unsigned char* data) const
{
    bind();

    // specify the texture image data
    glTexImage2D(GL_TEXTURE_2D, 0, internalformat, width, height, 0, format, GL_UNSIGNED_BYTE, data);

    unbind();
}

void Texture::generateMipmap() const
{
    bind();

    glGenerateMipmap(GL_TEXTURE_2D);
    setFiltering(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);

    unbind();
}

void Texture::setWrapping(const GLint s, const GLint t) const
{
    bind();

    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, s);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, t);

    unbind();
}

void Texture::setFiltering(const GLint min, const GLint mag) const
{
    bind();

    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag);

    unbind();
}

GLuint Texture::getID() const
{
    return m_texture;
}