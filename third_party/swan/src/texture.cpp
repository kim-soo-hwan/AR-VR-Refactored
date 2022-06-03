#include <iostream>
#include <filesystem> // C++17 filesystem::path::extension
using namespace std;

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <texture.h>

Texture::Texture()
{
    glGenTextures(1, &id_);

    // default setting: GL_REPEAT, GL_NEAREST
    //setWrapping(GL_REPEAT, GL_REPEAT);
    //setFiltering(GL_LINEAR, GL_LINEAR);
}

Texture::Texture(const string& filePath, const bool flipVertically)
: Texture()
{
    loadImage(filePath, flipVertically);
}

Texture::Texture(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically)
: Texture()
{
    loadImage(filePath, internalFormat, format, flipVertically);
}

Texture::~Texture()
{
    glDeleteTextures(1, &id_);
}

void Texture::bind() const
{
    glBindTexture(GL_TEXTURE_2D, id_);
}

void Texture::unbind() const
{
    glBindTexture(GL_TEXTURE_2D, 0);
}

bool Texture::loadImage(const string& filePath, const bool flipVertically) const
{
    // file extension
    const string extension = filesystem::path(filePath).extension().string();

    // for each file type
    if (extension.compare(".jpg") == 0) return loadImage(filePath, GL_RGB,   GL_RGB,  flipVertically);
    if (extension.compare(".png") == 0) return loadImage(filePath, GL_RGBA,  GL_RGBA, flipVertically);

    return false;
}

bool Texture::loadImage(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically) const
{
    // flip vertically
    stbi_set_flip_vertically_on_load(flipVertically);

    // load the image
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

    // mipmap
    generateMipmap();
    
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
    return id_;
}