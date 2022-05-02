#pragma once

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Mesh
{
public:
    // constructor
    Mesh(const int numVertices);

    // destructor
    ~Mesh();

    // setter
    void setVertexPositions(const GLint numComponentsPerVertex, const GLfloat* data);
    void setElementIndices(const GLint numIndices, const GLuint* data);
    void setPointSize(const GLfloat size);

    // draw
    void draw(GLenum mode = GL_TRIANGLES);

protected:
    // vertex array object
    GLuint m_VAO = 0;       // vertex array object

    // buffers
    GLuint m_VBO = 0;       // vertex buffer object
    GLuint m_EBO = 0;       // element buffer object

    // counts
    GLint m_numVertices = 0;
    GLint m_numIndices = 0;
};