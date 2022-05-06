#ifndef __MESH_HPP__
#define __MESH_HPP__

#include <vector>
using namespace std;

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
    void addVertexAttribute(const GLint numComponentsPerVertex, const GLfloat* data);
    void addVertexAttribute(const GLint numComponentsPerVertex1, const GLint numComponentsPerVertex2, const GLfloat* data);
    void setElementIndices(const GLint numIndices, const GLuint* data);
    void setPointSize(const GLfloat size);

    // draw
    void draw(GLenum mode = GL_TRIANGLES);

protected:
    // vertex array object
    GLuint m_VAO = 0;               // vertex array object

    // buffers
    vector<GLuint> m_VBOs;          // vertex buffer object
    GLuint m_EBO = 0;               // element buffer object

    // counts
    GLint m_numVertices = 0;
    GLint m_numIndices  = 0;
};

#endif // __MESH_HPP__