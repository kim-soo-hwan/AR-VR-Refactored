#ifndef __MESH_H__
#define __MESH_H__

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
    template<typename T1, typename... Tn>
    void setVertexAttributes(const GLfloat* data, const T1 numComponentsPerVertex1, const Tn... numComponentsPerVertexN)
    {
        // sum up the total number of components for each vertex
        const GLint numComponentsPerVertex = sumNumComponentsPerVertex(numComponentsPerVertex1, numComponentsPerVertexN...);

        // bind the vertex array object
        glBindVertexArray(_VAO);

        // create a vertex buffer object
        GLuint VBO;
        glGenBuffers(1, &VBO);

        // bind it to the array buffer target on GPU
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        // copy the vertex array on host to the array buffer on device (GPU)
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * numComponentsPerVertex * _numVertices, data, GL_STATIC_DRAW);

        // set the vertex attribute pointer    
        glVertexAttribPointer(_numVertexAttributes, numComponentsPerVertex1, GL_FLOAT, GL_FALSE, numComponentsPerVertex * sizeof(GLfloat), (void*)0);
        glEnableVertexAttribArray(_numVertexAttributes);

        // next
        _numVertexAttributes++;
        setVertexAttributePointer(data, numComponentsPerVertex, numComponentsPerVertex1, numComponentsPerVertexN...);
    }

    void setElementIndices(const GLuint* data, const GLint numIndices);
    void setPointSize(const GLfloat size);

    // draw
    void draw(GLenum mode = GL_TRIANGLES);

protected:
    template<typename T1, typename... Tn>
    GLint sumNumComponentsPerVertex(const T1 numComponentsPerVertex1, const Tn... numComponentsPerVertexN)
    {
        return numComponentsPerVertex1 + sumNumComponentsPerVertex(numComponentsPerVertexN...);
    }
    GLint sumNumComponentsPerVertex() { return 0; }

    template<typename T1, typename... Tn>
    void setVertexAttributePointer(const GLfloat* data, const GLint numComponentsPerVertex, const GLint cummulatedNumComponentsPerVertex, const T1 numComponentsPerVertex1, const Tn... numComponentsPerVertexN)
    {
        // set the vertex attribute pointer
        glVertexAttribPointer(_numVertexAttributes, numComponentsPerVertex1, GL_FLOAT, GL_FALSE, numComponentsPerVertex * sizeof(GLfloat), reinterpret_cast<void*>(cummulatedNumComponentsPerVertex));
        glEnableVertexAttribArray(_numVertexAttributes);

        // next
        _numVertexAttributes++;
        setVertexAttributePointer(data, numComponentsPerVertex, cummulatedNumComponentsPerVertex + numComponentsPerVertex1, numComponentsPerVertexN...);
    }

    void setVertexAttributePointer(const GLfloat* data, const GLint numComponentsPerVertex, const GLint cummulatedNumComponentsPerVertex) { }

protected:
    // vertex array object
    GLuint _VAO = 0;               // vertex array object

    // buffers
    vector<GLuint> _VBOs;          // vertex buffer object
    GLuint _EBO = 0;               // element buffer object

    // counts
    GLint  _numVertices = 0;
    GLint  _numIndices  = 0;
    GLuint _numVertexAttributes = 0;
};

#endif // __MESH_H__