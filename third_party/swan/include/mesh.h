#ifndef __MESH_H__
#define __MESH_H__

#include <vector>   // vector
#include <memory>   // shared_prt, make_shared
using namespace std;

// GLAD, GLFW
// GLAD must be included before GLFW
// It includes the required OpenGL headers like GL/gl.h
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// swan
#include <shader.h>

class Mesh
{
public:
    // constructor
    Mesh(const int numVertices);

    // destructor
    virtual ~Mesh();

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
        _VBOs.push_back(VBO);

        // bind it to the array buffer target on GPU
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        // copy the vertex array on host to the array buffer on device (GPU)
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * numComponentsPerVertex * _numVertices, data, GL_STATIC_DRAW);

        // set the vertex attribute pointer    
        setVertexAttributePointer(data, numComponentsPerVertex, 0, numComponentsPerVertex1, numComponentsPerVertexN...);
    }

    void setElementIndices(const GLuint* data, const GLint numIndices);
    void setPointSize(const GLfloat size);
    void setShaderProgram(const shared_ptr<ShaderProgram> &shaderProgram);

    // draw
    void draw(GLenum mode = GL_TRIANGLES);

protected:
    // sum up the numbers of components per vertex using the parameter pack
    // C++11 Parameter Pack, C++17 Unary Fold
    template<typename... T>
    GLint sumNumComponentsPerVertex(const T... numComponentsPerVertex)
    {
        return (... + numComponentsPerVertex);
    }

    // set vertex attribute pointer using the parameter pack (C++11)
    void setVertexAttributePointer(const GLfloat* data, const GLint numComponentsPerVertex, const GLint cummulatedNumComponentsPerVertex) { }

    template<typename T1, typename... Tn>
    void setVertexAttributePointer(const GLfloat* data, const GLint numComponentsPerVertex, const GLint cummulatedNumComponentsPerVertex, const T1 numComponentsPerVertex1, const Tn... numComponentsPerVertexN)
    {
        // set the vertex attribute pointer
        glVertexAttribPointer(_numVertexAttributes, numComponentsPerVertex1, GL_FLOAT, GL_FALSE, numComponentsPerVertex * sizeof(GLfloat), reinterpret_cast<void*>(cummulatedNumComponentsPerVertex * sizeof(GLfloat)));
        glEnableVertexAttribArray(_numVertexAttributes);

        // next
        _numVertexAttributes++;
        setVertexAttributePointer(data, numComponentsPerVertex, cummulatedNumComponentsPerVertex + numComponentsPerVertex1, numComponentsPerVertexN...);
    }

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

    // shader program
    shared_ptr<ShaderProgram> _shaderProgram;
};

#endif // __MESH_H__