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

    // vertex attributes
    template<typename T, typename T1, typename... Tn>
    void setVertexAttributes(const T* data, const T1 numComponentsPerVertex1, const Tn... numComponentsPerVertexN)
    {
        // sum up the total number of components for each vertex
        const GLint numComponentsPerVertex = sumNumComponentsPerVertex(numComponentsPerVertex1, numComponentsPerVertexN...);

        // bind the vertex array object
        glBindVertexArray(VAO_);

        // create a vertex buffer object
        GLuint VBO;
        glGenBuffers(1, &VBO);
        VBOs_.push_back(VBO);

        // bind it to the array buffer target on GPU
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        // copy the vertex array on host to the array buffer on device (GPU)
        glBufferData(GL_ARRAY_BUFFER, sizeof(T) * numComponentsPerVertex * numVertices_, data, GL_STATIC_DRAW);

        // set the vertex attribute pointer    
        setVertexAttributePointer(data, numComponentsPerVertex, 0, numComponentsPerVertex1, numComponentsPerVertexN...);
    }

    // element index
    void setElementIndices(const GLuint* data, const GLint numIndices);

    // point size
    void setPointSize(const GLfloat size) const;

    // shader program
    void setShaderProgram(const shared_ptr<ShaderProgram> &shaderProgram);

    // set draw mode
    void setDrawMode(const GLenum mode);

    // draw
    void draw() const;

protected:
    // sum up the numbers of components per vertex using the parameter pack
    // C++11 Parameter Pack, C++17 Unary Fold
    template<typename... T>
    GLint sumNumComponentsPerVertex(const T... numComponentsPerVertex) const
    {
        return (... + numComponentsPerVertex);
    }

    // set vertex attribute pointer using the parameter pack (C++11)
    template<typename T>
    void setVertexAttributePointer(const T* data, const GLint numComponentsPerVertex, const GLint cummulatedNumComponentsPerVertex) { }

    template<typename T, typename T1, typename... Tn>
    void setVertexAttributePointer(const T* data, const GLint numComponentsPerVertex, const GLint cummulatedNumComponentsPerVertex, const T1 numComponentsPerVertex1, const Tn... numComponentsPerVertexN)
    {
        // set the vertex attribute pointer
        glVertexAttribPointer(numVertexAttributes_, numComponentsPerVertex1, GL_FLOAT, GL_FALSE, numComponentsPerVertex * sizeof(T), reinterpret_cast<void*>(cummulatedNumComponentsPerVertex * sizeof(T)));
        glEnableVertexAttribArray(numVertexAttributes_);

        // next
        numVertexAttributes_++;
        setVertexAttributePointer(data, numComponentsPerVertex, cummulatedNumComponentsPerVertex + numComponentsPerVertex1, numComponentsPerVertexN...);
    }

protected:
    // draw mode
    GLenum drawMode_;

    // vertex array object
    GLuint VAO_ = 0;               // vertex array object

    // buffers
    vector<GLuint> VBOs_;          // vertex buffer object
    GLuint EBO_ = 0;               // element buffer object

    // counts
    GLint  numVertices_ = 0;
    GLint  numIndices_  = 0;
    GLuint numVertexAttributes_ = 0;

    // shader program
    shared_ptr<ShaderProgram> shaderProgram_;
};

#endif // __MESH_H__