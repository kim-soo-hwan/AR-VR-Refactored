#include "mesh.hpp"

// constructor
Mesh::Mesh(const int numVertices)
    : m_numVertices(numVertices)
{
    // create a vertex array object
    glGenVertexArrays(1, &m_VAO);
}

// destructor
Mesh::~Mesh()
{
    // delete buffers
    if (m_VBO != 0)   glDeleteBuffers(1, &m_VBO);
    if (m_EBO != 0)   glDeleteBuffers(1, &m_EBO);

    // delete the vertex array object
    if (m_VAO != 0) glDeleteVertexArrays(1, &m_VAO);
}

// setter
void Mesh::setVertexPositions(const GLint numComponentsPerVertex, const GLfloat* data)
{
    // bind the vertex array object
    glBindVertexArray(m_VAO);

    // if a vertex buffer object is already created, delete it.
    if (m_VBO != 0)
    {
        glDeleteBuffers(1, &m_VBO);
        m_VBO = 0;
    }

    // create a vertex buffer object
    glGenBuffers(1, &m_VBO);

    // bind it to the array buffer target on GPU
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

    // copy the vertex array on host to the array buffer on device (GPU)
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * numComponentsPerVertex * m_numVertices, data, GL_STATIC_DRAW);

    // set the vertex attribute pointer
    glVertexAttribPointer(0, numComponentsPerVertex, GL_FLOAT, GL_FALSE, numComponentsPerVertex * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);

    // unbind the vertex buffer object
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // unbind the vertex array object
    glBindVertexArray(0);
}

void Mesh::setElementIndices(const GLint numIndices, const GLuint* data)
{
    // set the number of vertices
    m_numIndices = numIndices;

    // if an element buffer object is already created, delete it.
    if (m_EBO != 0)
    {
        glDeleteBuffers(1, &m_EBO);
        m_EBO = 0;
    }

    // create an element buffer object
    glGenBuffers(1, &m_EBO);

    // bind the element buffers and and copy the input to GPU
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_numIndices, data, GL_STATIC_DRAW);

    // unbind the element buffer object
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::setPointSize(const GLfloat size)
{
    // initial value is 1
    // GL_INVALID_VALUE is generated if size is less than or equal to 0. 
    glPointSize(size);
}

// draw
void Mesh::draw(GLenum mode)
{
    // bind the vertex array object
    glBindVertexArray(m_VAO);

    // if we don't have any indicies,
    if (m_numIndices == 0)
    {
        // draw primitives from array data
        glDrawArrays(mode, 0, m_numVertices);
    }
    else
    {
        // bind the element array buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);

        // draw primitives from element data
        glDrawElements(mode, m_numIndices, GL_UNSIGNED_INT, 0);

        // unbind the element array buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    // unbind the vertex array object
    glBindVertexArray(0);
}