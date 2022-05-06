#include <mesh.h>

// constructor
Mesh::Mesh(const int numVertices)
: _numVertices(numVertices)
{
    // create a vertex array object
    glGenVertexArrays(1, &_VAO);
}

// destructor
Mesh::~Mesh()
{
    // delete buffers
    for(GLuint VBO : _VBOs)    glDeleteBuffers(1, &VBO);
    if (_EBO != 0)             glDeleteBuffers(1, &_EBO);

    // delete the vertex array object
    if (_VAO != 0) glDeleteVertexArrays(1, &_VAO);
}

// setter
void Mesh::setElementIndices(const GLuint* data, const GLint numIndices)
{
    // set the number of vertices
    _numIndices = numIndices;

    // if an element buffer object is already created, delete it.
    if (_EBO != 0)
    {
        glDeleteBuffers(1, &_EBO);
        _EBO = 0;
    }

    // create an element buffer object
    glGenBuffers(1, &_EBO);

    // bind the element buffers and and copy the input to GPU
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * _numIndices, data, GL_STATIC_DRAW);

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
    glBindVertexArray(_VAO);

    // if we don't have any indicies,
    if (_numIndices == 0)
    {
        // draw primitives from array data
        glDrawArrays(mode, 0, _numVertices);
    }
    else
    {
        // bind the element array buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO);

        // draw primitives from element data
        glDrawElements(mode, _numIndices, GL_UNSIGNED_INT, 0);

        // unbind the element array buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    // unbind the vertex array object
    glBindVertexArray(0);
}