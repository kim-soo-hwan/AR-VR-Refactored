#include <mesh.h>

// constructor
Mesh::Mesh(const int numVertices)
: numVertices_(numVertices),
  drawMode_(GL_TRIANGLES)
{
    // create a vertex array object
    glGenVertexArrays(1, &VAO_);
}

// destructor
Mesh::~Mesh()
{
    // delete buffers
    for(GLuint VBO : VBOs_)
    {
        if (VBO != 0)   glDeleteBuffers(1, &VBO);
    }
    if (EBO_ != 0)      glDeleteBuffers(1, &EBO_);

    // delete the vertex array object
    if (VAO_ != 0) glDeleteVertexArrays(1, &VAO_);
}

// setter
void Mesh::setElementIndices(const GLuint* data, const GLint numIndices)
{
    // set the number of vertices
    numIndices_ = numIndices;

    // if an element buffer object is already created, delete it.
    if (EBO_ != 0)
    {
        glDeleteBuffers(1, &EBO_);
        EBO_ = 0;
    }

    // create an element buffer object
    glGenBuffers(1, &EBO_);

    // bind the element buffers and and copy the input to GPU
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * numIndices_, data, GL_STATIC_DRAW);
}

void Mesh::setPointSize(const GLfloat size) const
{
    // initial value is 1
    // GL_INVALID_VALUE is generated if size is less than or equal to 0. 
    glPointSize(size);
}

void Mesh::setShaderProgram(const shared_ptr<ShaderProgram> &shaderProgram)
{
    shaderProgram_ = shaderProgram;
}

// set draw mode
void Mesh::setDrawMode(GLenum mode)
{
    drawMode_ = mode;
}

// draw
void Mesh::draw() const
{
    // shader program
    if(shaderProgram_) shaderProgram_->use();
    
    // bind the vertex array object
    glBindVertexArray(VAO_);

    // if we don't have any indicies,
    if (numIndices_ == 0)
    {
        // draw primitives from array data
        glDrawArrays(drawMode_, 0, numVertices_);
    }
    else
    {
        // bind the element array buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);

        // draw primitives from element data
        glDrawElements(drawMode_, numIndices_, GL_UNSIGNED_INT, 0);

        // unbind the element array buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    // unbind the vertex array object
    glBindVertexArray(0);
}