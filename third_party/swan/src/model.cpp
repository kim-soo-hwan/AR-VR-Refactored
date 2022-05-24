// GLM
#include <glm/gtc/matrix_transform.hpp>

#include <model.h>


// constructor
Model::Model(const int numVertices)
: Mesh(numVertices),
  model_(1.f)
{
}

// destructor
Model::~Model()
{
}

// transform
void Model::scale(const float scaleX, const float scaleY, const float scaleZ)   // scale
{
    // scale: T' = T_s * T
    model_ = glm::scale(model_, glm::vec3(scaleX, scaleY, scaleZ));
}

void Model::rotate(const float angleInDegrees,                                  // rotation angle
                   const float axisX, const float axisY, const float axisZ)     // rotation axis
{
    // rotate: T' = T_r * T
    model_ = glm::rotate(model_, glm::radians(angleInDegrees), glm::vec3(axisX, axisY, axisZ));
}

void Model::translate(const float tX, const float tY, const float tZ)          // translation vector
{
    // translate: T' = T_t * T
    model_ = glm::translate(model_, glm::vec3(tX, tY, tZ));
}

void Model::transform(const float angleInDegrees,                              // rotation angle
                      const float axisX, const float axisY, const float axisZ, // roation axis
                      const float tX, const float tY, const float tZ)          // translation vector
{
    // Note: The order doesn't matter!

    // translate: T' = T_t * T
    translate(tX, tY, tZ);

    // rotate: T'' = T_r * T' = T_r * T_t * T
    rotate(angleInDegrees, axisX, axisY, axisZ);
}

void Model::resetModelMatrix()
{
    model_ = glm::mat4(1.f);
}

void Model::setModelViewProjectionMatrixName(const string& name)
{
    modelViewProjectionMatrixName_ = name;
}

// getter
glm::mat4 Model::getModelMatrix() const
{
    return model_;
}

// draw
void Model::draw(const glm::mat4 &viewProjectionMatrix) const
{
    // model-view-projection matrix: F_T_O = F_T_C * C_T_G * G_T_O
    glm::mat4 modelViewProjectionMatrix = viewProjectionMatrix * model_;

    // set the transform variable
    if (!modelViewProjectionMatrixName_.empty() && shaderProgram_) 
        shaderProgram_->set(modelViewProjectionMatrixName_, modelViewProjectionMatrix);

    // draw mesh
    Mesh::draw();
}