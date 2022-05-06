// GLM
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <model.h>


// constructor
Model::Model()
: model_(1.f)
{
}

// destructor
Model::~Model()
{
}

// transform
void Model::rotate(const float angleInDegrees,                                  // rotation angle
                   const float axisX, const float axisY, const float axisZ)     // rotation axis
{
    // rotate: M' = T_r * M
    model_ = glm::rotate(model_, glm::radians(angleInDegrees), glm::vec3(axisX, axisY, axisZ));
}

void Model::translate(const float tX, const float tY, const float tZ)          // translation vector
{
    // translate: M' = T_t * M
    model_ = glm::translate(model_, glm::vec3(tX, tY, tZ));
}

void Model::transform(const float angleInDegrees,                              // rotation angle
                      const float axisX, const float axisY, const float axisZ, // roation axis
                      const float tX, const float tY, const float tZ)          // translation vector
{
    // Note: The order doesn't matter!

    // translate: M' = T_t * M
    translate(tX, tY, tZ);

    // rotate: M'' = T_r * M' = T_r * T_t * M
    rotate(angleInDegrees, axisX, axisY, axisZ);
}

void Model::resetModelMatrix()
{
    model_ = glm::mat4(1.f);
}

// getter
glm::mat4 Model::getModelMatrix() const
{
    return model_;
}