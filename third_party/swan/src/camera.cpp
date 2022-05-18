// GLM
#include <glm/gtc/matrix_transform.hpp>

// swan
#include <camera.h>


// constructor
Camera::Camera()
: projection_(1.f),
  view(1.f)
{

}

Camera::Camera(const float fovyInDegrees, const float aspect, const float near, const float far)
: projection_(glm::perspective(glm::radians(fovyInDegrees), aspect, near, far)),
  view_(1.f)
{
}

// destructor
Camera::~Camera()
{
}

// transform
void Camera::rotate(const float angleInDegrees,                                 // rotation angle
                   const float axisX, const float axisY, const float axisZ)     // rotation axis
{
    // rotate: T' = T_r * T
    view_ = glm::rotate(view_, glm::radians(angleInDegrees), glm::vec3(axisX, axisY, axisZ));
}

void Camera::translate(const float tX, const float tY, const float tZ)          // translation vector
{
    // translate: T' = T_t * T
    view_ = glm::translate(view_, glm::vec3(tX, tY, tZ));
}

void Camera::transform(const float angleInDegrees,                              // rotation angle
                       const float axisX, const float axisY, const float axisZ, // roation axis
                       const float tX, const float tY, const float tZ)          // translation vector
{
    // Note: The order doesn't matter!

    // translate: T' = T_t * T
    translate(tX, tY, tZ);

    // rotate: T'' = T_r * T' = T_r * T_t * T
    rotate(angleInDegrees, axisX, axisY, axisZ);
}

void Camera::resetViewMatrix()
{
    view_ = glm::mat4(1.f);
}

// getter
glm::mat4 Model::getProjectionMatrix() const
{
    // F_T_C
    return projection_;
}

glm::mat4 Model::getViewMatrix() const
{
    // C_T_G
    return view_;
}

glm::mat4 Model::getViewProjectionMatrix() const
{
    // F_T_G = F_T_C * C_T_G
    return projection_ * view_;
}