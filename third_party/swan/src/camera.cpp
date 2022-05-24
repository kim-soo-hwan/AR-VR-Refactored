// std
#include <iostream>
using namespace std;

// GLM
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_access.hpp>            // row, column
//#include <glm/gtx/matrix_interpolation.hpp>   // extractMatrixRotation
//#include <glm/gtx/matrix_decompose.hpp>       // decompose
#include <glm/gtx/euler_angles.hpp>             // yawPitchRoll

// swan
#include <camera.h>
#include <utility.h>

// constructor
Camera::Camera()
: projection_(1.f),
  view_(1.f)
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

// projection
void Camera::setProjectionMatrix(const float fx, const float fy, const float cx, const float cy)
{
    // typical 3x3 camera matrix but as 4x4 instead
    // K = [fx   0   cx   0]
    //     [ 0   fy  cy   0]
    //     [ 0   0    1   0]
    //     [ 0   0    0   1]
    // [caution] glm: column-major order
    projection_ = glm::mat4(1.f);
    projection_[0][0] = fx;
    projection_[1][1] = fy;
    projection_[2][0] = cx;
    projection_[2][1] = cy;
}

// view: transform
void Camera::rotate(const float angleInDegrees,                                 // rotation angle
                   const float axisX, const float axisY, const float axisZ)     // rotation axis
{
    // rotate: T' = T_r * T
    view_ = glm::rotate(view_, glm::radians(angleInDegrees), glm::vec3(axisX, axisY, axisZ));
}

void Camera::rotate(const float roll, const float pitch, const float yaw)       // euler angles
{
    // rotate: T' = T_r * T
    const glm::mat4 T_r = glm::yawPitchRoll(yaw, pitch, roll);
    view_ = T_r * view_;
}

void Camera::translate(const float tX, const float tY, const float tZ)          // translation vector
{
    // translate: T' = T_t * T
    view_ = glm::translate(view_, glm::vec3(tX, tY, tZ));
}

void Camera::transform(const float roll, const float pitch, const float yaw,    // euler angles
                       const float tX,   const float tY,    const float tZ)     // translation vector
{
    // Note: The order matters!

    // rotate: T'' = T_r * T' = T_r * T_t * T
    rotate(roll, pitch, yaw);

    // translate: T' = T_t * T
    translate(tX, tY, tZ);
}

void Camera::transform(const float angleInDegrees,                              // rotation angle
                       const float axisX, const float axisY, const float axisZ, // roation axis
                       const float tX, const float tY, const float tZ)          // translation vector
{
    // Note: The order matters!

    // rotate: T'' = T_r * T' = T_r * T_t * T
    rotate(angleInDegrees, axisX, axisY, axisZ);

    // translate: T' = T_t * T
    translate(tX, tY, tZ);
}

void Camera::setViewMatrix(const float r11, const float r12, const float r13,
                           const float r21, const float r22, const float r23,
                           const float r31, const float r32, const float r33,
                           const float tX,  const float tY,  const float tZ)
{
    // 1st column      2nd column         3rd column         4th column
    view_[0][0] = r11; view_[1][0] = r12; view_[2][0] = r13, view_[3][0] = tX;
    view_[0][1] = r21; view_[1][1] = r22; view_[2][1] = r23, view_[3][1] = tY;
    view_[0][2] = r31; view_[1][2] = r32; view_[2][2] = r33, view_[3][2] = tZ;
    view_[0][3] = 0.f; view_[1][3] = 0.f; view_[2][3] = 0.f, view_[3][3] = 1.f;
}

void Camera::resetViewMatrix()
{
    view_ = glm::mat4(1.f);
}

// view: loot at
void Camera::lookAt(const float eyeX,    const float eyeY,    const float eyeZ,     // camera position
                    const float centerX, const float centerY, const float centerZ,  // where to look
                    const float upX,     const float upY,     const float upZ)      // up vector
{
    view_ = glm::lookAt(glm::vec3(eyeX,    eyeY,    eyeZ),
                        glm::vec3(centerX, centerY, centerZ),
                        glm::vec3(upX,     upY,     upZ));
}

// move
// x: right, y: up, z: back
// inv(view): G_T_C = [G_R_C, G_t_GC]
//                    [  0  ,   1   ]
// view:      C_T_G = [C_R_G, C_t_CG] = [G_R_C^T, -G_R_C^T * G_t_GC] = [C_R_G, -C_R_G * G_t_GC] = [C_R_G, -C_t_GC]
//                    [  0  ,   1   ] = [  0   ,           1       ] = [  0  ,        1       ] = [  0  ,    1   ]
void Camera::moveInGlobalCoords(const glm::vec3 &G_t)
{
    // after movement
    // G_T_C' = [G_R_C, G_t_GC + G_t]
    //          [  0  ,      1      ]

    // update the view matrix
    // C_T_G' = [G_R_C^T, -G_R_C^T * (G_t_GC + G_t)]
    //          [  0    ,                  1       ]
    //        = [C_R_G,   -C_R_G * (G_t_GC + G_t)]
    //          [  0    ,                1       ]
    //        = [C_R_G,   -C_t_GC - C_R_G * G_t]
    //          [  0    ,              1       ]
    //        = [C_R_G,    C_t_CG - C_R_G * G_t]
    //          [  0    ,              1       ]
    //        = C_T_G + [- C_R_G * G_t]
    //                  [      0      ]

    // rotation matrix
    const glm::mat3 C_R_G(view_[0][0], view_[0][1], view_[0][2],    // 1st column
                          view_[1][0], view_[1][1], view_[1][2],    // 2st column
                          view_[2][0], view_[2][1], view_[2][2]);   // 3rd column

    // translate
    view_ = glm::translate(view_, - C_R_G * G_t);
}

void Camera::moveForward(const float displacement)
{
    moveBackward(-displacement);
}

void Camera::moveBackward(const float displacement)
{
    // z: back 
    // = 3rd column of G_R_C = G_R_C(c3)
    // = 3rd row    of C_R_G = C_R_G(r3)
    const glm::vec3 back(view_[0][2], view_[1][2], view_[2][2]);

    // move backward
    moveInGlobalCoords(displacement * back);
}

void Camera::moveLeft(const float displacement)
{
    moveRight(-displacement);
}

void Camera::moveRight(const float displacement)
{
    // x: right 
    // = 1st column of G_R_C = G_R_C(c1)
    // = 1st row    of C_R_G = C_R_G(r1)
    const glm::vec3 right(view_[0][0], view_[1][0], view_[2][0]);

    // move right
    moveInGlobalCoords(displacement * right);
}

void Camera::moveUp(const float displacement)
{
    // y: up 
    // = 2nd column of G_R_C = G_R_C(c2)
    // = 2nd row    of C_R_G = C_R_G(r2)
    const glm::vec3 up(view_[0][1], view_[1][1], view_[2][1]);

    // move up
    moveInGlobalCoords(displacement * up);
}

void Camera::moveDown(const float displacement)
{
    moveUp(-displacement);
}

// inv(view): G_T_C = [G_R_C, G_t_GC]
//                    [  0  ,   1   ]
// view:      C_T_G = [C_R_G, C_t_CG] = [G_R_C^T, -G_R_C^T * G_t_GC] = [C_R_G, -C_R_G * G_t_GC] = [C_R_G, -C_t_GC]
//                    [  0  ,   1   ] = [  0   ,           1       ] = [  0  ,        1       ] = [  0  ,    1   ]
void Camera::pan(const float angleInDegrees)
{
    // y: up
    // = 2nd column of G_R_C = G_R_C(c2)
    // = 2nd row    of C_R_G = C_R_G(r2)
    rotate(angleInDegrees, view_[0][1], view_[1][1], view_[2][1]);
}

void Camera::tilt(const float angleInDegrees)
{
    // x: right
    // = 1st column of G_R_C = G_R_C(c1)
    // = 1st row    of C_R_G = C_R_G(r1)
    rotate(angleInDegrees, view_[0][0], view_[1][0], view_[2][0]);
}

// getter
glm::mat4 Camera::getProjectionMatrix() const
{
    // F_T_C
    return projection_;
}

glm::mat4 Camera::getViewMatrix() const
{
    // C_T_G
    return view_;
}

glm::mat4 Camera::getViewProjectionMatrix() const
{
    // F_T_G = F_T_C * C_T_G
    return projection_ * view_;
}