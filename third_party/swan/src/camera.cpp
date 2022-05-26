// std
#include <iostream>
using namespace std;

// GLM
#include <glm/gtc/matrix_transform.hpp>
//#include <glm/gtc/matrix_access.hpp>            // row, column
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

Camera::Camera(const float fovy,    // vertical field of view in radian
               const float aspect,  // aspect ratio of field of view
               const float near,    // near plane
               const float far)     // far plane
: projection_(glm::perspective(fovy, aspect, near, far)),
  view_(1.f)
{
}

// destructor
Camera::~Camera()
{
}

// projection
void Camera::setProjectionMatrix(const float fx, const float fy,    // focal length
                                 const float cx, const float cy,    // optical center
                                 const float zn, const float zf,    // near/far plane
                                 const float w,  const float h)     // width/height of the image
{
    // ref) https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/
    // ref) https://fruty.io/2019/08/29/augmented-reality-with-opencv-and-opengl-the-tricky-projection-matrix/

    // NDC_T_C = [2*fx/w,      0,      (w-2*cx)/w,            0       ]
    //           [   0,    -2*fy/h,    (h-2*cy)/h,            0       ]
    //           [   0,        0,   -(zf+zn)/(zf-zn), -2*zf*zn/(zf-zn)]
    //           [   0,        0,          -1,                0       ]

    // [caution] glm: column-major order
    projection_ = glm::mat4(); // zero matrix

    projection_[0][0] =  2.f * fx / w;
    projection_[1][1] = -2.f * fy / h;

    projection_[2][0] = (w - 2.f * cx) / w;
    projection_[2][1] = (h - 2.f * cy) / h;
    projection_[2][2] = -(zf + zn) / (zf - zn);
    projection_[2][3] = -1.f;

    projection_[3][2] = -2.f * zf * zn / (zf - zn);
}

// view: transform
void Camera::rotate(const float angle,                                          // rotation angle in radian
                    const float axisX, const float axisY, const float axisZ)     // rotation axis
{
    // [caution] 
    // glm::rotate multiplies a new 4x4 transformation matrix 
    // created from the rotation angle-axis "to the right side" 
    // of the passed 4x4 transformation matrix
    // 
    // In other words, 
    // 
    // T = glm::rotate(T, r)
    //
    // is equivalent to 
    //
    // glm::mat4 T_new(1.f);
    // T_new = glm::rotate(T_new, r);
    // T = T * T_new; 
    //
    // here, T != T_new * T
    //
    // In other words,
    //
    // T' = [R  t][R' 0] = [RR' t]
    //      [0  1][0  1] = [0   1]
    //
    // Therefore, it means we apply the new rotation first and the existing rotation later,
    // having the translation fixed.

    view_ = glm::rotate(view_, angle, glm::vec3(axisX, axisY, axisZ));

    // [caution] glm::rotate: the angle is expressed in radians
    // as mentioned in the glm code (glm/gtx/transform.hpp),
    // not in degrees as mentioned in the glm web site (https://glm.g-truc.net/0.9.9/api/a00247.html)
}

void Camera::rotate(const float roll, const float pitch, const float yaw)       // euler angles
{
    // [caution] 
    // This only applies to the transformation matrix "to the right side"
    // of the existing view matrix just like glm::rotate.
    // T(view)'  = T(view) * T(R)
    // T(view)' != T(R) * T(view)
    const glm::mat4 T = glm::yawPitchRoll(yaw, pitch, roll);
    view_ =  view_ * T;
}

void Camera::translate(const float tX, const float tY, const float tZ)          // translation vector
{
    // [caution] 
    // glm::translate multiplies a new 4x4 transformation matrix 
    // created from the translation vector "to the right side" 
    // of the passed 4x4 transformation matrix
    // 
    // In other words, 
    // 
    // T = glm::translate(T, v)
    //
    // is equivalent to 
    //
    // glm::mat4 T_new(1.f);
    // T_new = glm::translate(T_new, v);
    // T = T * T_new; 
    //
    // here, T != T_new * T
    //
    // In other words,
    //
    // T'  = [R  t][I  v] = [R  Rv+t]
    //       [0  1][0  1] = [0    1 ]
    // T' != [I  v][R  t] = [R  t+v]
    //       [0  1][0  1] = [0   1 ]
    view_ = glm::translate(view_, glm::vec3(tX, tY, tZ));
}

void Camera::transform(const float angle,                                       // rotation angle in radian
                       const float axisX, const float axisY, const float axisZ, // roation axis
                       const float tX, const float tY, const float tZ)          // translation vector
{
    // [caution] 
    // The order matters!
    // Since rotate and translate multiply T on the right side,
    // we need to call translate first and then rotate later.
    // i.e., T' = T * T(t) * T(R)
    //
    // Then, when we transform a point,
    // we can rotate it first and then translate later.
    // p' = T * T(t) * T(R) p

    translate(tX, tY, tZ);
    rotate(angle, axisX, axisY, axisZ);
}

void Camera::transform(const float roll, const float pitch, const float yaw,    // euler angles
                       const float tX,   const float tY,    const float tZ)     // translation vector
{
    // [caution] 
    // The order matters!
    // Since rotate and translate multiply T on the right side,
    // we need to call translate first and then rotate later.
    // i.e., T' = T * T(t) * T(R)
    //
    // Then, when we transform a point,
    // we can rotate it first and then translate later.
    // p' = T * T(t) * T(R) p

    translate(tX, tY, tZ);
    rotate(roll, pitch, yaw);
}

void Camera::setViewMatrix(const float r11, const float r12, const float r13, const float tX,   // 1st row
                           const float r21, const float r22, const float r23, const float tY,   // 2nd row
                           const float r31, const float r32, const float r33, const float tZ)   // 3rd row
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

    // translation vector
    const glm::vec3 v = C_R_G * G_t;

    // 4th column
    view_[3][0] -= v.x;
    view_[3][1] -= v.y;
    view_[3][2] -= v.z;
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
void Camera::pan(const float angle)
{
    // y: up
    // = 2nd column of G_R_C = G_R_C(c2)
    // = 2nd row    of C_R_G = C_R_G(r2)
    rotate(angle, view_[0][1], view_[1][1], view_[2][1]);
}

void Camera::tilt(const float angle)
{
    // x: right
    // = 1st column of G_R_C = G_R_C(c1)
    // = 1st row    of C_R_G = C_R_G(r1)
    rotate(angle, view_[0][0], view_[1][0], view_[2][0]);
}

// getter
glm::mat4 Camera::getProjectionMatrix() const
{
    // NDC_T_C
    return projection_;
}

glm::mat4 Camera::getViewMatrix() const
{
    // C_T_G
    return view_;
}

glm::mat4 Camera::getViewProjectionMatrix() const
{
    // F_T_G = NDC_T_C * C_T_G
    return projection_ * view_;
}