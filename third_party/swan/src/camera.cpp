// GLM
#include <glm/gtc/matrix_transform.hpp> // perspective, lookAt

// swan
#include <camera.h>
#include <utility.h>

// constructor
Camera::Camera()
: projection_(1.f),
  view_(T_)
{
}

Camera::Camera(const float fovy,    // vertical field of view in radian
               const float aspect,  // aspect ratio of field of view
               const float near,    // near plane
               const float far)     // far plane
: projection_(glm::perspective(fovy, aspect, near, far)),
  view_(T_)
{
}

// destructor
Camera::~Camera()
{
}

// view
void Camera::setViewMatrixForMobilTech(const float yaw, const float pitch, const float roll,     // Euler angles
                                       const float tX,  const float tY,    const float tZ)       // translation vector
{
    // reset
    resetTransformationMatrix();

    // C_p = R * Rx(roll) * Ry(pitch) * Rz(yaw) * (L_p - t)
    //
    // [C_p] = [R  0][Rx(roll)  0][Ry(pitch)   0][Rz(yaw)  0][L_p - t]
    // [ 1 ] = [0  1][   0      1][   0        1][   0     1][   1   ]
    //
    // [C_p] = [R  0][Rx(roll)  0][Ry(pitch)   0][Rz(yaw)  0][I  -t][L_p]
    // [ 1 ] = [0  1][   0      1][   0        1][   0     1][0   1][ 1 ]
    //
    // C_p = C_T_L * L_P

    // transform
    translate_L(-tX, -tY, -tZ);
    rotateAboutZ_L(yaw);
    rotateAboutY_L(pitch);
    rotateAboutX_L(roll);

    // R
    glm::mat4 Tr(0.f);
    Tr[0][2] = -1.f;   Tr[1][0] =  1.f;   Tr[2][1] = -1.f;   Tr[3][3] = 1.f;

    // T' = T(R) * T
    T_ = Tr * T_;
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

    // rotate beforehand = rotate w.r.t the local coordinates
    rotate_R(angle, view_[0][1], view_[1][1], view_[2][1]);
}

void Camera::tilt(const float angle)
{
    // x: right
    // = 1st column of G_R_C = G_R_C(c1)
    // = 1st row    of C_R_G = C_R_G(r1)

    // rotate beforehand = rotate w.r.t the local coordinates
    rotate_R(angle, view_[0][0], view_[1][0], view_[2][0]);
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