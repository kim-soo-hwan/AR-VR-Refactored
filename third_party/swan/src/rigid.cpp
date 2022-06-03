// std
#include <cmath> // sin, cos
using namespace std;

// GLM
#include <glm/gtc/matrix_transform.hpp> // scale, rotate, translate
#include <glm/gtx/euler_angles.hpp>     // yawPitchRoll

// swan
#include <rigid.h>

// constructor
RigidBody::RigidBody()
: T_(1.f)
{
}

// destructor
RigidBody::~RigidBody()
{
}

// operator
ostream& operator<< (ostream& os, const RigidBody &T)
{
    for(int row = 0; row < 4; row++)
    {
        for(int col = 0; col < 4; col++)
        {
            os << T.T_[col][row];
            if(col != 3) os << ", ";
        }
        os << endl;
    }
    os << endl;
    return os;
}

// setter
void RigidBody::setTransformationMatrix(const glm::mat4 T)
{
    T_ = T;
}

void RigidBody::setTransformationMatrix(const float r11, const float r12, const float r13, const float tX,   // 1st row
                                        const float r21, const float r22, const float r23, const float tY,   // 2nd row
                                        const float r31, const float r32, const float r33, const float tZ)   // 3rd row
{
    // 1st column     2nd column        3rd column        4th column
    T_[0][0] = r11;   T_[1][0] = r12;   T_[2][0] = r13,   T_[3][0] = tX;
    T_[0][1] = r21;   T_[1][1] = r22;   T_[2][1] = r23,   T_[3][1] = tY;
    T_[0][2] = r31;   T_[1][2] = r32;   T_[2][2] = r33,   T_[3][2] = tZ;
    T_[0][3] = 0.f;   T_[1][3] = 0.f;   T_[2][3] = 0.f,   T_[3][3] = 1.f;
}

void RigidBody::setTransformationMatrixforMobileTech(const float yaw, const float pitch, const float roll,     // Euler angles
                                                     const float tX,  const float tY,    const float tZ)       // translation vector
{
    // reset
    resetTransformationMatrix();

    // transform
    translate_L(-tX, -tY, -tZ);
    rotateAboutZ_L(yaw);
    rotateAboutY_L(pitch);
    rotateAboutX_L(roll);

    // R
    glm::mat4 Tr(1.f);
    Tr[0][0] =  0.f;   Tr[1][0] =  1.f;   Tr[2][0] =  0.f;
    Tr[0][1] =  0.f;   Tr[1][1] =  0.f;   Tr[2][1] = -1.f;
    Tr[0][2] = -1.f;   Tr[1][2] =  0.f;   Tr[2][2] =  0.f;

    // T' = T(R) * T
    T_ = Tr * T_;
}

void RigidBody::resetTransformationMatrix()
{
    T_ = glm::identity<glm::mat4>();
    //T_ = glm::mat4(1.f);
}

// getter
glm::mat4& RigidBody::getTransformationMatrix()
{
    return T_;
}

const glm::mat4& RigidBody::getTransformationMatrix() const
{
    return T_;
}

// rotate
void RigidBody::rotateAboutX_L(const float angle)
{
    // constants
    const float s = sin(angle);
    const float c = cos(angle);

    // Rx
    glm::mat4 Tr(1.f);
    Tr[1][1] = c; Tr[2][1] = -s;
    Tr[1][2] = s; Tr[2][2] =  c;

    // T' = T(Rx) * T
    T_ = Tr * T_;
}

void RigidBody::rotateAboutY_L(const float angle)
{
    // constants
    const float s = sin(angle);
    const float c = cos(angle);

    // Ry
    glm::mat4 Tr(1.f);
    Tr[0][0] =  c; Tr[2][0] = s;
    Tr[0][2] = -s; Tr[2][2] = c;

    // T' = T(Ry) * T
    T_ = Tr * T_;
}

void RigidBody::rotateAboutZ_L(const float angle)
{
    // constants
    const float s = sin(angle);
    const float c = cos(angle);

    // Rz
    glm::mat4 Tr(1.f);
    Tr[0][0] = c; Tr[1][0] = -s;
    Tr[0][1] = s; Tr[1][1] =  c;

    // T' = T(Rz) * T
    T_ = Tr * T_;
}

void RigidBody::rotate_L(const float yaw, const float pitch, const float roll) // euler angles
{
    // T' = T(R) * T
    const glm::mat4 Tr = glm::yawPitchRoll(yaw, pitch, roll);
    T_ = Tr * T_;
}

void RigidBody::rotate_R(const float yaw, const float pitch, const float roll) // euler angles
{
    // T' = T * T(R)
    const glm::mat4 Tr = glm::yawPitchRoll(yaw, pitch, roll);
    T_ = T_ * Tr;
}

void RigidBody::rotate_L(const float angle,                                        // rotation angle in radian
                         const float axisX, const float axisY, const float axisZ)  // roation axis
{
    // T' = T(R) * T
    //    = [R' 0][R  t] = [R'R  R't]
    //      [0  1][0  1]   [0     1 ]
    const glm::mat4 Tr = glm::rotate(glm::identity<glm::mat4>(), angle, glm::vec3(axisX, axisY, axisZ));
    T_ = Tr * T_;
}

void RigidBody::rotate_R(const float angle,                                        // rotation angle in radian
                         const float axisX, const float axisY, const float axisZ)  // roation axis
{
    // T' = T * T(R)
    //    = [R  t][R' 0] = [RR' t]
    //      [0  1][0  1]   [0   1]
    T_ = glm::rotate(T_, angle, glm::vec3(axisX, axisY, axisZ));
}

// void RigidBody::rotate_L(const float qx, const float qy, const float qz, const float qw) // (unit) quaternion
// {
//     // T' = T(R) * T
//     //    = [Q  0][R  t] = [QR  Qt]
//     //      [0  1][0  1]   [0    1]
//     const glm::quat q(qw, qx, qy, qz);      // ref) glm/detail/type_quat.inl
//     const glm::mat4 Tr = glm::toMat4(q);    // ref) glm/gtx/quaternion.hpp
//     T_ = T_r * T_;
// }

// void RigidBody::rotate_R(const float qx, const float qy, const float qz, const float qw) // (unit) quaternion
// {
//     // T' = T * T(R)
//     //    = [R  t][Q  0] = [RQ  t]
//     //      [0  1][0  1]   [0   1]
//     const glm::quat q(qw, qx, qy, qz);      // ref) glm/detail/type_quat.inl
//     const glm::mat4 Tr = glm::toMat4(q);    // ref) glm/gtx/quaternion.hpp
//     T_ = T_ * T_r;
// }

// translate
void RigidBody::translate_L(const float tX, const float tY, const float tZ) // translation vector
{
    // T' = T(t) * T
    //    = [I  v][R  t] = [R  t+v]
    //      [0  1][0  1]   [0   1 ]
    const glm::mat4 Tt = glm::translate(glm::identity<glm::mat4>(), glm::vec3(tX, tY, tZ));
    T_ = Tt * T_;
}

void RigidBody::translate_R(const float tX, const float tY, const float tZ) // translation vector
{
    // T' = T * T(t)
    //    = [R  t][I  v] = [R  t+Rv]
    //      [0  1][0  1]   [0    1 ]
    T_ = glm::translate(T_, glm::vec3(tX, tY, tZ));
}

// transform = rotation and translation
void RigidBody::transform_L(const float yaw, const float pitch, const float roll,  // euler angles
                            const float tX, const float tY, const float tZ)        // translation vector
{
    // T' = T(R, t) * T = T(t) * T(R) * T
    //                  = [I  v][Q  0][R  t] = [Q  v][R  t] = [QR  Qt+v]
    //                    [0  1][0  1][0  1]   [0  1][0  1]   [0     1 ]
    rotate_L(roll, pitch, yaw);
    translate_L(tX, tY, tZ);
}

void RigidBody::transform_R(const float yaw, const float pitch, const float roll,  // euler angles
                            const float tX, const float tY, const float tZ)        // translation vector
{
    // T' = T * T(R, t) = T * T(t) * T(R)
    //                  = [R  t][I  v][Q  0] = [R  t][Q  v] = [RQ  Rv+t]
    //                    [0  1][0  1][0  1]   [0  1][0  1]   [0     1 ]
    translate_R(tX, tY, tZ);
    rotate_R(roll, pitch, yaw);
}

void RigidBody::transform_L(const float angle,                                         // rotation angle in radian
                            const float axisX, const float axisY, const float axisZ,   // rotation axis
                            const float tX, const float tY, const float tZ)            // translation vector
{
    // T' = T(R, t) * T = T(t) * T(R) * T
    //                  = [I  v][Q  0][R  t] = [Q  v][R  t] = [QR  Qt+v]
    //                    [0  1][0  1][0  1]   [0  1][0  1]   [0     1 ]
    rotate_L(angle, axisX, axisY, axisZ);
    translate_L(tX, tY, tZ);
}

void RigidBody::transform_R(const float angle,                                         // rotation angle in radian
                            const float axisX, const float axisY, const float axisZ,   // rotation axis
                            const float tX, const float tY, const float tZ)            // translation vector
{
    // T' = T * T(R, t) = T * T(t) * T(R)
    //                  = [R  t][I  v][Q  0] = [R  t][Q  v] = [RQ  Rv+t]
    //                    [0  1][0  1][0  1]   [0  1][0  1]   [0     1 ]
    translate_R(tX, tY, tZ);
    rotate_R(angle, axisX, axisY, axisZ);
}

// void RigidBody::transform_L(const float qx, const float qy, const float qz, const float qw,    // (unit) quaternion
//                             const float tX, const float tY, const float tZ)                    // translation vector
// {
//     // T' = T(R, t) * T = T(t) * T(R) * T
//     //                  = [I  v][Q  0][R  t] = [Q  v][R  t] = [QR  Qt+v]
//     //                    [0  1][0  1][0  1]   [0  1][0  1]   [0     1 ]
//     rotate_L(qx, qy, qz, qw);
//     translate_L(tX, tY, tZ);
// }

// void RigidBody::transform_R(const float qx, const float qy, const float qz, const float qw,    // (unit) quaternion
//                             const float tX, const float tY, const float tZ)                    // translation vector
// {
//     // T' = T * T(R, t) = T * T(t) * T(R)
//     //                  = [R  t][I  v][Q  0] = [R  t][Q  v] = [RQ  Rv+t]
//     //                    [0  1][0  1][0  1]   [0  1][0  1]   [0     1 ]
//     translate_R(tX, tY, tZ);
//     rotate_R(qx, qy, qz, qw);
// }

// inverse
void RigidBody::inverseTransformationMatrix()
{
    T_ = inverse(T_);
}

// constructor
ScalableBody::ScalableBody()
{
}

// destructor
ScalableBody::~ScalableBody()
{
}

// scale
void ScalableBody::scale_L(const float sX, const float sY, const float sZ) // scales
{
    // T' = T(s) * T
    //    = [S  0][R  t] = [SR  St]
    //      [0  1][0  1]   [0    1]
    glm::mat4 Ts(1.f);
    Ts[0][0] = sX;
    Ts[1][1] = sY;
    Ts[2][2] = sZ; 
    T_ = Ts * T_;
}

void ScalableBody::scale_R(const float sX, const float sY, const float sZ) // scales
{
    // T' = T * T(s)
    //    = [R  t][S  0] = [RS  t] = [SR  t]
    //      [0  1][0  1]   [0   1]   [0   1]
    T_ = glm::scale(T_, glm::vec3(sX, sY, sZ));
}