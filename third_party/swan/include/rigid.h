#ifndef _RIGID_H_
#define _RIGID_H_

// std
#include <ostream>
using namespace std;

// GLM
#include <glm/glm.hpp>

// Rigid Body 
// Pose = Coordinate System = Transformation Matrix
class RigidBody
{
public:
    // constructor
    RigidBody();

    // destructor
    virtual ~RigidBody();

    // operator
    friend ostream& operator<< (ostream& os, const RigidBody &T);

    // setter
    void setTransformationMatrix(const glm::mat4 T);
    void setTransformationMatrix(const float r11, const float r12, const float r13, const float tX,     // 1st row
                                 const float r21, const float r22, const float r23, const float tY,     // 2nd row
                                 const float r31, const float r32, const float r33, const float tZ);    // 3rd row
    void setTransformationMatrixforMobileTech(const float yaw, const float pitch, const float roll,     // Euler angles
                                              const float tX,  const float tY,    const float tZ);      // translation vector
    void resetTransformationMatrix();

    // getter
          glm::mat4& getTransformationMatrix();
    const glm::mat4& getTransformationMatrix() const;

    // rotate
    void rotateAboutX_L(const float angle);
    void rotateAboutY_L(const float angle);
    void rotateAboutZ_L(const float angle);
    void rotate_L(const float yaw, const float pitch, const float roll);                // Euler angles
    void rotate_R(const float yaw, const float pitch, const float roll);                // Euler angles
    void rotate_L(const float angle,                                                    // rotation angle in radian
                  const float axisX, const float axisY, const float axisZ);             // roation axis
    void rotate_R(const float angle,                                                    // rotation angle in radian
                  const float axisX, const float axisY, const float axisZ);             // roation axis
    //void rotate_L(const float qx, const float qy, const float qz, const float qw);      // (unit) quaternion
    //void rotate_R(const float qx, const float qy, const float qz, const float qw);      // (unit) quaternion

    // translate
    void translate_L(const float tX, const float tY, const float tZ);                   // translation vector
    void translate_R(const float tX, const float tY, const float tZ);                   // translation vector

    // transform = rotation and translation
    void transform_L(const float roll, const float pitch, const float yaw,              // Euler angles
                     const float tX, const float tY, const float tZ);                   // translation vector
    void transform_R(const float roll, const float pitch, const float yaw,              // Euler angles
                     const float tX, const float tY, const float tZ);                   // translation vector
    void transform_L(const float angle,                                                 // rotation angle in radian
                     const float axisX, const float axisY, const float axisZ,           // rotation axis
                     const float tX, const float tY, const float tZ);                   // translation vector
    void transform_R(const float angle,                                                 // rotation angle in radian
                     const float axisX, const float axisY, const float axisZ,           // rotation axis
                     const float tX, const float tY, const float tZ);                   // translation vector
    //void transform_L(const float qx, const float qy, const float qz, const float qw,    // (unit) quaternion
    //                 const float tX, const float tY, const float tZ);                   // translation vector
    //void transform_R(const float qx, const float qy, const float qz, const float qw,    // (unit) quaternion
    //                 const float tX, const float tY, const float tZ);                   // translation vector

    // inverse
    void inverseTransformationMatrix();

protected:
    // transformation matrix
    glm::mat4 T_;
};

class ScalableBody : public RigidBody
{
public:
    // constructor
    ScalableBody();

    // destructor
    virtual ~ScalableBody();

    // scale
    void scale_L(const float sX, const float sY, const float sZ);   // scales
    void scale_R(const float sX, const float sY, const float sZ);   // scales
};

#endif // _RIGID_H_