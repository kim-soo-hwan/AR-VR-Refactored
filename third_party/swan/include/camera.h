#ifndef _CAMERA_H_
#define _CAMERA_H_

// GLM
#include <glm/glm.hpp>

class Camera
{
public:
    // constructor
    Camera();
    Camera(const float fovyInDegrees, const float aspect, const float near, const float far);

    // destructor
    virtual ~Camera();

    // transform
    void rotate(const float angleInDegrees,                                 // rotation angle
                const float axisX, const float axisY, const float axisZ);   // roation axis
    void translate(const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float angleInDegrees,                              // rotation angle
                   const float axisX, const float axisY, const float axisZ, // roation axis
                   const float tX, const float tY, const float tZ);         // translation vector
    void resetViewMatrix();

    // getter
    glm::mat4 getProjectionMatrix() const;
    glm::mat4 getViewMatrix() const;
    glm::mat4 getViewProjectionMatrix() const;

protected:
    glm::mat4 projection_;  // camera intrinsics: F_T_C
    glm::mat4 view_;        // camera extrinsics: C_T_G
};

#endif // _CAMERA_H_