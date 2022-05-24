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

    // projection
    void setProjectionMatrix(const float fx, const float fy, const float cx, const float cy);

    // view: transform
    void rotate(const float angleInDegrees,                                 // rotation angle
                const float axisX, const float axisY, const float axisZ);   // rotation axis
    void rotate(const float roll, const float pitch, const float yaw);      // euler angles
    void translate(const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float angleInDegrees,                              // rotation angle
                   const float axisX, const float axisY, const float axisZ, // rotation axis
                   const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float roll, const float pitch, const float yaw,    // euler angles
                   const float tX, const float tY, const float tZ);         // translation vector
    void setViewMatrix(const float r11, const float r12, const float r13,
                       const float r21, const float r22, const float r23,
                       const float r31, const float r32, const float r33,
                       const float tX,  const float tY,  const float tZ);
    void resetViewMatrix();

    // view: loot at
    void lookAt(const float eyeX,    const float eyeY,    const float eyeZ,     // camera position
                const float centerX, const float centerY, const float centerZ,  // where to look at
                const float upX,     const float upY,     const float upZ);     // up vector
    
    // view: move (translate) by keyboard or mouse
    void moveForward (const float displacement);
    void moveBackward(const float displacement);
    void moveRight   (const float displacement);
    void moveLeft    (const float displacement);
    void moveUp      (const float displacement);
    void moveDown    (const float displacement);

    // view: pan/tilt (rotate) by mouse
    void pan(const float angleInDegrees);
    void tilt(const float angleInDegrees);

    // getter
    glm::mat4 getProjectionMatrix() const;
    glm::mat4 getViewMatrix() const;
    glm::mat4 getViewProjectionMatrix() const;

protected:
    void moveInGlobalCoords(const glm::vec3 &G_t);

protected:
    glm::mat4 projection_;  // camera intrinsics: F_T_C
    glm::mat4 view_;        // camera extrinsics: C_T_G
};

#endif // _CAMERA_H_