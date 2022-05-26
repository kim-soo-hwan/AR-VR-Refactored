#ifndef _CAMERA_H_
#define _CAMERA_H_

// GLM
#include <glm/glm.hpp>

class Camera
{
public:
    // constructor
    Camera();
    Camera(const float fovy,    // vertical field of view in radian
           const float aspect,  // aspect ratio of field of view
           const float near,    // near plane
           const float far);    // far plane

    // destructor
    virtual ~Camera();

    // projection
    void setProjectionMatrix(const float fx, const float fy,    // focal length
                             const float cx, const float cy,    // optical center
                             const float zn, const float zf,    // near/far plane
                             const float w,  const float h);    // width/height of the image

    // view: transform
    void rotate(const float angle,                                          // rotation angle in radian
                const float axisX, const float axisY, const float axisZ);   // rotation axis
    void rotate(const float roll, const float pitch, const float yaw);      // euler angles
    void translate(const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float roll, const float pitch, const float yaw,    // euler angles
                   const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float angle,                                       // rotation angle in radian
                   const float axisX, const float axisY, const float axisZ, // rotation axis
                   const float tX, const float tY, const float tZ);         // translation vector
    void setViewMatrix(const float r11, const float r12, const float r13, const float tX,   // 1st row
                       const float r21, const float r22, const float r23, const float tY,   // 2nd row
                       const float r31, const float r32, const float r33, const float tZ);  // 3rd row
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
    void pan(const float angle);    // angle in radian
    void tilt(const float angle);   // angle in radian

    // getter
    glm::mat4 getProjectionMatrix() const;
    glm::mat4 getViewMatrix() const;
    glm::mat4 getViewProjectionMatrix() const;

protected:
    void moveInGlobalCoords(const glm::vec3 &G_t);

protected:
    glm::mat4 projection_;  // camera intrinsics: NDC_T_C
    glm::mat4 view_;        // camera extrinsics: C_T_G
};

#endif // _CAMERA_H_