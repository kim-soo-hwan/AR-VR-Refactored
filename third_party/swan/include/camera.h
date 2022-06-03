#ifndef _CAMERA_H_
#define _CAMERA_H_

// GLM
#include <glm/glm.hpp>

// swan
#include <rigid.h>

class Camera : public RigidBody
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

    // view
    void setViewMatrixForMobilTech(const float yaw, const float pitch, const float roll,     // Euler angles
                                              const float tX,  const float tY,    const float tZ);      // translation vector

    // projection
    void setProjectionMatrix(const float fx, const float fy,    // focal length
                             const float cx, const float cy,    // optical center
                             const float zn, const float zf,    // near/far plane
                             const float w,  const float h);    // width/height of the image

    // inherited setter and getter
    //   setTransformationMatrix():   set (view) matrix
    // resetTransformationMatrix(): reset (view) matrix
    //   getTransformationMatrix():   get (view) matrix

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
    // projection matrix: camera intrinsices
    glm::mat4 projection_;  // NDC_T_C

    // inherited (view) matrix: camera extrinsices
    glm::mat4 &view_;       // C_T_G
};

#endif // _CAMERA_H_