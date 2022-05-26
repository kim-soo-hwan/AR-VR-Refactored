#ifndef __MODEL_H__
#define __MODEL_H__

// std
#include <string>
using namespace std;

// GLM
#include <glm/glm.hpp>

// swan
#include <mesh.h>

class Model : public Mesh
{
public:
    // constructor
    Model(const int numVertices);

    // destructor
    virtual ~Model();

    // transform
    void scale(const float scaleX, const float scaleY, const float scaleZ); // scale
    void rotate(const float roll, const float pitch, const float yaw);      // euler angles
    void rotate(const float angle,                                          // rotation angle in radian
                const float axisX, const float axisY, const float axisZ);   // roation axis
    void translate(const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float roll, const float pitch, const float yaw,    // euler angles
                   const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float angle,                                       // rotation angle in radian
                   const float axisX, const float axisY, const float axisZ, // roation axis
                   const float tX, const float tY, const float tZ);         // translation vector
    void setModelMatrix(const float r11, const float r12, const float r13, const float tX,   // 1st row
                        const float r21, const float r22, const float r23, const float tY,   // 2nd row
                        const float r31, const float r32, const float r33, const float tZ);   // 3rd row

    void resetModelMatrix();

    // set the name of the transformation matrix: uniform variable
    void setModelViewProjectionMatrixName(const string& name);
    
    // getter
    glm::mat4 getModelMatrix() const;

    // draw
    void draw(const glm::mat4 &viewProjectionMatrix = glm::mat4(1.f)) const;

protected:
    // model-view-projection matrix: uniform variable name
    string modelViewProjectionMatrixName_;

    // model matrix
    glm::mat4 model_; // model matrix: G_T_O
};

#endif // __MODEL_H__