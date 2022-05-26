// GLM
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>             // yawPitchRoll

#include <model.h>


// constructor
Model::Model(const int numVertices)
: Mesh(numVertices),
  model_(1.f)
{
}

// destructor
Model::~Model()
{
}

// transform
void Model::scale(const float scaleX, const float scaleY, const float scaleZ)   // scale
{
    // [caution] 
    // glm::scale multiplies a new 4x4 transformation matrix 
    // created from the three scale factors "to the right side" 
    // of the passed 4x4 transformation matrix
    // T' = T * T(s)
    model_ = glm::scale(model_, glm::vec3(scaleX, scaleY, scaleZ));
}

void Model::rotate(const float angle,                                           // rotation angle in radian
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
    model_ = glm::rotate(model_, angle, glm::vec3(axisX, axisY, axisZ));

    // [caution] glm::rotate: the angle is expressed in radians
    // as mentioned in the glm code (glm/gtx/transform.hpp),
    // not in degrees as mentioned in the glm web site (https://glm.g-truc.net/0.9.9/api/a00247.html)
}

void Model::rotate(const float roll, const float pitch, const float yaw)       // euler angles
{
    // [caution] 
    // This only applies to the transformation matrix "to the right side"
    // of the existing view matrix just like glm::rotate.
    // T(view)'  = T(view) * T(R)
    // T(view)' != T(R) * T(view)
    const glm::mat4 T = glm::yawPitchRoll(yaw, pitch, roll);
    model_ =  model_ * T;
}

void Model::translate(const float tX, const float tY, const float tZ)           // translation vector
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
    model_ = glm::translate(model_, glm::vec3(tX, tY, tZ));
}

void Model::transform(const float angle,                                        // rotation angle in radian
                      const float axisX, const float axisY, const float axisZ,  // roation axis
                      const float tX, const float tY, const float tZ)           // translation vector
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

void Model::transform(const float roll, const float pitch, const float yaw,    // euler angles
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

void Model::setModelMatrix(const float r11, const float r12, const float r13, const float tX,   // 1st row
                           const float r21, const float r22, const float r23, const float tY,   // 2nd row
                           const float r31, const float r32, const float r33, const float tZ)   // 3rd row
{
    // 1st column      2nd column         3rd column         4th column
    model_[0][0] = r11; model_[1][0] = r12; model_[2][0] = r13, model_[3][0] = tX;
    model_[0][1] = r21; model_[1][1] = r22; model_[2][1] = r23, model_[3][1] = tY;
    model_[0][2] = r31; model_[1][2] = r32; model_[2][2] = r33, model_[3][2] = tZ;
    model_[0][3] = 0.f; model_[1][3] = 0.f; model_[2][3] = 0.f, model_[3][3] = 1.f;
}

void Model::resetModelMatrix()
{
    model_ = glm::mat4(1.f);
}

void Model::setModelViewProjectionMatrixName(const string& name)
{
    modelViewProjectionMatrixName_ = name;
}

// getter
glm::mat4 Model::getModelMatrix() const
{
    return model_;
}

// draw
void Model::draw(const glm::mat4 &viewProjectionMatrix) const
{
    // model-view-projection matrix: F_T_O = NDC_T_C * C_T_G * G_T_O
    glm::mat4 modelViewProjectionMatrix = viewProjectionMatrix * model_;

    // set the transform variable
    if (!modelViewProjectionMatrixName_.empty() && shaderProgram_) 
        shaderProgram_->set(modelViewProjectionMatrixName_, modelViewProjectionMatrix);

    // draw mesh
    Mesh::draw();
}