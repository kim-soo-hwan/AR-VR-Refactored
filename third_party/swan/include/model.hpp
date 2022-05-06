#ifndef __MODEL_HPP__
#define __MODEL_HPP__

// GLM
#include <glm/glm.hpp>

#include "camera.hpp"
#include "shader.hpp"

class Model
{
public:
    // constructor
    Model(const Camera &camera, const Shader &shader);

    // destructor
    virtual ~Model();

    // transform
    void rotate(const float angleInDegrees,                                 // rotation angle
                const float axisX, const float axisY, const float axisZ);   // roation axis
    void translate(const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float angleInDegrees,                              // rotation angle
                   const float axisX, const float axisY, const float axisZ, // roation axis
                   const float tX, const float tY, const float tZ);         // translation vector
    void resetModelMatrix();
    
    // getter
    glm::mat4 getModelMatrix() const;

protected:
    // matrices
    glm::mat4 model_; // model matrix: G_T_O

    // camera
    const Camera &camera_;

    // Shader
    const Shader &shader_;
};

#endif // __MODEL_HPP__