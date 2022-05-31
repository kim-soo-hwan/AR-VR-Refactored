#ifndef __MODEL_H__
#define __MODEL_H__

// std
#include <string>
using namespace std;

// GLM
#include <glm/glm.hpp>

// swan
#include <mesh.h>
#include <rigid.h>

class Model : public Mesh, public ScalableBody
{
public:
    // constructor
    Model(const int numVertices);

    // destructor
    virtual ~Model();

    // set the name of the transformation matrix: uniform variable
    void setModelViewProjectionMatrixName(const string& name);
    
    // draw
    void draw(const glm::mat4 &viewProjectionMatrix = glm::mat4(1.f)) const;

    // inherited setter and getter
    //   setTransformationMatrix():   set (model) matrix
    // resetTransformationMatrix(): reset (model) matrix
    //   getTransformationMatrix():   get (model) matrix

protected:
    // model-view-projection matrix: uniform variable name
    string modelViewProjectionMatrixName_;

    // inherited (model) matrix
    glm::mat4 &model_; // G_T_O
};

#endif // __MODEL_H__