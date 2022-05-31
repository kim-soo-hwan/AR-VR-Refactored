// GLM
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>             // yawPitchRoll

#include <model.h>


// constructor
Model::Model(const int numVertices)
: Mesh(numVertices),
  model_(T_)
{
}

// destructor
Model::~Model()
{
}

// set the name of the transformation matrix: uniform variable
void Model::setModelViewProjectionMatrixName(const string& name)
{
    modelViewProjectionMatrixName_ = name;
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