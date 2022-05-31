#ifndef _AXES_H_
#define _AXES_H_

// swan
#include <model.h>
#include <shader.h>
#include <rigid.h>

class Axes : public ScalableBody
{
public:
    // constructor
    Axes();
    Axes(const float scale);
    Axes(const glm::mat4 &T);

    // destructor
    virtual ~Axes();

    // setter
    void setLineWidth(const float lineWidth);

    // draw
    void draw(const glm::mat4 &viewProjectionMatrix = glm::mat4(1.f)) const;

    // inherited setter and getter
    //   setTransformationMatrix():   set (model) matrix
    // resetTransformationMatrix(): reset (model) matrix
    //   getTransformationMatrix():   get (model) matrix

protected:
    // line width
    float lineWidth_ = 5.f;

    // inherited (model) matrix
    glm::mat4 &model_; // G_T_O

    // axes
    shared_ptr<Model> axis_[3];

    // shader program
    shared_ptr<ShaderProgram> shaderProgram_[3];
};

#endif // _AXES_H_