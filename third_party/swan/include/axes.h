#ifndef _AXES_H_
#define _AXES_H_

// swan
#include <model.h>
#include <shader.h>

class Axes
{
public:
    // constructor
    Axes(const float scale = 1.f);

    // destructor
    virtual ~Axes();

    // transform
    void scale(const float scaleX, const float scaleY, const float scaleZ); // scale
    void rotate(const float angle,                                          // rotation angle in radian
                const float axisX, const float axisY, const float axisZ);   // roation axis
    void translate(const float tX, const float tY, const float tZ);         // translation vector
    void transform(const float angle,                                       // rotation angle in radian
                   const float axisX, const float axisY, const float axisZ, // roation axis
                   const float tX, const float tY, const float tZ);         // translation vector
    void resetModelMatrix();

    // setter
    void setLineWidth(const float lineWidth);

    // draw
    void draw(const glm::mat4 &viewProjectionMatrix = glm::mat4(1.f)) const;
    
protected:
    // scale
    float scale_ = 1.f;
    float lineWidth_ = 5.f;

    // axes
    shared_ptr<Model> axis_[3];

    // shader program
    shared_ptr<ShaderProgram> shaderProgram_[3];
};

#endif // _AXES_H_