#include <axes.h>

// constructor
Axes::Axes(const float scale)
: scale_(scale)
{
    // vertex shader
    const string vertexShaderSource = "#version 430 core\n"
                                      "layout (location = 0) in vec3 vertexPosition;\n"
                                      "uniform mat4 model_view_projection;\n"
                                      "void main()\n"
                                      "{\n"
                                      "   gl_Position = model_view_projection * vec4(vertexPosition, 1.f);\n"
                                      "}\0";

    // fragment shader: R, G, B
    const string fragmentShaderSource[] = { "#version 430 core\n"
                                            "out vec4 fragmentColor;\n"
                                            "void main()\n"
                                            "{\n"
                                            "   fragmentColor = vec4(1.f, 0.f, 0.f, 1.0f);\n"   // R
                                            "}\n\0",

                                            "#version 430 core\n"
                                            "out vec4 fragmentColor;\n"
                                            "void main()\n"
                                            "{\n"
                                            "   fragmentColor = vec4(0.f, 1.f, 0.f, 1.0f);\n"   // G
                                            "}\n\0",

                                            "#version 430 core\n"
                                            "out vec4 fragmentColor;\n"
                                            "void main()\n"
                                            "{\n"
                                            "   fragmentColor = vec4(0.f, 0.f, 1.f, 1.0f);\n"   // B
                                            "}\n\0"
                                          };

    // vertex input: O_p
    GLfloat vertices[6] = {
        // start
        0.f, 0.f, 0.f,

        // end (origin)
        0.f, 0.f, 0.f
    };

    // for each axis
    for(int i = 0; i < 3; i++)
    {
        // shader program
        shaderProgram_[i] = make_shared<ShaderProgram>();
        shaderProgram_[i]->createShaderFromString(GL_VERTEX_SHADER,   vertexShaderSource);
        shaderProgram_[i]->createShaderFromString(GL_FRAGMENT_SHADER, fragmentShaderSource[i]);
        shaderProgram_[i]->attachAndLinkShaders();

        // vertex input: O_p
        for(int j = 0; j < 3; j++)
        {
            if(j == i) vertices[j] = 1.f;
            else       vertices[j] = 0.f;
        }

        // model
        axis_[i] = make_shared<Model>(2);
        axis_[i]->setVertexAttributes(vertices, 3);
        axis_[i]->setDrawMode(GL_LINES);
        axis_[i]->setShaderProgram(shaderProgram_[i]);
        axis_[i]->setModelViewProjectionMatrixName("model_view_projection");
    }

    // scale
    this->scale(scale_, scale_, scale_);
}

Axes::~Axes()
{
}

// transform
void Axes::scale(const float scaleX, const float scaleY, const float scaleZ)   // scale
{
    axis_[0]->scale(scaleX, 1.f, 1.f);  // x-axis
    axis_[1]->scale(1.f, scaleY, 1.f);  // y-axis
    axis_[2]->scale(1.f, 1.f, scaleZ);  // z-axis
}

void Axes::rotate(const float angle,                                           // rotation angle in radian
                  const float axisX, const float axisY, const float axisZ)     // rotation axis
{
    // for each axis
    for(int i = 0; i < 3; i++)
    {
        axis_[i]->rotate(angle, axisX, axisY, axisZ);
    }  
}

void Axes::translate(const float tX, const float tY, const float tZ)           // translation vector
{
    // for each axis
    for(int i = 0; i < 3; i++)
    {
        axis_[i]->translate(tX, tY, tZ);
    } 
}

void Axes::transform(const float angle,                                        // rotation angle in radian
                      const float axisX, const float axisY, const float axisZ,  // roation axis
                      const float tX, const float tY, const float tZ)           // translation vector
{
    // for each axis
    for(int i = 0; i < 3; i++)
    {
        axis_[i]->transform(angle, axisX, axisY, axisZ, tX, tY, tZ);
    } 
}

void Axes::resetModelMatrix()
{
    // for each axis
    for(int i = 0; i < 3; i++)
    {
        axis_[i]->resetModelMatrix();
    } 
}

// setter
void Axes::setLineWidth(const float lineWidth)
{
    lineWidth_ = lineWidth;
}

// draw
void Axes::draw(const glm::mat4 &viewProjectionMatrix) const
{
    // line width
    glLineWidth(lineWidth_);

    // for each axis
    for(int i = 0; i < 3; i++)
    {
        axis_[i]->draw(viewProjectionMatrix);
    }  
}