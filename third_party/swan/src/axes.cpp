#include <axes.h>

// constructor
Axes::Axes(const float scale)
: scale_(scale)
{
    // vertex shader
    const string vertexShaderSource = "#version 330 core\n"
                                      "layout (location = 0) in vec3 vertexPosition;\n"
                                      "uniform float scale;\n"
                                      "uniform mat4 model_view_projection;\n"
                                      "void main()\n"
                                      "{\n"
                                      "   gl_Position = model_view_projection * vec4(scale * vertexPosition, 1.f);\n"
                                      "}\0";

    // fragment shader: R, G, B
    const string fragmentShaderSource[] = { "#version 330 core\n"
                                            "out vec4 fragmentColor;\n"
                                            "void main()\n"
                                            "{\n"
                                            "   fragmentColor = vec4(1.f, 0.f, 0.f, 1.0f);\n"
                                            "}\n\0",

                                            "#version 330 core\n"
                                            "out vec4 fragmentColor;\n"
                                            "void main()\n"
                                            "{\n"
                                            "   fragmentColor = vec4(0.f, 1.f, 0.f, 1.0f);\n"
                                            "}\n\0",

                                            "#version 330 core\n"
                                            "out vec4 fragmentColor;\n"
                                            "void main()\n"
                                            "{\n"
                                            "   fragmentColor = vec4(0.f, 0.f, 1.f, 1.0f);\n"
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
    setScale(scale_);
}

Axes::~Axes()
{
}

// setter
void Axes::setScale(const float scale)
{
    scale_ = scale;

    // for each axis
    for(int i = 0; i < 3; i++)
    {
        shaderProgram_[i]->set("scale", scale_);
    }    
}

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