// std
#include <iostream>
using namespace std;

// GLM
#include <glm/ext/scalar_relational.hpp> // epsilonEqual
#include <glm/gtc/epsilon.hpp>

#include <utility.h>

// vertex shader
const char* vertexShaderSource = "#version 330 core\n"
                                 "layout (location = 0) in vec3 vertexPosition;\n"
                                 "void main()\n"
                                 "{\n"
                                 "   gl_Position = vec4(vertexPosition, 1.f);\n"
                                 "}\0";

// fragment shader
const char* fragmentShaderSource = "#version 330 core\n"
                                   "out vec4 fragmentColor;\n"
                                   "void main()\n"
                                   "{\n"
                                   "   fragmentColor = vec4(1.f, 0.5f, 0.2f, 1.f);\n"
                                   "}\n\0";

GLuint generateAndUseDefaultShaderProgram()
{
    // create and compile the vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // check for shader compile errors
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED" << endl;
        cout << infoLog << endl;
    }

    // create and compile the fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // check for shader compile errors
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED" << endl;
        cout << infoLog << endl;
    }

    // create a shader program
    GLuint shaderProgram = glCreateProgram();

    // attach the vertex and fragment shaders into the shader program
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // check for shader program link errors
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        cout << "ERROR::SHADER::FRAGMENT::LINKING_FAILED" << endl;
        cout << infoLog << endl;
    }

    // delete the vertex and fragment shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // use the shader program
    glUseProgram(shaderProgram);
    
    return shaderProgram;
}

void deleteDefaultShaderProgram(const GLuint shaderProgram)
{
    glDeleteProgram(shaderProgram);
}