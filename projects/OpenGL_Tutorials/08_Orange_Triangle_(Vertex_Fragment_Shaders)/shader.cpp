#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
using namespace std;

#include "shader.hpp"

// constructor
Shader::Shader(const char* vertexPath, const char* fragmentPath)
{
    // [Step 1] load the vertex/fragment source code from the files

    // vertex/fragment source code
    string vertexCode;
    string fragmentCode;

    // load the vertex/fragment source code using file streams
    try
    {
        // vertex/fragment input file streams
        ifstream vShaderFile;
        ifstream fShaderFile;

        // ensure ifstream objects can throw exceptions
        vShaderFile.exceptions(ifstream::failbit | ifstream::badbit);
        fShaderFile.exceptions(ifstream::failbit | ifstream::badbit);

        // open files
        vShaderFile.open(vertexPath);
        fShaderFile.open(fragmentPath);

        // read file's buffer contents into streams
        stringstream vShaderStream, fShaderStream;
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();

        // close file handlers
        vShaderFile.close();
        fShaderFile.close();

        // convert stream into string
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();
    }
    catch (ifstream::failure e)
    {
        cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << endl;
    }
    const char* vertexShaderSource = vertexCode.c_str();
    const char* fragmentShaderSource = fragmentCode.c_str();

    // [Step 2] create and compile the vertex/fragment shaders

    // vertex Shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // check for shader compile errors
    checkErrors(vertexShader, "VERTEX");

    // create and compile the fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // check for shader compile errors
    checkErrors(fragmentShader, "FRAGMENT");

    // [Step 3] create and link a shader program

    // create a shader program
    m_shaderProgram = glCreateProgram();

    // attach the vertex and fragment shaders into the shader program
    glAttachShader(m_shaderProgram, vertexShader);
    glAttachShader(m_shaderProgram, fragmentShader);
    glLinkProgram(m_shaderProgram);

    // check for shader program link errors
    checkErrors(m_shaderProgram, "PROGRAM");

    // delete the vertex and fragment shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Shader::checkErrors(GLuint shader, const string& type)
{
    int success;
    GLchar infoLog[1024];

    // shader program
    if (type == "PROGRAM")
    {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success)
        {
            glGetProgramInfoLog(shader, 1024, NULL, infoLog);
            cout << "ERROR::SHADER::" << type << "::LINKING_FAILED\n" << infoLog << endl;
        }
    }

    // vertex/fragment shader
    else
    {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            cout << "ERROR::SHADER::" << type << "::COMPILATION_FAILED\n" << infoLog << endl;
        }
    }
}

// activate the shader program
void Shader::use() const
{
    glUseProgram(m_shaderProgram);
}