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

void Shader::begin() const
{
    use();
    for (GLuint i = 0; i < m_textures.size(); i++)
    {
        const Texture& texture = m_textures[i];
        texture.bind(i);
    }
}

void Shader::end() const
{
    for (GLuint i = 0; i < m_textures.size(); i++)
    {
        const Texture& texture = m_textures[i];
        texture.unbind();
    }
}


// setter
void Shader::set(const string& name, GLint value) const
{
    use();
    GLuint location = glGetUniformLocation(m_shaderProgram, name.c_str());
    glUniform1i(location, value);
}

void Shader::set(const string& name, GLuint value) const
{
    use();
    GLuint location = glGetUniformLocation(m_shaderProgram, name.c_str());
    glUniform1ui(location, value);
}

void Shader::set(const string& name, GLfloat value) const
{
    use();
    GLuint location = glGetUniformLocation(m_shaderProgram, name.c_str());
    glUniform1f(location, value);
}

void Shader::set(const string& name, GLfloat v0, GLfloat v1) const
{
    use();
    GLuint location = glGetUniformLocation(m_shaderProgram, name.c_str());
    glUniform2f(location, v0, v1);
}

void Shader::set(const string& name, GLfloat v0, GLfloat v1, GLfloat v2) const
{
    use();
    GLuint location = glGetUniformLocation(m_shaderProgram, name.c_str());
    glUniform3f(location, v0, v1, v2);
}

void Shader::set(const string& name, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3) const
{
    use();
    GLuint location = glGetUniformLocation(m_shaderProgram, name.c_str());
    glUniform4f(location, v0, v1, v2, v3);
}

void Shader::set(const string& name, const unsigned int dim, GLsizei count, const GLfloat* value) const
{
    use();
    GLuint location = glGetUniformLocation(m_shaderProgram, name.c_str());
    switch (dim)
    {
    case 1: glUniform1fv(location, count, value); break;
    case 2: glUniform2fv(location, count, value); break;
    case 3: glUniform3fv(location, count, value); break;
    case 4: glUniform4fv(location, count, value); break;
    }
}

void Shader::set(const string& name, const glm::mat4& T) const
{
    use();

    // query the location of the uniform variable
    GLuint loc = glGetUniformLocation(m_shaderProgram, name.c_str());

    // copy the transformation matrix
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(T));
}

void Shader::set(const string& name, const Texture& texture)
{
    set(name, (GLint)m_textures.size());
    m_textures.push_back(std::ref(texture));
}