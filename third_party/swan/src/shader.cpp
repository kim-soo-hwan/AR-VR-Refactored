#include <fstream>
#include <sstream>
#include <iostream>
using namespace std;

#include <shader.h>

// constructor
Shader::Shader(const GLenum shaderType)
{
    // create a shader
    _id = glCreateShader(shaderType);
}

// destructor
Shader::~Shader()
{
    // delete the shader
    glDeleteShader(_id);
}

// load and compile the shader code
bool Shader::loadAndCompile(const string &filePath)
{
    // load the shader code from file
    string shaderCode;
    if(!load(filePath, shaderCode)) return false;

    // compile the shader code
    if(!compile(shaderCode)) return false;

    return true;
}

// load the shader code from file
bool Shader::load(const string &filePath, string &shaderCode) const
{
    // load the shader source code using file streams
    try
    {
        // shader input file streams
        ifstream shaderFile;

        // ensure ifstream objects can throw exceptions
        shaderFile.exceptions(ifstream::failbit | ifstream::badbit);

        // open files
        shaderFile.open(filePath);

        // read file's buffer contents into streams
        stringstream shaderStream;
        shaderStream << shaderFile.rdbuf();

        // close file handlers
        shaderFile.close();

        // convert stream into string
        shaderCode = shaderStream.str();
    }
    catch (ifstream::failure e)
    {
        cout << "[ERROR] Shader: Loading Failed! " << filePath << endl;
        return false;
    }

    return true;
}

// compile the shader code
bool Shader::compile(const string &shaderCode)
{
    // shader code
    const char* shaderSource = shaderCode.c_str();

    // set the source code
    glShaderSource(_id, 1, &shaderSource, NULL);

    // compile the source code
    glCompileShader(_id);

    // check for shader compile errors
    int success;
    GLchar infoLog[1024];
    glGetShaderiv(_id, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(_id, 1024, NULL, infoLog);
        cout << "[ERROR] Shader: Compile Failed! " << infoLog << endl;
        cout << shaderCode << endl;

        return false;
    }

    return true;
}

// getter
GLuint Shader::id() const
{
    return _id;
}

// constructor
ShaderProgram::ShaderProgram()
{
    // create a shader program
    _id = glCreateProgram();

    // maximum number of texture units
    glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &MAX_NUM_TEXTURE_UNITS);
}

// destructor
ShaderProgram::~ShaderProgram()
{
    // delete the shader program
    glDeleteProgram (_id);
}

// create a shader from file
bool ShaderProgram::createShader(const GLenum shaderType, const string &filePath)
{
    // shader
    shared_ptr<Shader> shader = make_shared<Shader>(shaderType);

    // load and compile the source code
    if(!shader->loadAndCompile(filePath)) return false;

    // keep it in the list
    _shaders.push_back(shader);

    return true;
}

// attach and link shaders
bool ShaderProgram::attachAndLinkShaders() const
{
    // for each shader
    for(auto &shader : _shaders)
    {
        // attach it to the shader program
        if(shader) glAttachShader(_id, shader->id());
    }

    // link the shader program
    glLinkProgram(_id);

    // check for the link error
    int success;
    GLchar infoLog[1024];
    glGetProgramiv(_id, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(_id, 1024, NULL, infoLog);
        cout << "[ERROR] Program Shader: Link Failed! " << infoLog << endl;
    }

    return true;
}

// use the shader program
void ShaderProgram::use() const
{
    // use the shader program
    glUseProgram(_id);

    // bind the texture
    if(_texture) _texture->bind();

    // bind the texture units
    GLuint numTextureUnits = 0;
    for(const auto& texture : _textureUnits)
    {
        if (texture)
        {
            // active texture
            glActiveTexture(GL_TEXTURE0 + numTextureUnits);
            texture->bind();
        }
    }
}

// set uniform variables
void ShaderProgram::set(const string& name, const GLint value) const
{
    glUseProgram(_id);
    GLuint location = glGetUniformLocation(_id, name.c_str());
    glUniform1i(location, value);
}

void ShaderProgram::set(const string& name, const GLuint value) const
{
    glUseProgram(_id);
    GLuint location = glGetUniformLocation(_id, name.c_str());
    glUniform1ui(location, value);
}

void ShaderProgram::set(const string& name, const GLfloat value) const
{
    glUseProgram(_id);
    GLuint location = glGetUniformLocation(_id, name.c_str());
    glUniform1f(location, value);
}

void ShaderProgram::set(const string& name, const GLfloat v0, const GLfloat v1) const
{
    glUseProgram(_id);
    GLuint location = glGetUniformLocation(_id, name.c_str());
    glUniform2f(location, v0, v1);
}

void ShaderProgram::set(const string& name, const GLfloat v0, const GLfloat v1, const GLfloat v2) const
{
    glUseProgram(_id);
    GLuint location = glGetUniformLocation(_id, name.c_str());
    glUniform3f(location, v0, v1, v2);
}

void ShaderProgram::set(const string& name, const GLfloat v0, const GLfloat v1, const GLfloat v2, const GLfloat v3) const
{
    glUseProgram(_id);
    GLuint location = glGetUniformLocation(_id, name.c_str());
    glUniform4f(location, v0, v1, v2, v3);
}

void ShaderProgram::set(const string& name, const unsigned int dim, GLsizei count, const GLfloat* value) const
{
    glUseProgram(_id);
    GLuint location = glGetUniformLocation(_id, name.c_str());
    switch (dim)
    {
        case 1: glUniform1fv(location, count, value); break;
        case 2: glUniform2fv(location, count, value); break;
        case 3: glUniform3fv(location, count, value); break;
        case 4: glUniform4fv(location, count, value); break;
    }
}

// texture
void ShaderProgram::setTexture(const shared_ptr<Texture> &texture)
{
    _texture = texture;
}


bool ShaderProgram::addTextureUnit(const string &name, const shared_ptr<Texture> &texture)
{
    // check the number of texture units
    if (_textureUnits.size() >= MAX_NUM_TEXTURE_UNITS) return false;

    // set the uniform variable
    set(name, static_cast<int>(_textureUnits.size()));

    // keep it
    _textureUnits.push_back(texture);

    return true;
}
