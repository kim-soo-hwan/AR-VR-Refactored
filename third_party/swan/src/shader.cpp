#include <fstream>
#include <sstream>
#include <iostream>
using namespace std;

// GLM
#include <glm/gtc/type_ptr.hpp>

#include <shader.h>

// constructors
Shader::Shader(const GLenum shaderType)
{
    // create a shader
    id_ = glCreateShader(shaderType);
}

Shader::Shader(const GLenum shaderType, const string &filePath)
: Shader(shaderType)
{
    loadAndCompile(filePath);
}

// destructor
Shader::~Shader()
{
    // delete the shader
    glDeleteShader(id_);
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
    glShaderSource(id_, 1, &shaderSource, NULL);

    // compile the source code
    glCompileShader(id_);

    // check for shader compile errors
    int success;
    GLchar infoLog[1024];
    glGetShaderiv(id_, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(id_, 1024, NULL, infoLog);
        cout << "[ERROR] Shader: Compile Failed! " << infoLog << endl;
        cout << shaderCode << endl;

        return false;
    }

    return true;
}

// getter
GLuint Shader::id() const
{
    return id_;
}

// constructor
ShaderProgram::ShaderProgram()
{
    // create a shader program
    id_ = glCreateProgram();

    // maximum number of texture units
    glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &MAX_NUM_TEXTURE_UNITS_);
}

// destructor
ShaderProgram::~ShaderProgram()
{
    // delete the shader program
    glDeleteProgram (id_);
}

// create a shader from file
bool ShaderProgram::createShaderFromFile(const GLenum shaderType, const string &filePath)
{
    // shader
    shared_ptr<Shader> shader = make_shared<Shader>(shaderType);

    // load and compile the source code
    if(!shader->loadAndCompile(filePath)) return false;

    // keep it in the list
    shaders_.push_back(shader);

    return true;
}

// create a shader from string
bool ShaderProgram::createShaderFromString(const GLenum shaderType, const string &code)
{
    // shader
    shared_ptr<Shader> shader = make_shared<Shader>(shaderType);

    // compile the source code
    if(!shader->compile(code)) return false;

    // keep it in the list
    shaders_.push_back(shader);

    return true;
}

void ShaderProgram::addShader(const shared_ptr<Shader> &shader)
{
    // keep it in the list
    shaders_.push_back(shader);
}

// attach and link shaders
bool ShaderProgram::attachAndLinkShaders() const
{
    // for each shader
    for(auto &shader : shaders_)
    {
        // attach it to the shader program
        if(shader) glAttachShader(id_, shader->id());
    }

    // link the shader program
    glLinkProgram(id_);

    // check for the link error
    int success;
    GLchar infoLog[1024];
    glGetProgramiv(id_, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(id_, 1024, NULL, infoLog);
        cout << "[ERROR] Program Shader: Link Failed! " << infoLog << endl;
    }

    return true;
}

// use the shader program
void ShaderProgram::use() const
{
    // use the shader program
    glUseProgram(id_);

    // bind the texture
    if(texture_) texture_->bind();

    // bind the texture units
    GLuint numTextureUnits = 0;
    for(const auto& texture : textureUnits_)
    {
        if (texture)
        {
            // active texture
            glActiveTexture(GL_TEXTURE0 + numTextureUnits);
            texture->bind();

            // next
            numTextureUnits++;
        }
    }
}

// set uniform variables
void ShaderProgram::set(const string& name, const GLint value) const
{
    glUseProgram(id_);
    GLuint location = glGetUniformLocation(id_, name.c_str());
    glUniform1i(location, value);
}

void ShaderProgram::set(const string& name, const GLuint value) const
{
    glUseProgram(id_);
    GLuint location = glGetUniformLocation(id_, name.c_str());
    glUniform1ui(location, value);
}

void ShaderProgram::set(const string& name, const GLfloat value) const
{
    glUseProgram(id_);
    GLuint location = glGetUniformLocation(id_, name.c_str());
    glUniform1f(location, value);
}

void ShaderProgram::set(const string& name, const GLfloat v0, const GLfloat v1) const
{
    glUseProgram(id_);
    GLuint location = glGetUniformLocation(id_, name.c_str());
    glUniform2f(location, v0, v1);
}

void ShaderProgram::set(const string& name, const GLfloat v0, const GLfloat v1, const GLfloat v2) const
{
    glUseProgram(id_);
    GLuint location = glGetUniformLocation(id_, name.c_str());
    glUniform3f(location, v0, v1, v2);
}

void ShaderProgram::set(const string& name, const GLfloat v0, const GLfloat v1, const GLfloat v2, const GLfloat v3) const
{
    glUseProgram(id_);
    GLuint location = glGetUniformLocation(id_, name.c_str());
    glUniform4f(location, v0, v1, v2, v3);
}

void ShaderProgram::set(const string& name, const unsigned int dim, GLsizei count, const GLfloat* value) const
{
    glUseProgram(id_);
    GLuint location = glGetUniformLocation(id_, name.c_str());
    switch (dim)
    {
        case 1: glUniform1fv(location, count, value); break;
        case 2: glUniform2fv(location, count, value); break;
        case 3: glUniform3fv(location, count, value); break;
        case 4: glUniform4fv(location, count, value); break;
    }
}

void ShaderProgram::set(const string& name, const glm::mat4& T) const
{
    glUseProgram(id_);
    GLuint loc = glGetUniformLocation(id_, name.c_str());
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(T));
}

void ShaderProgram::set(const string& name, const glm::mat3x4& T) const
{
    glUseProgram(id_);
    GLuint loc = glGetUniformLocation(id_, name.c_str());
    glUniformMatrix3x4fv(loc, 1, GL_FALSE, glm::value_ptr(T));
}

// texture
void ShaderProgram::setTexture(const shared_ptr<Texture> &texture)
{
    texture_ = texture;
}

bool ShaderProgram::setTexture(const string& filePath, const bool flipVertically)
{
    texture_ = make_shared<Texture>();
    return texture_->loadImage(filePath, flipVertically);
}

bool ShaderProgram::setTexture(const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically)
{
    texture_ = make_shared<Texture>();
    return texture_->loadImage(filePath, internalFormat, format, flipVertically);
}

bool ShaderProgram::addTextureUnit(const string &name, const shared_ptr<Texture> &texture)
{
    // check the number of texture units
    if (textureUnits_.size() >= MAX_NUM_TEXTURE_UNITS_) return false;

    // set the uniform variable
    set(name, static_cast<int>(textureUnits_.size()));

    // keep it
    textureUnits_.push_back(texture);

    return true;
}

bool ShaderProgram::addTextureUnit(const string &name, const string& filePath, const bool flipVertically)
{
    shared_ptr<Texture> texture = make_shared<Texture>();
    if (!texture->loadImage(filePath, flipVertically)) return false;
    return addTextureUnit(name, texture);
}

bool ShaderProgram::addTextureUnit(const string &name, const string& filePath, const GLint internalFormat, const GLenum format, const bool flipVertically)
{
    shared_ptr<Texture> texture = make_shared<Texture>();
    if (!texture->loadImage(filePath, internalFormat, format, flipVertically)) return false;
    return addTextureUnit(name, texture);
}
