#ifndef __UTILITY_H__
#define __UTILITY_H__

// std
#include <string>
using namespace std;

// OpenGL
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>

// shader program
GLuint generateAndUseDefaultShaderProgram();
void deleteDefaultShaderProgram(const GLuint shaderProgram);

#endif // __UTILITY_H__