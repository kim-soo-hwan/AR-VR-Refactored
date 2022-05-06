#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <glad/glad.h>
#include <GLFW/glfw3.h>

GLuint generateAndUseDefaultShaderProgram();
void deleteDefaultShaderProgram(const GLuint shaderProgram);

#endif // __UTILITY_H__