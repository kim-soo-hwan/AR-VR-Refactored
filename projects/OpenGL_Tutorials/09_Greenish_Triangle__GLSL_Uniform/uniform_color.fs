#version 430 core

// output variables
out vec4 fragmentColor;  

// we set this variable in the OpenGL code
uniform vec4 ourColor;

void main()
{
    fragmentColor = ourColor;
}