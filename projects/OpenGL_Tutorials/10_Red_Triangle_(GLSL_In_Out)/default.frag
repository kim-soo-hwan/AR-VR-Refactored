#version 330 core

// the input variable from the vertex shader (same name and same type)
in vec4 vertexColor;

// output variables
out vec4 fragmentColor;  

void main()
{
    fragmentColor = vertexColor;
}