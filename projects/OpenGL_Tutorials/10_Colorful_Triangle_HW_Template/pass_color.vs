#version 430 core

// input variables
layout (location = 0) in vec3 vertexPosition;   // position attribute
layout (location = 1) in vec3 vertexColor;      // color attribute

// output variables
out vec3 ourColor;

void main()
{
    // position in homogenous coordinates
    gl_Position = vec4(vertexPosition, 1.0);

    // set to the input color from the vertex data
    ourColor = vertexColor;
} 