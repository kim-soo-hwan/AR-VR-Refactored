#version 330 core

// input variables
layout (location = 0) in vec3 vertexPosition;   // position attribute

// output variables
out vec4 vertexColor;

void main()
{
    // position in homogenous coordinates
    gl_Position = vec4(vertexPosition, 1.0);

    // set the output variable to a dark-red color
    vertexColor = vec4(0.5, 0.0, 0.0, 1.0);
} 