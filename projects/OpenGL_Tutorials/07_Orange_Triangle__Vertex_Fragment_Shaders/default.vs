#version 330 core

// input variables
layout (location = 0) in vec3 vertexPosition;   // position attribute

void main()
{
    // position in homogenous coordinates
    gl_Position = vec4(vertexPosition, 1.0);
} 