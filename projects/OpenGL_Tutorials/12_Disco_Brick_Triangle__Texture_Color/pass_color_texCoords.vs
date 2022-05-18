#version 330 core

// input variables
layout (location = 0) in vec3 vertexPosition;      // position attribute
layout (location = 1) in vec3 vertexColor;    // color attribute
layout (location = 2) in vec2 vertexTexCoords; // texture coord attribute

// output variables to the fragment shader
out vec3 ourColor;
out vec2 texCoords;

void main()
{
    // position in homogenous coordinates
    gl_Position = vec4(vertexPosition, 1.0);

    // set to the input color from the vertex input
    ourColor = vertexColor;

    // set to the texture coords from the vertex input
    texCoords = vertexTexCoords;
} 