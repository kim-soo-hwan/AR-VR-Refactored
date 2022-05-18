#version 330 core

// input variables
layout (location = 0) in vec3 vertexPosition;   // position attribute: C_p
layout (location = 1) in vec2 vertexTexCoords;  // texture coord attribute

// output variables to the fragment shader
out vec2 texCoords;

// uniform variables
uniform mat4 transform; // G_T_C

void main()
{
    // transform each vertex point (matrix-vector multiplication)
    // G_p = G_T_C * C_p
    gl_Position = transform * vec4(vertexPosition, 1.0);

    // set to the texture coords from the vertex input
    texCoords = vertexTexCoords;
} 