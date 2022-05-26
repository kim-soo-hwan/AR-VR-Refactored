#version 430 core

// input variables
layout (location = 0) in vec3 vertexPosition;   // position attribute: O_p
layout (location = 1) in vec2 vertexTexCoords;  // texture coord attribute

// output variables to the fragment shader
out vec2 texCoords;

// uniform variables
//uniform mat4 model;
//uniform mat4 view;
//uniform mat4 projection;
uniform mat4 model_view_projection;

void main()
{
    // transform each vertex point (matrix-vector multiplication)
    // F_p = NDC_T_C * C_T_G * G_T_O * O_p
    //gl_Position = projection * view * model * vec4(vertexPosition, 1.0);
    gl_Position = model_view_projection * vec4(vertexPosition, 1.0);

    // set to the texture coords from the vertex input
    texCoords = vertexTexCoords;
} 