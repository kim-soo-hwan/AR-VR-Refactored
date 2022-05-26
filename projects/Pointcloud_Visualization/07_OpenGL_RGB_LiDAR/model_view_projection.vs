#version 430 core

// input variables
layout (location = 0) in vec4 vertexPosition;   // position attribute: O_p

// uniform variables
uniform mat4 model_view_projection;

void main()
{
    // transform each vertex point (matrix-vector multiplication)
    // F_p = NDC_T_C * C_T_G * G_T_O * O_p
    gl_Position = model_view_projection * vertexPosition;
} 