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

    // point size
    //        gl_Position.z        : [  -1,  1]
    //        gl_Position.z + 2.f  : [   1,  3]
    //  3.f / (gl_Position.z + 2.f): [   1,  3]
    // 30.f / (gl_Position.z + 2.f): [  10, 30]
    gl_PointSize = 30.f / (gl_Position.z + 2.f);
} 