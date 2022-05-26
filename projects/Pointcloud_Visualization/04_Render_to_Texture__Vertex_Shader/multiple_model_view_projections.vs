#version 430 core

// input variables
layout (location = 0) in vec4 vertexPosition;   // position attribute: O_p

// uniform variables
uniform mat4 model_view_projection;   // current camera
uniform mat4 model_view_projection_0; // capture camera 0
uniform mat4 model_view_projection_1; // capture camera 1
uniform mat4 model_view_projection_2; // capture camera 2
uniform mat4 model_view_projection_3; // capture camera 3
uniform mat4 model_view_projection_4; // capture camera 4

// output variables to the fragment shader
out int  imageNum;
out vec2 texCoords;

bool texture_coord(mat4 mvp)
{
    vec4 ndcCoords = mvp * vertexPosition; // in NDC

    // texture vs. screen coordinates
    //                    gl_Position.xy / gl_Position.w   : x = [-1, 1], y = [-1, 1]
    //               (1 + gl_Position.xy / gl_Position.w)/2: x = [ 0, 1], y = [ 0, 1]: texture coordinates
    // viewport.wh * (1 + gl_Position.xy / gl_Position.w)/2: x = [ 0, w], y = [ 0, h]: screen coordinates
    texCoords = (1.f + ndcCoords.xy / ndcCoords.w)/2.f;

    // check if the point is projected on to the image
    return (texCoords.x > 0 && texCoords.x < 1 && texCoords.y > 0 && texCoords.y < 1);
}

void main()
{
    // for current camera
    // transform each vertex point (matrix-vector multiplication)
    // F_p = NDC_T_C * C_T_G * G_T_O * O_p
    gl_Position = model_view_projection * vertexPosition;

    // for each camera
    if (texture_coord(model_view_projection_0)) { imageNum = 0; return; }
    if (texture_coord(model_view_projection_1)) { imageNum = 1; return; }
    if (texture_coord(model_view_projection_2)) { imageNum = 2; return; }
    if (texture_coord(model_view_projection_3)) { imageNum = 3; return; }
    if (texture_coord(model_view_projection_4)) { imageNum = 4; return; }
} 