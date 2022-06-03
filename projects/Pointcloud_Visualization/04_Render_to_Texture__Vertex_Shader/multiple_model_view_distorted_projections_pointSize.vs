#version 430 core

// input variables
layout (location = 0) in vec4 vertexPosition;   // position attribute: O_p

// uniform variables
uniform mat4 model_view_projection;   // current camera
uniform mat4 model_view_0; // capture camera 0
uniform mat4 model_view_1; // capture camera 1
uniform mat4 model_view_2; // capture camera 2
uniform mat4 model_view_3; // capture camera 3
uniform mat4 model_view_4; // capture camera 4

// intrinsic parameters
// [fx, k1, width, 
//  fy, k2, height, 
//  cx, p1, min_depth
//  cy, p2, ?]
uniform mat3x4 intrinsics_0; // capture camera 0
uniform mat3x4 intrinsics_1; // capture camera 1
uniform mat3x4 intrinsics_2; // capture camera 2
uniform mat3x4 intrinsics_3; // capture camera 3
uniform mat3x4 intrinsics_4; // capture camera 4

// output variables to the fragment shader
flat out int  imageNum;
out vec2 texCoords;

// ref) https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
bool texture_coord(mat4 model_view, mat3x4 intrinsics)
{
    // C_p = C_T_G * G_T_O * O_p
    vec4 C_p = model_view * vertexPosition;
    
    // depth
    const float min_depth = intrinsics[2][2];
    const float z = C_p.z;
    if (z < min_depth) return false;

    // intrinsic parameters
    // [fx, k1, width, 
    //  fy, k2, height, 
    //  cx, p1, min_depth
    //  cy, p2, ?]
    const float fx = intrinsics[0][0];
    const float fy = intrinsics[0][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[0][3];
    const float k1 = intrinsics[1][0];
    const float k2 = intrinsics[1][1];
    const float p1 = intrinsics[1][2];
    const float p2 = intrinsics[1][3];
    const float width  = intrinsics[2][0];
    const float height = intrinsics[2][1];

    // projection
    const float x = C_p.x / z;
    const float y = C_p.y / z;

    // distortion
    const float xx = x*x;       // x^2
    const float yy = y*y;       // y^2
    const float xy = x*y;       // xy
    const float rr = xx + yy;   // r^2

    const float ratio = (1 + k1*rr + k2*rr*rr); 
    const float x_distorted = x * ratio + 2*p1*xy + p2*(rr+2*xx);
    const float y_distorted = y * ratio + p1*(rr+2*yy) + 2*p2*xy;

    // scale and shift
    const float u = fx * x_distorted + cx;
    const float v = fy * y_distorted + cy;

    // check if the point is projected on to the image
    if (u < 0 || u > width || v < 0 || v > height) return false;

    // texture coordinates
    texCoords.x = u / width;
    texCoords.y = 1.f - v / height;

    return true;
}

void main()
{
    // for current camera
    // transform each vertex point (matrix-vector multiplication)
    // F_p = NDC_T_C * C_T_G * G_T_O * O_p
    gl_Position = model_view_projection * vertexPosition;

    // point size
    //          gl_Position.z       : [  -1,  1]
    //          gl_Position.z + 2.f : [   1,  3]
    //   3.f / (gl_Position.z + 2.f): [   1,  3]
    // 100.f / (gl_Position.z + 2.f): [  10, 30]
    gl_PointSize = 80.f / (gl_Position.z + 2.f);

    // for each camera\
    imageNum = -1; // default value
    if (texture_coord(model_view_0, intrinsics_0)) { imageNum = 0; return; }
    if (texture_coord(model_view_1, intrinsics_1)) { imageNum = 1; return; }
    if (texture_coord(model_view_2, intrinsics_2)) { imageNum = 2; return; }
    if (texture_coord(model_view_3, intrinsics_3)) { imageNum = 3; return; }
    if (texture_coord(model_view_4, intrinsics_4)) { imageNum = 4; return; }
} 