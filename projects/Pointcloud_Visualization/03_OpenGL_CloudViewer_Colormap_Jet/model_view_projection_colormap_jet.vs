#version 330 core

// input variables
layout (location = 0) in vec4 vertexPosition;   // position attribute: O_p

// uniform variables
uniform float minValue;
uniform float maxValue;
uniform mat4 model_view_projection;

// output variables to the fragment shader
out vec3 vertexColor;

// ref) https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
// t in [0, 1]
vec3 jet(float t)
{
  return clamp(vec3(1.5) - abs(4.0 * vec3(t) + vec3(-3, -2, -1)), vec3(0), vec3(1));
}

void main()
{
    // transform each vertex point (matrix-vector multiplication)
    // F_p = F_T_C * C_T_G * G_T_O * O_p
    gl_Position = model_view_projection * vertexPosition;

    // vertex color
    vertexColor = jet((vertexPosition.z - minValue)/(maxValue - minValue));
} 