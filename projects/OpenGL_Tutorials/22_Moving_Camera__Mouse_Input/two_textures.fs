#version 330 core

// input variables from the vertex shader
in vec2 texCoords;

// output variables
out vec4 fragmentColor;  

// global variables
uniform sampler2D texture1;
uniform sampler2D texture2;

void main()
{
    // mix: x*(1-a) + y*a
    fragmentColor = mix(texture(texture1, texCoords), texture(texture2, texCoords), 0.2);
}