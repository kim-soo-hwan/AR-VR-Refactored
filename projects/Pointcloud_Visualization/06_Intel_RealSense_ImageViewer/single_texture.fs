#version 430 core

// input variables from the vertex shader
in vec2 texCoords;

// output variables
out vec4 fragmentColor;  

// global variables
uniform sampler2D ourTexture;

void main()
{
    // mix: x*(1-a) + y*a
    fragmentColor = texture(ourTexture, texCoords);
}