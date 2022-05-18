#version 330 core

// input variables from the vertex shader
in vec2 texCoords;

// output variables
out vec4 fragmentColor;  

// global variables
uniform sampler2D ourTexture;

void main()
{
    fragmentColor = texture(ourTexture, texCoords);
}