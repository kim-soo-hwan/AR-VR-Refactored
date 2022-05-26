#version 430 core

// input variables
in vec3 ourColor;

// output variables
out vec4 fragmentColor;  

void main()
{
    fragmentColor = vec4(ourColor, 1.0);
}