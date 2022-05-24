#version 330 core


// input variables
in vec3 vertexColor;

// output variables
out vec4 fragmentColor;  

void main()
{
    // fixed color
    fragmentColor = vec4(vertexColor, 1.f);
}