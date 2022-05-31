#version 430 core

// input variables from the vertex shader
flat in int  imageNum;
in vec2 texCoords;

// output variables
out vec4 fragmentColor;  

// global variables
uniform sampler2D image_0; // captured by camera 0
uniform sampler2D image_1; // captured by camera 1
uniform sampler2D image_2; // captured by camera 2
uniform sampler2D image_3; // captured by camera 3
uniform sampler2D image_4; // captured by camera 4

void main()
{
    // // for each camera, colorize the point
    // if (imageNum == 0) { fragmentColor = texture(image_0, texCoords); return; }
    // if (imageNum == 1) { fragmentColor = texture(image_1, texCoords); return; }
    // if (imageNum == 2) { fragmentColor = texture(image_2, texCoords); return; }
    // if (imageNum == 3) { fragmentColor = texture(image_3, texCoords); return; }
    // if (imageNum == 4) { fragmentColor = texture(image_4, texCoords); return; }

    // // else, do not draw the point
    // discard;

    // for each camera, colorize the point
    switch(imageNum)
    {
        case 0:  fragmentColor = vec4(1.f, 0.f, 0.f, 1.f); break; // R
        case 1:  fragmentColor = vec4(0.f, 1.f, 0.f, 1.f); break; // G
        case 2:  fragmentColor = vec4(0.f, 0.f, 1.f, 1.f); break; // B
        case 3:  fragmentColor = vec4(1.f, 1.f, 0.f, 1.f); break; // Y
        case 4:  fragmentColor = vec4(1.f, 0.f, 1.f, 1.f); break; // M
        default: fragmentColor = vec4(1.f, 1.f, 1.f, 1.f);        // W
    }

    // if (imageNum == 0) { fragmentColor = vec4(1.f, 0.f, 0.f, 1.f); return; } // R
    // if (imageNum == 1) { fragmentColor = vec4(0.f, 1.f, 0.f, 1.f); return; } // G
    // if (imageNum == 2) { fragmentColor = vec4(0.f, 0.f, 1.f, 1.f); return; } // B
    // if (imageNum == 3) { fragmentColor = vec4(1.f, 1.f, 0.f, 1.f); return; } // Y
    // if (imageNum == 4) { fragmentColor = vec4(1.f, 0.f, 1.f, 1.f); return; } // M

    // fragmentColor = vec4(1.f, 1.f, 1.f, 1.f); // W
    // return;
}