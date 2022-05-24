// std
#include <iostream>
using namespace std;

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// swan
#include <window.h>
#include <mesh.h>
#include <shader.h>
#include <texture.h>
#include <model.h>
#include <scene.h>
#include <camera.h>
#include <axes.h>

int main()
{
    // PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("000000001.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file 000000001.pcd \n");
        return (-1);
    }
    cout << "Loaded " << cloud->width << " x " << cloud->height
         << " data points from test_pcd.pcd with the following fields: "
         << endl;

    // visualization

    // OpenGL window
    Window window(800, 600, "Pointcloud Visualization");
    window.setBackgroundColor(0.f, 0.f, 0.f, 1.0f);
    window.setDepthEnabled();

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "model_view_projection.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "white.fs");
    shaderProgram->attachAndLinkShaders();

    // vertex input: O_p

    // model
    shared_ptr<Model> pointCloud = make_shared<Model>(cloud->size());
    pointCloud->setVertexAttributes((GLfloat *)(cloud->data()), 4);
    pointCloud->setDrawMode(GL_POINTS);
    pointCloud->setShaderProgram(shaderProgram);
    pointCloud->setModelViewProjectionMatrixName("model_view_projection");

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 10000.f);
    camera->setViewMatrix(0.905129, 0.423663, 0.0352294, 
                         -0.0927226, 0.115865, 0.988919,
                          0.414886, -0.898373, 0.144155, 
                          -0.120035, 0.9758, -69.8173);
    window.setCamera(camera);

    // scene
    Scene scene;
    scene.addModel(pointCloud);
    scene.setCamera(camera);
    scene.setAxes(10.f);

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // draw        
        scene.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();
    }

    return 0;
}