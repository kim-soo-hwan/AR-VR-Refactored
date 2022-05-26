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
    cout << "Loaded " << cloud->width << " x " << cloud->height << " data points from test_pcd.pcd with the following fields: " << endl;


    // visualization

    // OpenGL window
    Window window(800, 600, "Pointcloud Visualization");
    window.setBackgroundColor(0.f, 0.f, 0.f, 1.0f);
    window.setDepthEnabled();

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "multiple_model_view_projections.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "projected_color.fs");
    shaderProgram->attachAndLinkShaders();

    // capturing camera
    Camera capturingCamera;

    // camera 0: euler angles in radian?
    capturingCamera.transform(4.694164,0.258938,0.008250, 0.000000,-0.206387,-0.150950);
    capturingCamera.setProjectionMatrix(1066.199336,1066.132909,999.624838,600.302905, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_0", capturingCamera.getViewProjectionMatrix());

    // camera 1
    capturingCamera.transform(-2.843782,0.077074,0.242250, 0.092252,-0.139361,-0.150950);
    capturingCamera.setProjectionMatrix(1062.135424,1063.059400,992.146065,628.093140, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_1", capturingCamera.getViewProjectionMatrix());

    // camera 2
    capturingCamera.transform(2.210281,-0.209859,0.147563, 0.057015,0.030912,-0.150950);
    capturingCamera.setProjectionMatrix(1066.215109,1066.475016,992.281252,608.039672, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_2", capturingCamera.getViewProjectionMatrix());

    // camera 3
    capturingCamera.transform(-5.367000,-0.209039,-0.154023, -0.057015,0.030912,-0.150950);
    capturingCamera.setProjectionMatrix(1060.315354,1061.005324,945.352784,617.652102, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_3", capturingCamera.getViewProjectionMatrix());

    // camera 4
    capturingCamera.transform(-0.321687,0.084313,-0.241000, -0.092252,-0.139361,-0.150950);
    capturingCamera.setProjectionMatrix(1068.730240,1068.356202,979.756477,624.191779, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_4", capturingCamera.getViewProjectionMatrix());

    // texture
    shaderProgram->addTextureUnit("image_0", "0.jpg");
    shaderProgram->addTextureUnit("image_1", "1.jpg");
    shaderProgram->addTextureUnit("image_2", "2.jpg");
    shaderProgram->addTextureUnit("image_3", "3.jpg");
    shaderProgram->addTextureUnit("image_4", "4.jpg");

    // vertex input: O_p

    // model
    shared_ptr<Model> pointCloud = make_shared<Model>(cloud->size());
    pointCloud->setVertexAttributes((GLfloat *)(cloud->data()), 4);
    pointCloud->setDrawMode(GL_POINTS);
    //pointCloud->setPointSize(10);
    pointCloud->setShaderProgram(shaderProgram);
    pointCloud->setModelViewProjectionMatrixName("model_view_projection");

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 10000.f);
    // camera->setViewMatrix(0.905129, 0.423663, 0.0352294, 
    //                      -0.0927226, 0.115865, 0.988919,
    //                       0.414886, -0.898373, 0.144155, 
    //                       -0.120035, 0.9758, -69.8173);
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