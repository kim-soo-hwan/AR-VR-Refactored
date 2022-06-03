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
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("MobilTech_01/000000001.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file 000000001.pcd \n");
        return (-1);
    }
    cout << "Loaded " << cloud->width << " x " << cloud->height << " data points from test_pcd.pcd with the following fields: " << endl;


    // visualization

    // OpenGL window
    Window window(1920, 1200, "Pointcloud Visualization");
    window.setBackgroundColor(0.f, 0.f, 0.f, 1.0f);
    window.enable(GL_DEPTH_TEST);
    window.enable(GL_PROGRAM_POINT_SIZE); // for gl_PointSize in vertex shader

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "multiple_model_view_distorted_projections_pointSize.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "projected_color.fs");
    shaderProgram->attachAndLinkShaders();

    // capturing camera
    Camera capturingCamera;

    // camera 0
    capturingCamera.setViewMatrixForMobilTech(4.694164,0.258938,0.008250, 0.000000,-0.206387,-0.150950);
    shaderProgram->set("model_view_0", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_0", glm::mat3x4(1066.199336,1066.132909,999.624838,600.302905,-0.150919,0.080171,0.000161,0.000209, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_00 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_00->inverseTransformationMatrix(); // G_T_C

    // camera 1
    capturingCamera.setViewMatrixForMobilTech(-2.843782,0.077074,0.242250, 0.092252,-0.139361,-0.150950);
    shaderProgram->set("model_view_1", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_1", glm::mat3x4(1062.135424,1063.059400,992.146065,628.093140,-0.152023,0.084686,0.001038,-0.000576, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_01 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_01->inverseTransformationMatrix(); // G_T_C

    // camera 2
    capturingCamera.setViewMatrixForMobilTech(2.210281,-0.209859,0.147563, 0.057015,0.030912,-0.150950);
    shaderProgram->set("model_view_2", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_2", glm::mat3x4(1066.215109,1066.475016,992.281252,608.039672,-0.151921,0.079388,0.000004,-0.0010022, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_02 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_02->inverseTransformationMatrix(); // G_T_C

    // camera 3
    capturingCamera.setViewMatrixForMobilTech(-5.367000,-0.209039,-0.154023, -0.057015,0.030912,-0.150950);
    shaderProgram->set("model_view_3", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_3", glm::mat3x4(1060.315354,1061.005324,945.352784,617.652102,-0.153736,0.091515,-0.000025,-0.000263, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_03 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_03->inverseTransformationMatrix(); // G_T_C

    // camera 4
    capturingCamera.setViewMatrixForMobilTech(-0.321687,0.084313,-0.241000, -0.092252,-0.139361,-0.150950);
    shaderProgram->set("model_view_4", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_4", glm::mat3x4(1068.730240,1068.356202,979.756477,624.191779,-0.153923,0.081875,-0.000038,-0.000230, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_04 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_04->inverseTransformationMatrix(); // G_T_C

    // texture
    shaderProgram->addTextureUnit("image_0", "MobilTech_01/0.jpg");
    shaderProgram->addTextureUnit("image_1", "MobilTech_01/1.jpg");
    shaderProgram->addTextureUnit("image_2", "MobilTech_01/2.jpg");
    shaderProgram->addTextureUnit("image_3", "MobilTech_01/3.jpg");
    shaderProgram->addTextureUnit("image_4", "MobilTech_01/4.jpg");

    // vertex input: O_p

    // model
    shared_ptr<Model> pointCloud = make_shared<Model>(cloud->size());
    pointCloud->setVertexAttributes((GLfloat *)(cloud->data()), 4);
    pointCloud->setDrawMode(GL_POINTS);
    pointCloud->setShaderProgram(shaderProgram);
    pointCloud->setModelViewProjectionMatrixName("model_view_projection");

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 10000.f);
    camera->setTransformationMatrix(0.299295, 0.934426, 0.193069, 0.459999, 
                                   -0.397038, -0.0620311, 0.915705, -1.65997,
                                    0.867634, -0.350721, 0.352437, -20);
    window.setCamera(camera);

    // scene
    Scene scene;
    scene.addModel(pointCloud);
    scene.setCamera(camera);
    scene.setGlobalAxes(10.f);
    scene.addAxes(axes_00);
    scene.addAxes(axes_01);
    scene.addAxes(axes_02);
    scene.addAxes(axes_03);
    scene.addAxes(axes_04);

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