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
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("MobilTech_02/000000.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file 000000.pcd \n");
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
    capturingCamera.setViewMatrixForMobilTech(-0.010250,0.010125,0.000500, -0.084300,-0.061250,-0.129400);
    shaderProgram->set("model_view_0", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_0", glm::mat3x4(1068.824000,1065.331000,975.406000,600.000000,-0.148948,0.067679,0.000019,0.000028, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_00 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_00->inverseTransformationMatrix(); // G_T_C

    // camera 1
    capturingCamera.setViewMatrixForMobilTech(-5.028656,-0.012226,-0.001438, -0.022350,0.150200,-0.129400);
    capturingCamera.setProjectionMatrix(1062.135424,1063.059400,992.146065,628.093140, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_1", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_1", glm::mat3x4(1068.266188,1057.231000,973.556000,600.000000,-0.140354,0.067679,0.000019,0.000028, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_01 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_01->inverseTransformationMatrix(); // G_T_C

    // camera 2
    capturingCamera.setViewMatrixForMobilTech(-3.771093,-0.001000,-0.003625, 0.140700,0.059600,-0.129400);
    capturingCamera.setProjectionMatrix(1066.215109,1066.475016,992.281252,608.039672, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_2", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_2", glm::mat3x4(1063.533375,1062.506000,988.906000,600.000000,-0.148948,0.067679,0.000019,0.000028, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_02 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_02->inverseTransformationMatrix(); // G_T_C

    // camera 3
    capturingCamera.setViewMatrixForMobilTech(-2.530940,-0.011250,0.006375, 0.108200,-0.049600,-0.129400);
    capturingCamera.setProjectionMatrix(1060.315354,1061.005324,945.352784,617.652102, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_3", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_3", glm::mat3x4(1055.499000,1060.981000,964.606000,600.000000,-0.136448,0.072366,0.000019,0.000028, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_03 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_03->inverseTransformationMatrix(); // G_T_C

    // camera 4
    capturingCamera.setViewMatrixForMobilTech(-7.551531,-0.001187,-0.002250, -0.026100,-0.080200,-0.129400);
    capturingCamera.setProjectionMatrix(1068.730240,1068.356202,979.756477,624.191779, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_4", capturingCamera.getViewProjectionMatrix());
    shaderProgram->set("intrinsics_4", glm::mat3x4(1073.874000,1062.906000,963.406000,600.000000,-0.173948,0.089554,0.000019,0.000028, 1920, 1200, 1, 0));

    shared_ptr<Axes> axes_04 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // C_T_G: view matrix
    axes_04->inverseTransformationMatrix(); // G_T_C

    // texture
    shaderProgram->addTextureUnit("image_0", "MobilTech_02/0.jpg");
    shaderProgram->addTextureUnit("image_1", "MobilTech_02/1.jpg");
    shaderProgram->addTextureUnit("image_2", "MobilTech_02/2.jpg");
    shaderProgram->addTextureUnit("image_3", "MobilTech_02/3.jpg");
    shaderProgram->addTextureUnit("image_4", "MobilTech_02/4.jpg");

    // vertex input: O_p

    // model
    shared_ptr<Model> pointCloud = make_shared<Model>(cloud->size());
    pointCloud->setVertexAttributes((GLfloat *)(cloud->data()), 4);
    pointCloud->setDrawMode(GL_POINTS);
    pointCloud->setShaderProgram(shaderProgram);
    pointCloud->setModelViewProjectionMatrixName("model_view_projection");

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 10000.f);
    camera->setTransformationMatrix(0.689583,  0.718777, 0.0885293, 1.43387e-05, 
                                   -0.516173,  0.402065, 0.75625,   7.77542e-05,
                                    0.507979, -0.567191, 0.64827,  -2.99995);
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