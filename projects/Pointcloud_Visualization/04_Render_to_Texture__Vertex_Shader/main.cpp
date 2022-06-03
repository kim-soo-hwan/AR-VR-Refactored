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
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("000000.pcd", *cloud) == -1) //* load the file
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
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "multiple_model_view_projections_pointSize.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "projected_color.fs");
    shaderProgram->attachAndLinkShaders();

    // capturing camera
    Camera capturingCamera;

    // camera 0
    // capturingCamera.setTransformationMatrixforMobileTech(-0.010250,0.010125,0.000500, -0.084300,-0.061250,-0.129400);
    capturingCamera.resetTransformationMatrix();
    capturingCamera.translate_L(-3.f, 0.f, 0.f);
    shared_ptr<Axes> axes_00 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // G_T_C
    capturingCamera.inverseTransformationMatrix(); // C_T_G: view matrix
    capturingCamera.setProjectionMatrix(1066.199336,1066.132909,999.624838,600.302905, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_0", capturingCamera.getViewProjectionMatrix());

    // camera 1
    // capturingCamera.setTransformationMatrixforMobileTech(-5.028656,-0.012226,-0.001438, -0.022350,0.150200,-0.129400);
    capturingCamera.resetTransformationMatrix();
    capturingCamera.rotateAboutZ_L(glm::radians(90.f));
    capturingCamera.translate_L(0.f, -3.f, 0.f);
    shared_ptr<Axes> axes_01 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // G_T_C
    capturingCamera.inverseTransformationMatrix(); // C_T_G: view matrix
    capturingCamera.setProjectionMatrix(1062.135424,1063.059400,992.146065,628.093140, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_1", capturingCamera.getViewProjectionMatrix());

    // camera 2
    capturingCamera.setTransformationMatrixforMobileTech(-3.771093,-0.001000,-0.003625, 0.140700,0.059600,-0.129400);
    shared_ptr<Axes> axes_02 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // G_T_C
    capturingCamera.inverseTransformationMatrix(); // C_T_G: view matrix
    capturingCamera.setProjectionMatrix(1066.215109,1066.475016,992.281252,608.039672, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_2", capturingCamera.getViewProjectionMatrix());

    // camera 3
    capturingCamera.setTransformationMatrixforMobileTech(-2.530940,-0.011250,0.006375, 0.108200,-0.049600,-0.129400);
    shared_ptr<Axes> axes_03 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // G_T_C
    capturingCamera.inverseTransformationMatrix(); // C_T_G: view matrix
    capturingCamera.setProjectionMatrix(1060.315354,1061.005324,945.352784,617.652102, 0.1f, 10000.f, 1920.f, 1200.f);
    shaderProgram->set("model_view_projection_3", capturingCamera.getViewProjectionMatrix());

    // camera 4
    capturingCamera.setTransformationMatrixforMobileTech(-7.551531,-0.001187,-0.002250, -0.026100,-0.080200,-0.129400);
    shared_ptr<Axes> axes_04 = make_shared<Axes>(capturingCamera.getTransformationMatrix()); // G_T_C
    capturingCamera.inverseTransformationMatrix(); // C_T_G: view matrix
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
    pointCloud->setShaderProgram(shaderProgram);
    pointCloud->setModelViewProjectionMatrixName("model_view_projection");

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 10000.f);
    camera->setTransformationMatrix(0.888122, 0.458937, 0.0248475, -1.11125e-05, 
                                   -0.187117, 0.311666, 0.931587, -1.13249e-05,
                                    0.419795, -0.832011, 0.362673, -64.0001);
    window.setCamera(camera);

    // scene
    Scene scene;
    scene.addModel(pointCloud);
    scene.setCamera(camera);
    scene.setGlobalAxes(10.f);
    scene.addAxes(axes_00);
    scene.addAxes(axes_01);
    // scene.addAxes(axes_02);
    // scene.addAxes(axes_03);
    // scene.addAxes(axes_04);

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