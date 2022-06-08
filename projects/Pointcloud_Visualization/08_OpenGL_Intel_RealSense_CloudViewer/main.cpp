// std
#include <iostream>
using namespace std;

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

// Intel RealSense
#include <librealsense2/rs.hpp>

// swan
#include <window.h>
#include <mesh.h>
#include <shader.h>
#include <texture.h>
#include <model.h>
#include <scene.h>
#include <camera.h>
#include <axes.h>

// ref) https://dev.intelrealsense.com/docs/pcl-wrapper
// ref) https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl-color/rs-pcl-color.cpp

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
// void PCL_Conversion(const rs2::points& points, const rs2::video_frame& frame,
//                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
//                     pcl::PointCloud<pcl::RGB>::Ptr &color)
void PCL_Conversion(const rs2::points& points, const rs2::video_frame& frame,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &color)
{
    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    color->width  = static_cast<uint32_t>( sp.width()  );   
    color->height = static_cast<uint32_t>( sp.height() );
    color->is_dense = false;
    color->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(frame, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        // color->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        // color->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        // color->points[i].b = get<0>(RGB_Color); // Reference tuple<0>
        color->points[i].x = get<2>(RGB_Color) / 255.f; // Reference tuple<2>
        color->points[i].y = get<1>(RGB_Color) / 255.f; // Reference tuple<1>
        color->points[i].z = get<0>(RGB_Color) / 255.f; // Reference tuple<0>
    }
}

int main() try
{ 
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Stream configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    // start streaming with the custom configuration
    rs2::pipeline_profile selection = pipe.start(cfg); 

    // device setting
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }
    
    // visualization

    // OpenGL window
    Window window(800, 600, "Pointcloud Visualization");
    window.setBackgroundColor(0.f, 0.f, 0.f, 1.0f);
    window.enable(GL_DEPTH_TEST);
    window.enable(GL_PROGRAM_POINT_SIZE); // for gl_PointSize in vertex shader

    // shader program
    shared_ptr<ShaderProgram> shaderProgram = make_shared<ShaderProgram>();
    shaderProgram->createShaderFromFile(GL_VERTEX_SHADER,   "model_view_projection_pointSize_PCL_XYZRGB.vs");
    shaderProgram->createShaderFromFile(GL_FRAGMENT_SHADER, "vertex_color.fs");
    shaderProgram->attachAndLinkShaders();

    // camera
    shared_ptr<Camera> camera = make_shared<Camera>(45.f, window.getRatio(), 0.1f, 10000.f);
    camera->setTransformationMatrix(0.939393, 0.116092, 0.322596, -0.160007, 
                                    0.0487825, -0.976615, 0.209399, -0.389943,
                                    0.339361, -0.180972, -0.923086, -1.99998);
    window.setCamera(camera);

    // Loop and take frame captures upon user input
    while(!window.shouldClose())
    {
        // Capture a single frame and obtain depth + RGB values from it    
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto RGB = frames.get_color_frame();

        // Map Color texture to each point
        pc.map_to(RGB);

        // Generate Point Cloud
        auto points = pc.calculate(depth);

        // Convert generated Point Cloud to PCL Formatting
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
     // pcl::PointCloud<pcl::RGB>::Ptr      color(new pcl::PointCloud<pcl::RGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr color(new pcl::PointCloud<pcl::PointXYZ>);
        PCL_Conversion(points, RGB, cloud, color);

        // show

        // model
        shared_ptr<Model> pointCloud = make_shared<Model>(cloud->size());
        pointCloud->setVertexAttributes((GLfloat *)(cloud->data()), 4);
        pointCloud->setVertexAttributes((GLfloat *)(color->data()), 4);
        pointCloud->setDrawMode(GL_POINTS);
        pointCloud->setShaderProgram(shaderProgram);
        pointCloud->setModelViewProjectionMatrixName("model_view_projection");

        // scene
        Scene scene;
        scene.addModel(pointCloud);
        scene.setCamera(camera);
        scene.setGlobalAxes(10.f);

        // wipe out
        window.wipeOut();

        // draw        
        scene.draw();

        // display
        window.display();

        // user inputs
        window.processUserInputs();        
    }
   
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}