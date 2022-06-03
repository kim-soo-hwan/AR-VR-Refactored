#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Core>



int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << argv[0] << "Syntax is : [img file path] [lidar file path]" << std::endl;
        return (-1);
    }

    std::string img_path = std::string(argv[1]);
    std::string lidar_path = std::string(argv[2]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_path, *lidar) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }

    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
    if (img.empty())
    {
     std::cout << "Could not read the image: " << img_path << std::endl;
        return (-1);   
    }

    int points_size = lidar->points.size();
    std::cout << " total points : " << points_size << std::endl;

    double fx = 1066.199336;
    double fy = 1066.132909;
    double cx = 999.624838;
    double cy = 600.302905;
    double k1 = -0.150919;
    double k2 = 0.080171;
    double p1 = 0.000161;
    double p2 = 0.000209;
    

    Eigen::Matrix<double, 3, 3> rotationMatrix;

    rotationMatrix << 0, 1, 0, 0, 0, -1, -1, 0, 0;

    Eigen::Matrix3d rotLR = Eigen::AngleAxisd(4.694164, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d rotUD = Eigen::AngleAxisd(0.258938, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rotRot = Eigen::AngleAxisd(0.008250, Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    rotationMatrix = rotationMatrix * rotRot * rotUD * rotLR;  //3*3
    Eigen::Matrix<double, 3, 1> translationMatrix;
    translationMatrix << 0.000000,-0.206387,-0.150950;


    for (int i=0;i <  points_size; i++)
    {
        Eigen::Matrix<double, 3, 1> point_l, pointC;
        point_l << lidar->points[i].x, lidar->points[i].y, lidar->points[i].z;
        // point_l = LiDAR_CORRECTION * point_l;
        pointC = rotationMatrix * (point_l - translationMatrix);

        double xp = pointC(0) / pointC(2);
        double yp = pointC(1) / pointC(2);

        double r_sq = xp*xp + yp*yp;
        double ratio = (1 + k1*r_sq + k2*r_sq*r_sq ); 
        double xpp = xp * ratio + 2*p1*xp*yp + p2*(r_sq+2*xp*xp);
        double ypp = yp * ratio + p1 *(r_sq + 2*yp*yp) + 2*p2*xp*yp;
        int camX = fx * xpp + cx;
        int camY = fy * ypp + cy;
    
        if (camX >= 0 && camX < img.cols && camY >= 0 && camY < img.rows && pointC(2, 0) > 0)
        {
            // img.at<cv::Vec3b>(cv::Point(camX, camY)) = (255,0,0);
            cv::circle(img, cv::Point(camX, camY), 3, (255,0,0), -1);
        }
    }

    cv::imshow("Display window", img);
    int k = cv::waitKey(0);
}

