// std
#include <iostream>
using namespace std;

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("MobilTech_01/000000001.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file 000000001.pcd \n");
        return (-1);
    }

    cout << "Loaded "
         << cloud->width * cloud->height
         << " data points from test_pcd.pcd with the following fields: "
         << endl;

    // visualization
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);
    while(!viewer.wasStopped())
    {
    }
    return (0);
}