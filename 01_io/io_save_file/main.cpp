#include <iostream>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //加载文件
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(R"(.\data\bun045.ply)", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    //保存文件
    pcl::io::savePCDFileASCII(u8"test_pcd.pcd", *cloud);

    //重新加载文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(u8"test_pcd.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    //显示图像
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }

    return 0;
}
