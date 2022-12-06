#include <iostream>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // 点云坐标变换
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h> //KD树

//This is the main function
int main()
{
    //用系统时间初始化随机种子
    srand(time(nullptr));

    //创建一个PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //生成随机点云
    cloud->width = 1000; //此处点云数量
    cloud->height = 1;   //表示点云为无序点云
    cloud->points.resize(cloud->width * cloud->height);
    for (auto& point : cloud->points) //循环填充点云数据
    {
        point.x = 1024.0f * (float)rand() / (RAND_MAX + 1.0f); //// 产生数值为0-1024的浮点数
        point.y = 1024.0f * (float)rand() / (RAND_MAX + 1.0f);
        point.z = 1024.0f * (float)rand() / (RAND_MAX + 1.0f);
    }

    //创建kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    //设置随机查询值
    pcl::PointXYZ search_point;
    search_point.x = 1024.0f * (float)rand() / (RAND_MAX + 1.0f);
    search_point.y = 1024.0f * (float)rand() / (RAND_MAX + 1.0f);
    search_point.z = 1024.0f * (float)rand() / (RAND_MAX + 1.0f);

    //K 临近搜索
    //创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
    int k = 10;
    std::vector<int> point_idx_nkn_search(k);         //存储查询点近邻索引
    std::vector<float> point_nkn_squared_distance(k); //存储近邻点对应距离平方

    //打印相关信息
    std::cout << "K nearest neighbor search at (" << search_point.x
              << " " << search_point.y
              << " " << search_point.z
              << ") with K=" << k << std::endl;

    //执行K近邻搜索
    if (kdtree.nearestKSearch(search_point, k, point_idx_nkn_search, point_nkn_squared_distance) > 0)
    {
        //打印所有近邻坐标
        for (size_t i = 0; i < point_idx_nkn_search.size(); ++i)
        {
            std::cout << "    " << cloud->points[point_idx_nkn_search[i]].x
                      << " " << cloud->points[point_idx_nkn_search[i]].y
                      << " " << cloud->points[point_idx_nkn_search[i]].z
                      << " (squared distance: " << point_nkn_squared_distance[i] << ")" << std::endl;
        }
    }

    //半径 R内近邻搜索方法
    float radius = 256.0f * (float)rand() / (RAND_MAX + 1.0f); //随机的生成某一半径
    std::vector<int> point_idx_radius_search;                  //存储近邻索引
    std::vector<float> point_radius_squared_distance;          //存储近邻对应距离的平方

    //打印输出
    std::cout << "Neighbors within radius search at (" << search_point.x
              << " " << search_point.y
              << " " << search_point.z
              << ") with radius=" << radius << std::endl;

    //假设我们的kdtree返回了大于0个近邻。那么它将打印出在我们"searchPoint"附近的10个最近的邻居并把它们存到先前创立的向量中。
    if (kdtree.radiusSearch(search_point, radius, point_idx_radius_search, point_radius_squared_distance) > 0) //执行半径R内近邻搜索方法
    {
        for (size_t i = 0; i < point_idx_radius_search.size(); ++i)
        {
            std::cout << "    " << cloud->points[point_idx_radius_search[i]].x
                      << " " << cloud->points[point_idx_radius_search[i]].y
                      << " " << cloud->points[point_idx_radius_search[i]].z
                      << " (squared distance: " << point_radius_squared_distance[i] << ")" << std::endl;
        }
    }

    return 0;
}