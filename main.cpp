#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <chrono>


pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);


int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPLYFile<pcl::PointXYZ> ("../S.ply", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }


for (const auto& point : *cloud)
{
    std::cout << "x: " << point.x
              << ", y: " << point.y
              << ", z: " << point.z
              << std::endl;
}

    auto start_time = std::chrono::high_resolution_clock::now();

    ne.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    std::cout << " 1 " << std::endl;
    ne.setSearchMethod (tree);
    std::cout << " 2 " << std::endl;
    ne.setRadiusSearch (0.03);
    std::cout << " 3 " << std::endl;
    ne.compute (*cloud_normals);
    std::cout << " 4 " << std::endl;
    auto end_time = std::chrono::high_resolution_clock::now();


    for (const auto& point : *cloud_normals)
    {
        std::cout << "x NORM: " << point.normal_x
                  << ", y NORM: " << point.normal_y
                  << ", z NORM: " << point.normal_z
                  << std::endl;
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;

/*
    pcl::PointCloud<pcl::PointNormal>::Ptr xyzNormalsCloud(new pcl::PointCloud<pcl::PointNormal>);

    pcl::concatenateFields(*cloud, *cloud_normals, *xyzNormalsCloud);

for (const auto& point : *xyzNormalsCloud)
{
    std::cout << "x: " << point.x
              << ", y: " << point.y
              << ", z: " << point.z
              << ", normal_x: " << point.normal_x
              << ", normal_y: " << point.normal_y
              << ", normal_z: " << point.normal_z << std::endl;
}
*/




 //   pcl::io::savePLYFileASCII ("merged.ply", *xyzNormalsCloud);


  return (0);
}


