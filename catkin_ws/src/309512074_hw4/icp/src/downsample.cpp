#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "Downsampling");

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("/home/ray/self-driving-car-2021/catkin_ws/src/309512074_hw4/map/map.pcd", *cloud); 

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("/home/ray/self-driving-car-2021/catkin_ws/src/309512074_hw4/map/map_downsample.pcd", *cloud_filtered,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return 0;

}