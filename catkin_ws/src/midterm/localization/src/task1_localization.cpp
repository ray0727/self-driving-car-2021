#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "math.h"

// using namespace ros;
using namespace std;

class icp_localization
{
private:
  ros::Subscriber sub_lidar_pc;
  ros::Publisher  pub_icp_pc, pub_odom, pub_map;
  ros::NodeHandle nh;
  int count;
  sensor_msgs::PointCloud2 map_pc, icp_pc;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_input;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_voxel;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_input;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_input_voxel;
  Eigen::Matrix4f initial_guess;
  tf::TransformListener listener;
  ofstream outFile;

public:
  icp_localization();
  void cb_lidar_pc(const sensor_msgs::PointCloud2 &msg);
  void cb_gps(const geometry_msgs::PoseStamped &msg);
  Eigen::Matrix4f get_initial_guess();
  Eigen::Matrix4f get_transfrom(std::string link);
};

icp_localization::icp_localization(){
  map_input.reset(new pcl::PointCloud<pcl::PointXYZI>());
  map_voxel.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pc_input.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pc_input_voxel.reset(new pcl::PointCloud<pcl::PointXYZI>());

  count = 0;
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/ray/self-driving-car-2021/catkin_ws/src/midterm/localization/map/itri_map.pcd", *map_input) == -1)
  {
    PCL_ERROR ("Couldn't read file map_downsampled.pcd \n");
    exit(0);
  }

  //=======voxel grid filter=====================
  cout << "PointCloud before filtering: " << map_input->points.size() << endl;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_map;
  pcl::PCLPointCloud2::Ptr map_tmp (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*map_input, *map_tmp);
  sor_map.setInputCloud (map_tmp);
  sor_map.setLeafSize (0.3f, 0.3f, 0.3f);
  sor_map.filter (*map_tmp);
  pcl::fromPCLPointCloud2(*map_tmp, *map_voxel);
  cout << "PointCloud after filtering: " << map_voxel->points.size() << endl;

  pcl::toROSMsg(*map_voxel, map_pc);

  sub_lidar_pc = nh.subscribe("lidar_points", 1, &icp_localization::cb_lidar_pc, this);
  pub_icp_pc = nh.advertise<sensor_msgs::PointCloud2>("icp_pc", 10);
  pub_odom = nh.advertise<nav_msgs::Odometry>("odom_result", 10);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("map", 10);

  cout << "waiting for gps and IMU message" << endl;
  initial_guess = get_initial_guess();
  cout << "initial guess:" << endl;
  cout << initial_guess << endl;
  
  outFile.open("/home/ray/self-driving-car-2021/catkin_ws/src/midterm/localization/task1_result.csv", ios::out);
  outFile <<"id,x,y,z,yaw,pitch,roll"<<endl;
  cout << "start icp:" << endl;

}

Eigen::Matrix4f icp_localization::get_initial_guess(){

  // sensor_msgs::ImuConstPtr imu;
  // imu = ros::topic::waitForMessage<sensor_msgs::Imu>("/imu/data", nh);
  // double roll, pitch, yaw;
  // tf2::Quaternion imu_orien(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
  // tf2::Matrix3x3 m(imu_orien);
  // m.getRPY(roll, pitch, yaw);
  // yaw = 2.454;
  double yaw = 2.45;
  tf2::Quaternion q;
  q.setRPY(0,0,yaw);
  tf2::Matrix3x3 rotation;
  rotation.setRotation(q);
  Eigen::Matrix4f trans = Eigen::Matrix4f::Zero();
  geometry_msgs::PointStampedConstPtr gps;
  gps = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", nh);
  
  
  trans << rotation[0][0], rotation[0][1], rotation[0][2],  (*gps).point.x,
           rotation[1][0], rotation[1][1], rotation[1][2],  (*gps).point.y,
			     rotation[2][0], rotation[2][1], rotation[2][2],  (*gps).point.z,
			          0,              0,              0,                      1;

  return trans;
}

Eigen::Matrix4f icp_localization::get_transfrom(std::string link){

	tf::StampedTransform transform;
	Eigen::Matrix4f trans;

	try{
      ros::Duration five_seconds(5.0);
      listener.waitForTransform("/base_link", link, ros::Time(0), five_seconds);
      listener.lookupTransform("/base_link", link, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return trans;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
	Eigen::Matrix3f matrix = q.toRotationMatrix();
	trans << matrix(0,0), matrix(0,1), matrix(0,2), transform.getOrigin().getX(),
			     matrix(1,0), matrix(1,1), matrix(1,2), transform.getOrigin().getY(),
			     matrix(2,0), matrix(2,1), matrix(2,2), transform.getOrigin().getZ(),
			     0, 0, 0, 1;
	return trans;
}

void icp_localization::cb_lidar_pc(const sensor_msgs::PointCloud2 &msg){
  count++;
  cout<< "cb_num: " << count <<endl;
  pcl::fromROSMsg(msg, *pc_input);
  Eigen::Matrix4f trans = get_transfrom("/velodyne");
	pcl::transformPointCloud(*pc_input, *pc_input, trans);
  ROS_INFO("transformed to base_link");
  

  cout<<"original: "<< pc_input->points.size()<<endl;
  // =========filter============

  pcl::PassThrough<pcl::PointXYZI> pass_x;
  pass_x.setInputCloud(pc_input);
  pass_x.setFilterFieldName ("x"); 
  pass_x.setFilterLimits (-25, 25);
  pass_x.filter (*pc_input);
  cout<<"filter_x: "<<pc_input->points.size()<<endl;

  pcl::PassThrough<pcl::PointXYZI> pass_y;
  pass_y.setInputCloud(pc_input);
  pass_y.setFilterFieldName ("y"); 
  pass_y.setFilterLimits (-10, 10);
  pass_y.filter (*pc_input);
  cout<<"filter_y: "<<pc_input->points.size()<<endl;

  
  pcl::PassThrough<pcl::PointXYZI> pass_z;
  pass_z.setInputCloud(pc_input);
  pass_z.setFilterFieldName ("z"); 
  pass_z.setFilterLimits (-1, 3);
  pass_z.filter (*pc_input);
  cout<<"filter_z: "<<pc_input->points.size()<<endl;
  
  // //=======voxel grid filter=====================
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::PCLPointCloud2::Ptr pc_input_tmp (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*pc_input, *pc_input_tmp);
  sor.setInputCloud (pc_input_tmp);
  sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.filter (*pc_input_tmp);
  pcl::fromPCLPointCloud2(*pc_input_tmp, *pc_input_voxel);
  cout<<"voxel grid filter: "<<pc_input_voxel->points.size()<<endl;

  
  //=======icp==========================
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(pc_input_voxel);
  icp.setInputTarget(map_voxel);
  icp.setMaximumIterations (2000);
  icp.setTransformationEpsilon (1e-10);
  icp.setMaxCorrespondenceDistance (2);
  icp.setEuclideanFitnessEpsilon (1e-4);
  icp.setRANSACOutlierRejectionThreshold (0.02);
  pcl::PointCloud<pcl::PointXYZI> Final;
  icp.align(Final, initial_guess);

  cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
  cout << icp.getFinalTransformation() << endl;
  initial_guess = icp.getFinalTransformation();

  tf::Matrix3x3 tf3d;
  tf3d.setValue((initial_guess(0,0)), (initial_guess(0,1)), (initial_guess(0,2)),
        (initial_guess(1,0)), (initial_guess(1,1)), (initial_guess(1,2)),
        (initial_guess(2,0)), (initial_guess(2,1)), (initial_guess(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(initial_guess(0,3),initial_guess(1,3),initial_guess(2,3)));
  transform.setRotation(tfqt);

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","base_link"));

  // Publish my lidar pointcloud after doing ICP.
  pcl::toROSMsg(Final, icp_pc);
  icp_pc.header=msg.header;
  icp_pc.header.frame_id = "world";
  pub_icp_pc.publish(icp_pc);

  //=======show map===================== 
  map_pc.header.frame_id = "world";
  map_pc.header.stamp = ros::Time::now();
  pub_map.publish(map_pc);

  //====publish localization result====
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = initial_guess(0,3);
  odom.pose.pose.position.y = initial_guess(1,3);
  odom.pose.pose.position.z = initial_guess(2,3);
  tf2::Matrix3x3 tfm;
  tfm.setValue(initial_guess(0,0) ,initial_guess(0,1) ,initial_guess(0,2) ,
              initial_guess(1,0) ,initial_guess(1,1) ,initial_guess(1,2) ,
              initial_guess(2,0) ,initial_guess(2,1) ,initial_guess(2,2));
  tf2::Quaternion tfq2;
  tfm.getRotation(tfq2);
  odom.pose.pose.orientation.x = tfq2[0];
  odom.pose.pose.orientation.y = tfq2[1];
  odom.pose.pose.orientation.z = tfq2[2];
  odom.pose.pose.orientation.w = tfq2[3];
  pub_odom.publish(odom);

  
  double quatx= odom.pose.pose.orientation.x;
  double quaty= odom.pose.pose.orientation.y;
  double quatz= odom.pose.pose.orientation.z;
  double quatw= odom.pose.pose.orientation.w;

  tf::Quaternion qua(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 matrix(qua);
  double roll, pitch, yaw;
  // cout<< "cb_num: " << count <<endl;
  matrix.getRPY(roll, pitch, yaw);

  outFile << count <<','<< odom.pose.pose.position.x << ',' << odom.pose.pose.position.y << ',' << odom.pose.pose.position.z <<','<< yaw << ',' << pitch << ',' << roll << endl;
  if(count==201){
    cout<<"close file"<<endl;
    outFile.close();
  }

}
int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp_localization");
  icp_localization icp;
  ros::spin();
}