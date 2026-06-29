/**
 * @file visualize.cpp
 * @brief 优化后点云地图可视化节点
 *
 * 逐帧加载 PCD，按 pose.json 变换到世界系，发布点云地图、轨迹和位姿标记。
 * 需配合 visualize.launch 使用。
 */
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ros/ros.h"
#include <math.h>
#include <rosbag/bag.h>
#include <ceres/ceres.h>

#include "ba.hpp"
#include "tools.hpp"
#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh("~");

  // ROS 话题发布器
  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 100);
  ros::Publisher pub_debug = nh.advertise<sensor_msgs::PointCloud2>("/cloud_debug", 100);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseArray>("/poseArrayTopic", 10);
  ros::Publisher pub_trajectory = nh.advertise<visualization_msgs::Marker>("/trajectory_marker", 100);
  ros::Publisher pub_pose_number = nh.advertise<visualization_msgs::MarkerArray>("/pose_number", 100);

  string file_path;
  double downsample_size, marker_size;
  int pcd_name_fill_num;

  nh.getParam("file_path", file_path);
  nh.getParam("downsample_size", downsample_size);
  nh.getParam("pcd_name_fill_num", pcd_name_fill_num);
  nh.getParam("marker_size", marker_size);

  sensor_msgs::PointCloud2 debugMsg, cloudMsg, outMsg;
  vector<mypcl::pose> pose_vec;

  pose_vec = mypcl::read_pose(file_path + "pose.json");
  size_t pose_size = pose_vec.size();
  cout<<"pose size "<<pose_size<<endl;

  pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_full(new pcl::PointCloud<pcl::PointXYZRGB>);

  ros::Time cur_t = ros::Time::now();
  geometry_msgs::PoseArray parray;
  parray.header.frame_id = "camera_init";
  parray.header.stamp = cur_t;
  visualization_msgs::MarkerArray markerArray;

  cout<<"push enter to view"<<endl;
  getchar();

  // 逐帧加载、变换、发布
  for(size_t i = 0; i < pose_size; i++)
  {
    cout<<"loading PCD: "<< i << ".pcd"<<endl;
    cout.flush();
    mypcl::loadPCD(file_path + "pcd/", pcd_name_fill_num, pc_surf, i);
    if(pc_surf->points.size() < 5)
    {
      cout<< i << ".pcd is empty: " << pc_surf->points.size() <<endl;
      continue;
    }
    cout<<"loaded PCD: "<< i << ".pcd"<<endl;

    pcl::PointCloud<PointType>::Ptr pc_filtered(new pcl::PointCloud<PointType>);
    pc_filtered->resize(pc_surf->points.size());
    int cnt = 0;
    for(size_t j = 0; j < pc_surf->points.size(); j++)
    {
      pc_filtered->points[cnt] = pc_surf->points[j];
      cnt++;
    }
    pc_filtered->resize(cnt);
    
    // 变换到世界坐标系并降采样
    mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);
    downsample_voxel(*pc_filtered, downsample_size);

    pcl::toROSMsg(*pc_filtered, cloudMsg);
    cloudMsg.header.frame_id = "camera_init";
    cloudMsg.header.stamp = cur_t;
    cout << "publish " << i << ".pcd" << endl;
    pub_map.publish(cloudMsg);
    cout << "##### 1 #####"<<endl;

    // 发布当前帧位姿
    geometry_msgs::Pose apose;
    apose.orientation.w = pose_vec[i].q.w();
    apose.orientation.x = pose_vec[i].q.x();
    apose.orientation.y = pose_vec[i].q.y();
    apose.orientation.z = pose_vec[i].q.z();
    apose.position.x = pose_vec[i].t(0);
    apose.position.y = pose_vec[i].t(1);
    apose.position.z = pose_vec[i].t(2);
    parray.poses.push_back(apose);
    cout << "publish pose" << endl;
    pub_pose.publish(parray);
    cout << "##### 2 #####"<<endl;

    // 发布轨迹球体标记（颜色随帧号渐变）
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = cur_t;
    marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = pose_vec[i].t(0);
    marker.pose.position.y = pose_vec[i].t(1);
    marker.pose.position.z = pose_vec[i].t(2);
    pose_vec[i].q.normalize();
    marker.pose.orientation.x = pose_vec[i].q.x();
    marker.pose.orientation.y = pose_vec[i].q.y();
    marker.pose.orientation.z = pose_vec[i].q.x();
    marker.pose.orientation.w = pose_vec[i].q.w();
    marker.scale.x = marker_size;
    marker.scale.y = marker_size;
    marker.scale.z = marker_size;
    marker.color.r = float(1-float(i)/pose_size);
    marker.color.g = float(float(i)/pose_size);
    marker.color.b = float(float(i)/pose_size);
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    cout << "publish trajectory" << endl;
    pub_trajectory.publish(marker);
    cout << "##### 3 #####"<<endl;

    // 每 GAP 帧发布帧号文字标记
    visualization_msgs::Marker marker_txt;
    marker_txt.header.frame_id = "camera_init";
    marker_txt.header.stamp = cur_t;
    marker_txt.ns = "marker_txt";
    marker_txt.id = i;
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ostringstream str;
    str << i;
    marker_txt.text = str.str();
    marker.action = visualization_msgs::Marker::ADD;
    marker_txt.action = visualization_msgs::Marker::ADD;
    marker_txt.pose.position.x = pose_vec[i].t(0)+marker_size;
    marker_txt.pose.position.y = pose_vec[i].t(1)+marker_size;
    marker_txt.pose.position.z = pose_vec[i].t(2);
    marker_txt.pose.orientation.x = pose_vec[i].q.x();
    marker_txt.pose.orientation.y = pose_vec[i].q.y();
    marker_txt.pose.orientation.z = pose_vec[i].q.x();
    marker_txt.pose.orientation.w = 1.0;
    marker_txt.scale.x = marker_size;
    marker_txt.scale.y = marker_size;
    marker_txt.scale.z = marker_size;
    marker_txt.color.r = 1.0f;
    marker_txt.color.g = 1.0f;
    marker_txt.color.b = 1.0f;
    marker_txt.color.a = 1.0;
    marker_txt.lifetime = ros::Duration();
    if(i%GAP == 0) markerArray.markers.push_back(marker_txt);
    cout << "publish marker" << endl;
    pub_pose_number.publish(markerArray);
    cout << "##### 4 #####"<<endl;

    // use_sim_time=true 且无 /clock 时，ros::Duration::sleep() 会永久阻塞
    ros::WallDuration(0.001).sleep();
    ros::spinOnce();
  }

  // 保持节点运行以便 RViz 持续显示
  ros::WallRate loop_rate(1);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
