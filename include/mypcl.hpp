/**
 * @file mypcl.hpp
 * @brief 点云与位姿 I/O、坐标变换等 PCL 辅助函数
 */
#ifndef MYPCL_HPP
#define MYPCL_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3d;
typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > vector_quad;
typedef pcl::PointXYZ PointType;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace mypcl
{
  /** @brief 位姿：四元数旋转 + 平移向量 */
  struct pose
  {
    pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
         Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)):q(_q), t(_t){}
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
  };

  /**
   * @brief 从磁盘加载单帧 PCD 点云
   * @param filePath 目录路径
   * @param pcd_fill_num 文件名零填充位数（如 KITTI 为 5，则 00000.pcd）
   * @param pc 输出点云指针
   * @param num 帧编号
   * @param prefix 子目录前缀，默认空
   */
  void loadPCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr& pc, int num,
               std::string prefix = "")
  {
    std::stringstream ss;
    if(pcd_fill_num > 0)
      ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
    else
      ss << num;
    pcl::io::loadPCDFile(filePath + prefix + ss.str() + ".pcd", *pc);
  }

  /** @brief 保存单帧 PCD 点云（二进制格式） */
  void savdPCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr& pc, int num)
  {
    std::stringstream ss;
    if(pcd_fill_num > 0)
      ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
    else
      ss << num;
    pcl::io::savePCDFileBinary(filePath + ss.str() + ".pcd", *pc);
  }
  
  /**
   * @brief 从 pose.json 读取位姿序列
   * 格式：每行 tx ty tz qw qx qy qz
   * @param qe, te 可选的外参变换，用于坐标系对齐
   */
  std::vector<pose> read_pose(std::string filename,
                              Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0),
                              Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
  {
    std::vector<pose> pose_vec;
    std::fstream file;
    file.open(filename);
    double tx, ty, tz, w, x, y, z;
    while(!file.eof())
    {
      file >> tx >> ty >> tz >> w >> x >> y >> z;
      Eigen::Quaterniond q(w, x, y, z);
      Eigen::Vector3d t(tx, ty, tz);
      pose_vec.push_back(pose(qe * q, qe * t + te));
    }
    file.close();
    return pose_vec;
  }

  /** @brief 对点云施加刚体变换：p_out = q * p_in + t */
  void transform_pointcloud(pcl::PointCloud<PointType> const& pc_in,
                            pcl::PointCloud<PointType>& pt_out,
                            Eigen::Vector3d t,
                            Eigen::Quaterniond q)
  {
    size_t size = pc_in.points.size();
    pt_out.points.resize(size);
    for(size_t i = 0; i < size; i++)
    {
      Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
      Eigen::Vector3d pt_to;
      pt_to = q * pt_cur + t;
      pt_out.points[i].x = pt_to.x();
      pt_out.points[i].y = pt_to.y();
      pt_out.points[i].z = pt_to.z();
    }
  }

  /** @brief 将 pc2 追加到 pc1 末尾（RGB 点云版本） */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr append_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1,
                                                      pcl::PointCloud<pcl::PointXYZRGB> pc2)
  {
    size_t size1 = pc1->points.size();
    size_t size2 = pc2.points.size();
    pc1->points.resize(size1 + size2);
    for(size_t i = size1; i < size1 + size2; i++)
    {
      pc1->points[i].x = pc2.points[i-size1].x;
      pc1->points[i].y = pc2.points[i-size1].y;
      pc1->points[i].z = pc2.points[i-size1].z;
      pc1->points[i].r = pc2.points[i-size1].r;
      pc1->points[i].g = pc2.points[i-size1].g;
      pc1->points[i].b = pc2.points[i-size1].b;
    }
    return pc1;
  }

  /** @brief 将 pc2 追加到 pc1 末尾（XYZ 点云版本） */
  pcl::PointCloud<PointType>::Ptr append_cloud(pcl::PointCloud<PointType>::Ptr pc1,
                                               pcl::PointCloud<PointType> pc2)
  {
    size_t size1 = pc1->points.size();
    size_t size2 = pc2.points.size();
    pc1->points.resize(size1 + size2);
    for(size_t i = size1; i < size1 + size2; i++)
    {
      pc1->points[i].x = pc2.points[i-size1].x;
      pc1->points[i].y = pc2.points[i-size1].y;
      pc1->points[i].z = pc2.points[i-size1].z;
    }
    return pc1;
  }

  /** @brief 根据残差计算内点比例阈值对应的距离 */
  double compute_inlier_ratio(std::vector<double> residuals, double ratio)
  {
    std::set<double> dis_vec;
    for(size_t i = 0; i < (size_t)(residuals.size() / 3); i++)
      dis_vec.insert(fabs(residuals[3 * i + 0]) +
                     fabs(residuals[3 * i + 1]) + fabs(residuals[3 * i + 2]));

    return *(std::next(dis_vec.begin(), (int)((ratio) * dis_vec.size())));
  }

  /**
   * @brief 将位姿序列写入 pose.json
   * 写入前将所有位姿变换到以第 0 帧为参考的坐标系
   */
  void write_pose(std::vector<pose>& pose_vec, std::string path)
  {
    std::ofstream file;
    file.open(path + "pose.json", std::ofstream::trunc);
    file.close();
    Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
    Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
    file.open(path + "pose.json", std::ofstream::app);

    for(size_t i = 0; i < pose_vec.size(); i++)
    {
      // 相对第 0 帧的位姿
      pose_vec[i].t << q0.inverse()*(pose_vec[i].t-t0);
      pose_vec[i].q.w() = (q0.inverse()*pose_vec[i].q).w();
      pose_vec[i].q.x() = (q0.inverse()*pose_vec[i].q).x();
      pose_vec[i].q.y() = (q0.inverse()*pose_vec[i].q).y();
      pose_vec[i].q.z() = (q0.inverse()*pose_vec[i].q).z();
      file << pose_vec[i].t(0) << " "
           << pose_vec[i].t(1) << " "
           << pose_vec[i].t(2) << " "
           << pose_vec[i].q.w() << " " << pose_vec[i].q.x() << " "
           << pose_vec[i].q.y() << " " << pose_vec[i].q.z();
      if(i < pose_vec.size()-1) file << "\n";
    }
    file.close();
  }

  /** @brief 导出 EVO 评估格式的位姿文件 (timestamp tx ty tz qx qy qz qw) */
  void writeEVOPose(std::vector<double>& lidar_times, std::vector<pose>& pose_vec, std::string path)
  {
    std::ofstream file;
    file.open(path + "evo_pose.txt", std::ofstream::trunc);
    for(size_t i = 0; i < pose_vec.size(); i++)
    {
      file << std::setprecision(18) << lidar_times[i] << " " << std::setprecision(6)
           << pose_vec[i].t(0) << " " << pose_vec[i].t(1) << " " << pose_vec[i].t(2) << " "
           << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
           << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
      if(i < pose_vec.size()-1) file << "\n";
    }
    file.close();
  }
}

#endif
