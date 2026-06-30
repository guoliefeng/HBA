#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = boost::filesystem;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct VoxelKey
{
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator==(const VoxelKey& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey& key) const
  {
    size_t seed = 0;
    auto mix = [&seed](int64_t value)
    {
      seed ^= std::hash<int64_t>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
  }
};

struct VoxelAccum
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  uint32_t count = 0;
};

struct Pose
{
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

bool readPoseFile(const fs::path& path, std::vector<Pose>& poses)
{
  poses.clear();
  std::ifstream file(path.string());
  if(!file.is_open())
  {
    std::cerr << "[ERROR] cannot open pose file: " << path.string() << std::endl;
    return false;
  }

  double tx, ty, tz, qw, qx, qy, qz;
  while(file >> tx >> ty >> tz >> qw >> qx >> qy >> qz)
  {
    Pose p;
    p.t = Eigen::Vector3d(tx, ty, tz);
    p.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
    poses.push_back(p);
  }
  return !poses.empty();
}

bool readSinglePose(const fs::path& path, Pose& pose)
{
  std::vector<Pose> poses;
  if(!readPoseFile(path, poses))
    return false;
  pose = poses.front();
  return true;
}

std::string paddedName(size_t index, int fill_num)
{
  std::ostringstream ss;
  ss << std::setw(fill_num) << std::setfill('0') << index << ".pcd";
  return ss.str();
}

void transformAppend(const CloudT& src,
                     std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash>& voxels,
                     const Pose& origin,
                     const Pose& local,
                     double voxel_size)
{
  const Eigen::Quaterniond q_global = (origin.q * local.q).normalized();
  const Eigen::Vector3d t_global = origin.q * local.t + origin.t;

  for(size_t i = 0; i < src.size(); ++i)
  {
    const auto& p = src.points[i];
    const Eigen::Vector3d pt = q_global * Eigen::Vector3d(p.x, p.y, p.z) + t_global;
    if(!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z()))
      continue;

    VoxelKey key;
    key.x = static_cast<int64_t>(std::floor(pt.x() / voxel_size));
    key.y = static_cast<int64_t>(std::floor(pt.y() / voxel_size));
    key.z = static_cast<int64_t>(std::floor(pt.z() / voxel_size));

    auto& acc = voxels[key];
    acc.x += pt.x();
    acc.y += pt.y();
    acc.z += pt.z();
    acc.count += 1;
  }
}

bool mergeRoute(const fs::path& route_dir,
                std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash>& voxels,
                double voxel_size)
{
  Pose origin;
  std::vector<Pose> poses;
  if(!readSinglePose(route_dir / "origin.json", origin))
    return false;
  if(!readPoseFile(route_dir / "pose_after_hba.json", poses))
    return false;

  const fs::path pcd_dir = route_dir / "pcd";
  if(!fs::exists(pcd_dir) || !fs::is_directory(pcd_dir))
  {
    std::cerr << "[ERROR] pcd directory not found: " << pcd_dir.string() << std::endl;
    return false;
  }

  const int fill_num = std::max(1, static_cast<int>(std::floor(std::log10(std::max<size_t>(1, poses.size() - 1)))) + 1);
  size_t loaded = 0;
  for(size_t i = 0; i < poses.size(); ++i)
  {
    const fs::path pcd_path = pcd_dir / paddedName(i, fill_num);
    CloudT frame;
    if(pcl::io::loadPCDFile(pcd_path.string(), frame) != 0)
    {
      std::cerr << "[WARN] skip failed pcd: " << pcd_path.string() << std::endl;
      continue;
    }
    transformAppend(frame, voxels, origin, poses[i], voxel_size);
    ++loaded;
  }

  std::cout << "[INFO] " << route_dir.filename().string()
            << " poses=" << poses.size()
            << " loaded_pcd=" << loaded
            << " voxel_count=" << voxels.size() << std::endl;
  return loaded > 0;
}

int main(int argc, char** argv)
{
  const fs::path routes_root = argc > 1
      ? fs::path(argv[1])
      : fs::path("/home/glf/dataDisk/hainan/yangpu/HBA/routes");
  const fs::path output_path = argc > 2
      ? fs::path(argv[2])
      : routes_root / "yangpu_13routes_merged_voxel_0_1.pcd";
  const double voxel_size = argc > 3 ? std::stod(argv[3]) : 0.1;

  std::vector<fs::path> route_dirs;
  for(const auto& entry : fs::directory_iterator(routes_root))
  {
    if(!fs::is_directory(entry.path()))
      continue;
    const std::string name = entry.path().filename().string();
    if(name.find("route_") == 0)
      route_dirs.push_back(entry.path());
  }
  std::sort(route_dirs.begin(), route_dirs.end());

  std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;
  for(const auto& route_dir : route_dirs)
    mergeRoute(route_dir, voxels, voxel_size);

  CloudT::Ptr filtered(new CloudT);
  filtered->reserve(voxels.size());
  for(const auto& item : voxels)
  {
    const auto& acc = item.second;
    if(acc.count == 0)
      continue;
    PointT p;
    p.x = static_cast<float>(acc.x / acc.count);
    p.y = static_cast<float>(acc.y / acc.count);
    p.z = static_cast<float>(acc.z / acc.count);
    filtered->push_back(p);
  }
  filtered->width = static_cast<uint32_t>(filtered->size());
  filtered->height = 1;
  filtered->is_dense = false;

  pcl::io::savePCDFileBinary(output_path.string(), *filtered);
  std::cout << "[DONE] wrote voxel merged map: " << output_path.string()
            << " voxel=" << voxel_size
            << " points=" << filtered->size() << std::endl;
  return 0;
}
