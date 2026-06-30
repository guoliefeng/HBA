#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

namespace fs = std::filesystem;

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct VoxelKey {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = 0;
    auto mix = [&seed](int64_t value) {
      seed ^= std::hash<int64_t>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
  }
};

struct VoxelAccum {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
  std::size_t count = 0;
};

std::vector<std::string> split(const std::string &line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

std::string fmt(double value) {
  std::ostringstream oss;
  oss << std::setprecision(15) << value;
  return oss.str();
}

fs::path mapDirFromRoot(const fs::path &input) {
  if (fs::exists(input / "map.pcd") || fs::exists(input / "pose_graph.g2o")) {
    return input;
  }
  return input / "Map";
}

bool loadCloud(const fs::path &path, CloudT::Ptr &cloud) {
  CloudT::Ptr raw(new CloudT);
  if (pcl::io::loadPCDFile<PointT>(path.string(), *raw) != 0) {
    std::cerr << "[ERROR] failed to load " << path << std::endl;
    return false;
  }
  cloud.reset(new CloudT);
  cloud->reserve(raw->size());
  for (const auto &p : raw->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    if (std::abs(p.x) > 1.0e6 || std::abs(p.y) > 1.0e6 || std::abs(p.z) > 1.0e6) {
      continue;
    }
    cloud->push_back(p);
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  std::cout << "[INFO] loaded " << path << " points=" << raw->size()
            << " finite_points=" << cloud->size() << std::endl;
  return true;
}

CloudT::Ptr downsample(const CloudT::ConstPtr &input, double leaf) {
  if (leaf <= 0.0) {
    CloudT::Ptr copy(new CloudT);
    *copy = *input;
    return copy;
  }
  CloudT::Ptr output(new CloudT);
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.setInputCloud(input);
  voxel.filter(*output);
  return output;
}

CloudT::Ptr hashDownsample(const CloudT::ConstPtr &input, double leaf) {
  if (leaf <= 0.0) {
    CloudT::Ptr copy(new CloudT);
    *copy = *input;
    return copy;
  }

  std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;
  voxels.reserve(input->size());
  for (const auto &p : input->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    VoxelKey key;
    key.x = static_cast<int64_t>(std::floor(static_cast<double>(p.x) / leaf));
    key.y = static_cast<int64_t>(std::floor(static_cast<double>(p.y) / leaf));
    key.z = static_cast<int64_t>(std::floor(static_cast<double>(p.z) / leaf));
    auto &acc = voxels[key];
    acc.x += p.x;
    acc.y += p.y;
    acc.z += p.z;
    acc.intensity += p.intensity;
    acc.count += 1;
  }

  CloudT::Ptr output(new CloudT);
  output->reserve(voxels.size());
  for (const auto &item : voxels) {
    const auto &acc = item.second;
    if (acc.count == 0) {
      continue;
    }
    PointT p;
    p.x = static_cast<float>(acc.x / acc.count);
    p.y = static_cast<float>(acc.y / acc.count);
    p.z = static_cast<float>(acc.z / acc.count);
    p.intensity = static_cast<float>(acc.intensity / acc.count);
    output->push_back(p);
  }
  output->width = static_cast<uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

bool estimateTransform(const CloudT::ConstPtr &fixed, const CloudT::ConstPtr &moving,
                       double max_corr, Eigen::Isometry3d &t_moving_to_fixed) {
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setInputSource(moving);
  gicp.setInputTarget(fixed);
  gicp.setMaximumIterations(100);
  gicp.setTransformationEpsilon(1e-7);
  gicp.setEuclideanFitnessEpsilon(1e-5);
  gicp.setMaxCorrespondenceDistance(max_corr);

  CloudT aligned;
  gicp.align(aligned);
  if (!gicp.hasConverged()) {
    std::cerr << "[ERROR] GICP did not converge. score=" << gicp.getFitnessScore() << std::endl;
    return false;
  }

  t_moving_to_fixed.setIdentity();
  t_moving_to_fixed.matrix() = gicp.getFinalTransformation().cast<double>();
  const Eigen::AngleAxisd aa(t_moving_to_fixed.rotation());
  std::cout << "[INFO] GICP converged score=" << gicp.getFitnessScore()
            << " translation_norm=" << t_moving_to_fixed.translation().norm()
            << " rotation_rad=" << std::abs(aa.angle()) << std::endl;
  return true;
}

bool saveTransform(const fs::path &path, const Eigen::Isometry3d &tf) {
  std::ofstream out(path);
  if (!out.is_open()) {
    std::cerr << "[ERROR] failed to write " << path << std::endl;
    return false;
  }
  out << std::setprecision(15);
  out << "# T_track2_to_track1. Apply as p_track1 = T * p_track2\n";
  out << "matrix4x4\n";
  out << tf.matrix() << "\n";
  const Eigen::Quaterniond q(tf.rotation());
  out << "translation_quaternion_xyzw\n";
  out << tf.translation().x() << ' ' << tf.translation().y() << ' ' << tf.translation().z() << ' '
      << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << "\n";
  return true;
}

bool rewritePoseGraphWithTransform(const fs::path &input_g2o, const fs::path &output_g2o,
                                   const Eigen::Isometry3d &tf) {
  std::ifstream in(input_g2o);
  if (!in.is_open()) {
    std::cerr << "[WARN] no pose graph to transform: " << input_g2o << std::endl;
    return false;
  }
  std::ofstream out(output_g2o);
  if (!out.is_open()) {
    std::cerr << "[ERROR] failed to write " << output_g2o << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(in, line)) {
    const auto tokens = split(line);
    if (tokens.size() >= 9 && tokens[0] == "VERTEX_SE3:QUAT") {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q(std::stod(tokens[8]), std::stod(tokens[5]),
                           std::stod(tokens[6]), std::stod(tokens[7]));
      q.normalize();
      pose.linear() = q.toRotationMatrix();
      pose.translation() = Eigen::Vector3d(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));

      const Eigen::Isometry3d aligned = tf * pose;
      const Eigen::Quaterniond qa(aligned.rotation());
      out << "VERTEX_SE3:QUAT " << tokens[1] << ' '
          << fmt(aligned.translation().x()) << ' '
          << fmt(aligned.translation().y()) << ' '
          << fmt(aligned.translation().z()) << ' '
          << fmt(qa.x()) << ' ' << fmt(qa.y()) << ' '
          << fmt(qa.z()) << ' ' << fmt(qa.w()) << '\n';
    } else {
      out << line << '\n';
    }
  }
  return true;
}

bool writeMergedCloud(const fs::path &track2_aligned_path, const fs::path &merged_path,
                      const CloudT::ConstPtr &fixed_raw, const CloudT::ConstPtr &moving_raw,
                      const Eigen::Isometry3d &tf, double output_leaf) {
  CloudT::Ptr moving_aligned(new CloudT);
  pcl::transformPointCloud(*moving_raw, *moving_aligned, tf.matrix().cast<float>());
  if (pcl::io::savePCDFileBinary(track2_aligned_path.string(), *moving_aligned) != 0) {
    std::cerr << "[ERROR] failed to write " << track2_aligned_path << std::endl;
    return false;
  }

  CloudT::Ptr merged(new CloudT);
  *merged = *fixed_raw;
  *merged += *moving_aligned;
  CloudT::Ptr merged_filtered = hashDownsample(merged, output_leaf);
  if (pcl::io::savePCDFileBinary(merged_path.string(), *merged_filtered) != 0) {
    std::cerr << "[ERROR] failed to write " << merged_path << std::endl;
    return false;
  }

  std::cout << "[DONE] wrote " << track2_aligned_path << " points=" << moving_aligned->size() << std::endl;
  std::cout << "[DONE] wrote " << merged_path << " points=" << merged_filtered->size()
            << " output_leaf=" << output_leaf << std::endl;
  return true;
}

void writeOptionalFullMapPreview(const fs::path &track1_map, const fs::path &track2_map,
                                 const fs::path &output_dir, const std::string &matched_map_name,
                                 const Eigen::Isometry3d &tf, double output_leaf) {
  if (matched_map_name == "map.pcd") {
    return;
  }
  const fs::path full1 = track1_map / "map.pcd";
  const fs::path full2 = track2_map / "map.pcd";
  if (!fs::exists(full1) || !fs::exists(full2)) {
    return;
  }

  CloudT::Ptr fixed_full;
  CloudT::Ptr moving_full;
  if (!loadCloud(full1, fixed_full) || !loadCloud(full2, moving_full)) {
    return;
  }
  writeMergedCloud(output_dir / "track2_aligned_full_map.pcd",
                   output_dir / "track1_track2_merged_full_map.pcd",
                   fixed_full, moving_full, tf, output_leaf);
}

int main(int argc, char **argv) {
  if (argc < 4 || argc > 8) {
    std::cerr << "usage: track_align TRACK1_ROOT_OR_MAP_DIR TRACK2_ROOT_OR_MAP_DIR OUTPUT_DIR "
                 "[map_name=map.pcd] [icp_leaf=0.5] [output_leaf=0.2] [max_corr=3.0]\n";
    return 2;
  }

  const fs::path track1_map = mapDirFromRoot(argv[1]);
  const fs::path track2_map = mapDirFromRoot(argv[2]);
  const fs::path output_dir = argv[3];
  const std::string map_name = argc >= 5 ? argv[4] : "map.pcd";
  const double icp_leaf = argc >= 6 ? std::stod(argv[5]) : 0.5;
  const double output_leaf = argc >= 7 ? std::stod(argv[6]) : 0.2;
  const double max_corr = argc >= 8 ? std::stod(argv[7]) : 3.0;

  fs::create_directories(output_dir);

  CloudT::Ptr fixed_raw;
  CloudT::Ptr moving_raw;
  if (!loadCloud(track1_map / map_name, fixed_raw) ||
      !loadCloud(track2_map / map_name, moving_raw)) {
    return 1;
  }

  CloudT::Ptr fixed_ds = downsample(fixed_raw, icp_leaf);
  CloudT::Ptr moving_ds = downsample(moving_raw, icp_leaf);
  std::cout << "[INFO] downsampled fixed=" << fixed_ds->size()
            << " moving=" << moving_ds->size()
            << " icp_leaf=" << icp_leaf << std::endl;

  Eigen::Isometry3d t_track2_to_track1 = Eigen::Isometry3d::Identity();
  if (!estimateTransform(fixed_ds, moving_ds, max_corr, t_track2_to_track1)) {
    return 1;
  }

  saveTransform(output_dir / "T_track2_to_track1.txt", t_track2_to_track1);
  rewritePoseGraphWithTransform(track2_map / "pose_graph.g2o",
                                output_dir / "pose_graph_track2_aligned.g2o",
                                t_track2_to_track1);
  if (!writeMergedCloud(output_dir / "track2_aligned.pcd",
                        output_dir / "track1_track2_merged.pcd",
                        fixed_raw, moving_raw, t_track2_to_track1, output_leaf)) {
    return 1;
  }
  writeOptionalFullMapPreview(track1_map, track2_map, output_dir, map_name,
                              t_track2_to_track1, output_leaf);

  return 0;
}
