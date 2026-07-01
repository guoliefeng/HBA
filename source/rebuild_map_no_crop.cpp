#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

struct Pose {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

std::vector<std::string> split(const std::string &line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) tokens.push_back(token);
  return tokens;
}

bool parseInt(const std::string &text, int &value) {
  char *end = nullptr;
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') return false;
  value = static_cast<int>(parsed);
  return true;
}

Eigen::Isometry3d poseToIso(const Pose &pose) {
  Eigen::Quaterniond q = pose.q;
  q.normalize();
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = q.toRotationMatrix();
  iso.translation() = pose.t;
  return iso;
}

bool readPoses(const fs::path &g2o_path, std::vector<Pose> &poses) {
  std::ifstream input(g2o_path);
  if (!input.is_open()) {
    std::cerr << "[ERROR] failed to open " << g2o_path << std::endl;
    return false;
  }

  std::vector<std::pair<int, Pose>> indexed;
  std::string line;
  int max_id = -1;
  while (std::getline(input, line)) {
    const auto tokens = split(line);
    if (tokens.size() < 9 || tokens[0] != "VERTEX_SE3:QUAT") continue;

    int id = -1;
    if (!parseInt(tokens[1], id)) continue;

    Pose pose;
    pose.t = Eigen::Vector3d(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));
    pose.q = Eigen::Quaterniond(std::stod(tokens[8]), std::stod(tokens[5]),
                                std::stod(tokens[6]), std::stod(tokens[7]));
    pose.q.normalize();
    indexed.emplace_back(id, pose);
    max_id = std::max(max_id, id);
  }

  if (max_id < 0) {
    std::cerr << "[ERROR] no VERTEX_SE3:QUAT in " << g2o_path << std::endl;
    return false;
  }

  poses.assign(static_cast<size_t>(max_id + 1), Pose());
  std::vector<bool> seen(static_cast<size_t>(max_id + 1), false);
  for (const auto &item : indexed) {
    poses[static_cast<size_t>(item.first)] = item.second;
    seen[static_cast<size_t>(item.first)] = true;
  }
  for (int i = 0; i <= max_id; ++i) {
    if (!seen[static_cast<size_t>(i)]) {
      std::cerr << "[ERROR] missing pose id " << i << std::endl;
      return false;
    }
  }
  return true;
}

std::vector<fs::path> sortedPcdFiles(const fs::path &pcd_dir) {
  std::vector<std::pair<int, fs::path>> indexed;
  for (const auto &entry : fs::directory_iterator(pcd_dir)) {
    if (entry.path().extension() != ".pcd") continue;
    int id = -1;
    if (!parseInt(entry.path().stem().string(), id)) continue;
    indexed.emplace_back(id, entry.path());
  }
  std::sort(indexed.begin(), indexed.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  std::vector<fs::path> files;
  files.reserve(indexed.size());
  for (const auto &item : indexed) files.push_back(item.second);
  return files;
}

void voxelFilter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input,
                 pcl::PointCloud<pcl::PointXYZI> &output,
                 float leaf) {
  output.clear();
  if (!input || input->empty()) return;
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(input);
  voxel.filter(output);
}

bool rebuildNoCrop(const fs::path &map_dir) {
  std::vector<Pose> poses;
  if (!readPoses(map_dir / "pose_graph.g2o", poses)) return false;

  const auto pcd_files = sortedPcdFiles(map_dir / "pcd_buffer");
  if (pcd_files.size() != poses.size()) {
    std::cerr << "[ERROR] pcd count " << pcd_files.size()
              << " != pose count " << poses.size() << std::endl;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr sub_map(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> transformed;

  int chunk_index = 0;
  for (size_t i = 0; i < pcd_files.size(); ++i) {
    if (i % 500 == 0) {
      std::cout << "rebuild frame " << i << " / " << pcd_files.size() << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZI> cur;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files[i].string(), cur) == -1) {
      std::cerr << "[WARN] failed to load " << pcd_files[i] << std::endl;
      continue;
    }

    const Eigen::Matrix4f tf = poseToIso(poses[i]).matrix().cast<float>();
    pcl::transformPointCloud(cur, transformed, tf);
    *sub_map += transformed;

    if (static_cast<int>(i) > 100 * chunk_index || i == pcd_files.size() - 1) {
      ++chunk_index;
      pcl::PointCloud<pcl::PointXYZI> chunk;
      voxelFilter(sub_map, chunk, 0.1f);
      *map += chunk;
      sub_map->clear();
    }
  }

  pcl::PointCloud<pcl::PointXYZI> raw_filtered;
  voxelFilter(map, raw_filtered, 0.1f);
  pcl::io::savePCDFileBinary((map_dir / "raw_map.pcd").string(), raw_filtered);

  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_ptr(new pcl::PointCloud<pcl::PointXYZI>(raw_filtered));
  pcl::PointCloud<pcl::PointXYZI> map_filtered;
  voxelFilter(raw_ptr, map_filtered, 0.35f);
  pcl::io::savePCDFileBinary((map_dir / "map.pcd").string(), map_filtered);

  std::cout << "raw_map points: " << raw_filtered.size() << std::endl;
  std::cout << "map points: " << map_filtered.size() << std::endl;
  return true;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: rebuild_map_no_crop MAP_DIR" << std::endl;
    return 2;
  }
  return rebuildNoCrop(argv[1]) ? 0 : 1;
}
