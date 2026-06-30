#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

namespace fs = std::filesystem;

struct PoseEntry {
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

struct EdgeEntry {
  std::size_t i = 0;
  std::size_t j = 0;
  PoseEntry measurement;
  bool is_loop = false;
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

bool parseSize(const std::string &text, std::size_t &value) {
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  value = static_cast<std::size_t>(parsed);
  return true;
}

Eigen::Isometry3d poseToIso(const PoseEntry &pose) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q = pose.q;
  q.normalize();
  iso.linear() = q.toRotationMatrix();
  iso.translation() = pose.t;
  return iso;
}

PoseEntry isoToPose(const Eigen::Isometry3d &iso) {
  PoseEntry pose;
  pose.t = iso.translation();
  pose.q = Eigen::Quaterniond(iso.rotation());
  pose.q.normalize();
  return pose;
}

gtsam::Pose3 toGtsam(const PoseEntry &pose) {
  Eigen::Quaterniond q = pose.q;
  q.normalize();
  return gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()), gtsam::Point3(pose.t));
}

PoseEntry fromGtsam(const gtsam::Pose3 &pose) {
  PoseEntry out;
  out.t = Eigen::Vector3d(pose.x(), pose.y(), pose.z());
  out.q = Eigen::Quaterniond(pose.rotation().matrix());
  out.q.normalize();
  return out;
}

std::string fmt(double value) {
  std::ostringstream oss;
  oss << std::setprecision(15) << value;
  return oss.str();
}

fs::path mapDirFromRoot(const fs::path &input) {
  if (fs::exists(input / "pose_graph.g2o") && fs::exists(input / "pcd_buffer")) {
    return input;
  }
  return input / "Map";
}

bool readGraph(const fs::path &g2o_path, std::vector<PoseEntry> &poses,
               std::vector<EdgeEntry> &edges, std::vector<std::size_t> &fix_ids) {
  std::ifstream input(g2o_path);
  if (!input.is_open()) {
    std::cerr << "failed to open " << g2o_path << std::endl;
    return false;
  }

  std::string line;
  std::size_t max_id = 0;
  bool saw_vertex = false;
  std::vector<std::pair<std::size_t, PoseEntry>> parsed_poses;

  while (std::getline(input, line)) {
    const auto tokens = split(line);
    if (tokens.empty()) {
      continue;
    }
    if (tokens[0] == "VERTEX_SE3:QUAT" && tokens.size() >= 9) {
      std::size_t id = 0;
      if (!parseSize(tokens[1], id)) {
        std::cerr << "bad vertex id: " << line << std::endl;
        return false;
      }
      PoseEntry pose;
      pose.t = Eigen::Vector3d(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));
      pose.q = Eigen::Quaterniond(std::stod(tokens[8]), std::stod(tokens[5]),
                                  std::stod(tokens[6]), std::stod(tokens[7]));
      pose.q.normalize();
      max_id = std::max(max_id, id);
      saw_vertex = true;
      parsed_poses.emplace_back(id, pose);
    } else if (tokens[0] == "EDGE_SE3:QUAT" && tokens.size() >= 10) {
      EdgeEntry edge;
      if (!parseSize(tokens[1], edge.i) || !parseSize(tokens[2], edge.j)) {
        std::cerr << "bad edge id: " << line << std::endl;
        return false;
      }
      edge.measurement.t = Eigen::Vector3d(std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]));
      edge.measurement.q = Eigen::Quaterniond(std::stod(tokens[9]), std::stod(tokens[6]),
                                              std::stod(tokens[7]), std::stod(tokens[8]));
      edge.measurement.q.normalize();
      edge.is_loop = edge.i + 1 != edge.j;
      edges.push_back(edge);
    } else if (tokens[0] == "FIX" && tokens.size() >= 2) {
      std::size_t id = 0;
      if (parseSize(tokens[1], id)) {
        fix_ids.push_back(id);
      }
    }
  }

  if (!saw_vertex) {
    std::cerr << "no VERTEX_SE3:QUAT in " << g2o_path << std::endl;
    return false;
  }

  poses.assign(max_id + 1, PoseEntry{});
  std::vector<bool> exists(max_id + 1, false);
  for (const auto &item : parsed_poses) {
    poses[item.first] = item.second;
    exists[item.first] = true;
  }
  for (std::size_t i = 0; i < exists.size(); ++i) {
    if (!exists[i]) {
      std::cerr << "missing vertex id " << i << std::endl;
      return false;
    }
  }
  return true;
}

std::vector<fs::path> sortedPcdFiles(const fs::path &pcd_dir) {
  std::vector<std::pair<std::size_t, fs::path>> files;
  for (const auto &entry : fs::directory_iterator(pcd_dir)) {
    if (!entry.path().has_extension() || entry.path().extension() != ".pcd") {
      continue;
    }
    std::size_t id = 0;
    if (parseSize(entry.path().stem().string(), id)) {
      files.emplace_back(id, entry.path());
    }
  }
  std::sort(files.begin(), files.end(), [](const auto &a, const auto &b) {
    return a.first < b.first;
  });
  std::vector<fs::path> paths;
  paths.reserve(files.size());
  for (const auto &file : files) {
    paths.push_back(file.second);
  }
  return paths;
}

bool loadDownsampledGlobalCloud(const fs::path &pcd_path, const PoseEntry &pose,
                                double leaf,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path.string(), *raw) != 0) {
    std::cerr << "failed to load " << pcd_path << std::endl;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  if (leaf > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
    voxel.setInputCloud(raw);
    voxel.filter(*filtered);
  } else {
    filtered = raw;
  }

  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*filtered, *cloud, poseToIso(pose).matrix().cast<float>());
  return true;
}

bool estimateLoopMeasurement(const fs::path &pcd_dir, const std::vector<PoseEntry> &poses,
                             std::size_t fixed_id, std::size_t moving_id, double leaf,
                             PoseEntry &measurement) {
  const auto pcd_files = sortedPcdFiles(pcd_dir);
  if (pcd_files.size() != poses.size()) {
    std::cerr << "pcd count " << pcd_files.size() << " != pose count " << poses.size() << std::endl;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr fixed_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr moving_cloud;
  if (!loadDownsampledGlobalCloud(pcd_files.at(fixed_id), poses.at(fixed_id), leaf, fixed_cloud) ||
      !loadDownsampledGlobalCloud(pcd_files.at(moving_id), poses.at(moving_id), leaf, moving_cloud)) {
    return false;
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
  gicp.setInputSource(moving_cloud);
  gicp.setInputTarget(fixed_cloud);
  gicp.setMaximumIterations(80);
  gicp.setTransformationEpsilon(1e-6);
  gicp.setEuclideanFitnessEpsilon(1e-4);
  gicp.setMaxCorrespondenceDistance(std::max(2.0, leaf * 8.0));

  pcl::PointCloud<pcl::PointXYZI> aligned;
  gicp.align(aligned);
  if (!gicp.hasConverged()) {
    std::cerr << "GICP did not converge. score=" << gicp.getFitnessScore() << std::endl;
    return false;
  }

  const Eigen::Matrix4f correction_f = gicp.getFinalTransformation();
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.matrix() = correction_f.cast<double>();

  const Eigen::Isometry3d fixed_pose = poseToIso(poses.at(fixed_id));
  const Eigen::Isometry3d moving_pose = poseToIso(poses.at(moving_id));
  const Eigen::Isometry3d corrected_moving_pose = correction * moving_pose;
  measurement = isoToPose(fixed_pose.inverse() * corrected_moving_pose);

  const Eigen::Vector3d delta_t = correction.translation();
  const Eigen::AngleAxisd delta_r(correction.rotation());
  std::cout << "GICP converged. score=" << gicp.getFitnessScore()
            << " correction_translation_norm=" << delta_t.norm()
            << " correction_angle_rad=" << std::abs(delta_r.angle()) << std::endl;
  return true;
}

bool optimizeLoopOnly(const std::vector<PoseEntry> &initial_poses,
                      const std::vector<EdgeEntry> &edges,
                      const std::vector<std::size_t> &fix_ids,
                      std::size_t fixed_id, std::size_t moving_id,
                      const PoseEntry &loop_measurement,
                      double odom_t_sigma, double odom_r_sigma,
                      double loop_t_sigma, double loop_r_sigma,
                      std::vector<PoseEntry> &optimized_poses) {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  for (std::size_t i = 0; i < initial_poses.size(); ++i) {
    initial.insert(i, toGtsam(initial_poses[i]));
  }

  const gtsam::Vector6 prior_sigmas =
      (gtsam::Vector6() << odom_r_sigma, odom_r_sigma, odom_r_sigma,
                           odom_t_sigma, odom_t_sigma, odom_t_sigma).finished();
  const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);
  if (fix_ids.empty()) {
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, toGtsam(initial_poses[0]), prior_noise));
  } else {
    for (const auto id : fix_ids) {
      if (id < initial_poses.size()) {
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(id, toGtsam(initial_poses[id]), prior_noise));
      }
    }
  }
  if (fixed_id < initial_poses.size()) {
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(fixed_id, toGtsam(initial_poses[fixed_id]), prior_noise));
  }

  const gtsam::Vector6 odom_sigmas =
      (gtsam::Vector6() << odom_r_sigma, odom_r_sigma, odom_r_sigma,
                           odom_t_sigma, odom_t_sigma, odom_t_sigma).finished();
  const auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas);
  for (const auto &edge : edges) {
    if (edge.i >= initial_poses.size() || edge.j >= initial_poses.size()) {
      std::cerr << "skip out-of-range edge " << edge.i << " " << edge.j << std::endl;
      continue;
    }
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(edge.i, edge.j, toGtsam(edge.measurement), odom_noise));
  }

  const gtsam::Vector6 loop_sigmas =
      (gtsam::Vector6() << loop_r_sigma, loop_r_sigma, loop_r_sigma,
                           loop_t_sigma, loop_t_sigma, loop_t_sigma).finished();
  const auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(loop_sigmas);
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(fixed_id, moving_id, toGtsam(loop_measurement), loop_noise));

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setMaxIterations(100);
  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  optimized_poses.resize(initial_poses.size());
  for (std::size_t i = 0; i < initial_poses.size(); ++i) {
    optimized_poses[i] = fromGtsam(result.at<gtsam::Pose3>(i));
  }
  return true;
}

bool writeGraph(const fs::path &output_path, const std::vector<PoseEntry> &poses,
                const std::vector<EdgeEntry> &edges, std::size_t fixed_id,
                std::size_t moving_id, const PoseEntry &loop_measurement,
                const std::vector<std::size_t> &fix_ids) {
  std::ofstream output(output_path);
  if (!output.is_open()) {
    std::cerr << "failed to write " << output_path << std::endl;
    return false;
  }

  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto &pose = poses[i];
    output << "VERTEX_SE3:QUAT " << i << ' '
           << fmt(pose.t.x()) << ' ' << fmt(pose.t.y()) << ' ' << fmt(pose.t.z()) << ' '
           << fmt(pose.q.x()) << ' ' << fmt(pose.q.y()) << ' ' << fmt(pose.q.z()) << ' '
           << fmt(pose.q.w()) << '\n';
  }

  const std::string odom_info = "10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 1e+06 0 0 1e+06 0 1e+06";
  for (const auto &edge : edges) {
    const auto &m = edge.measurement;
    output << "EDGE_SE3:QUAT " << edge.i << ' ' << edge.j << ' '
           << fmt(m.t.x()) << ' ' << fmt(m.t.y()) << ' ' << fmt(m.t.z()) << ' '
           << fmt(m.q.x()) << ' ' << fmt(m.q.y()) << ' ' << fmt(m.q.z()) << ' '
           << fmt(m.q.w()) << ' ' << odom_info << '\n';
  }

  const std::string loop_info = "100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 400 0 0 400 0 400";
  output << "EDGE_SE3:QUAT " << fixed_id << ' ' << moving_id << ' '
         << fmt(loop_measurement.t.x()) << ' ' << fmt(loop_measurement.t.y()) << ' '
         << fmt(loop_measurement.t.z()) << ' ' << fmt(loop_measurement.q.x()) << ' '
         << fmt(loop_measurement.q.y()) << ' ' << fmt(loop_measurement.q.z()) << ' '
         << fmt(loop_measurement.q.w()) << ' ' << loop_info << '\n';

  if (fix_ids.empty()) {
    output << "FIX 0\n";
  } else {
    for (const auto id : fix_ids) {
      output << "FIX " << id << '\n';
    }
  }
  return true;
}

bool writeDeltaCsv(const fs::path &path, const std::vector<PoseEntry> &before,
                   const std::vector<PoseEntry> &after) {
  std::ofstream output(path);
  if (!output.is_open()) {
    return false;
  }
  output << "id,translation_delta_m,rotation_delta_rad,before_x,before_y,before_z,after_x,after_y,after_z\n";
  for (std::size_t i = 0; i < before.size(); ++i) {
    const Eigen::Vector3d dt = after[i].t - before[i].t;
    const Eigen::Quaterniond dq = before[i].q.conjugate() * after[i].q;
    const Eigen::AngleAxisd aa(dq.normalized());
    output << i << ',' << dt.norm() << ',' << std::abs(aa.angle()) << ','
           << before[i].t.x() << ',' << before[i].t.y() << ',' << before[i].t.z() << ','
           << after[i].t.x() << ',' << after[i].t.y() << ',' << after[i].t.z() << '\n';
  }
  return true;
}

int main(int argc, char **argv) {
  if (argc < 4 || argc > 8) {
    std::cerr << "usage: loop_optimize MAP_ROOT_OR_MAP_DIR fixed_id moving_id "
                 "[output_g2o] [icp_leaf=0.5] [loop_t_sigma=0.1] [loop_r_sigma=0.05]\n";
    return 2;
  }

  const fs::path map_dir = mapDirFromRoot(argv[1]);
  std::size_t fixed_id = 0;
  std::size_t moving_id = 0;
  if (!parseSize(argv[2], fixed_id) || !parseSize(argv[3], moving_id)) {
    std::cerr << "fixed_id and moving_id must be non-negative integers." << std::endl;
    return 2;
  }

  const fs::path output_g2o =
      argc >= 5 ? fs::path(argv[4]) : (map_dir / ("pose_graph_loop_" + std::to_string(fixed_id) + "_" +
                                                 std::to_string(moving_id) + ".g2o"));
  const double icp_leaf = argc >= 6 ? std::stod(argv[5]) : 0.5;
  const double loop_t_sigma = argc >= 7 ? std::stod(argv[6]) : 0.1;
  const double loop_r_sigma = argc >= 8 ? std::stod(argv[7]) : 0.05;

  std::vector<PoseEntry> poses;
  std::vector<EdgeEntry> edges;
  std::vector<std::size_t> fix_ids;
  if (!readGraph(map_dir / "pose_graph.g2o", poses, edges, fix_ids)) {
    return 1;
  }
  if (fixed_id >= poses.size() || moving_id >= poses.size()) {
    std::cerr << "id out of range. pose count=" << poses.size() << std::endl;
    return 2;
  }

  std::cout << "loaded graph: vertices=" << poses.size() << " se3_edges=" << edges.size()
            << " fix=" << fix_ids.size() << std::endl;
  std::cout << "GPS/scale factors are not loaded by this tool." << std::endl;

  PoseEntry loop_measurement;
  if (!estimateLoopMeasurement(map_dir / "pcd_buffer", poses, fixed_id, moving_id, icp_leaf,
                               loop_measurement)) {
    return 1;
  }

  std::vector<PoseEntry> optimized;
  if (!optimizeLoopOnly(poses, edges, fix_ids, fixed_id, moving_id, loop_measurement,
                        0.01, 0.001, loop_t_sigma, loop_r_sigma, optimized)) {
    return 1;
  }

  fs::create_directories(output_g2o.parent_path());
  if (!writeGraph(output_g2o, optimized, edges, fixed_id, moving_id, loop_measurement, fix_ids)) {
    return 1;
  }
  writeDeltaCsv(output_g2o.string() + ".delta.csv", poses, optimized);

  const Eigen::Vector3d fixed_delta = optimized[fixed_id].t - poses[fixed_id].t;
  const Eigen::Vector3d moving_delta = optimized[moving_id].t - poses[moving_id].t;
  std::cout << "fixed frame delta: " << fixed_delta.norm() << " m" << std::endl;
  std::cout << "moving frame delta: " << moving_delta.norm() << " m" << std::endl;
  std::cout << "wrote " << output_g2o << std::endl;
  std::cout << "wrote " << output_g2o.string() << ".delta.csv" << std::endl;
  return 0;
}
