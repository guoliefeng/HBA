#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace fs = std::filesystem;

struct PoseEntry {
  int id = -1;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct EdgeEntry {
  int i = -1;
  int j = -1;
  Eigen::Isometry3d measurement = Eigen::Isometry3d::Identity();
  std::vector<std::string> info_tokens;
};

struct GpsEntry {
  int original_id = -1;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d information_diag = Eigen::Vector3d::Ones();
};

struct RouteRange {
  std::string route;
  int original_start = 0;
  int merged_start = 0;
  int count = 0;
};

std::vector<std::string> split(const std::string &line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) tokens.push_back(token);
  return tokens;
}

std::string fmt(double value) {
  std::ostringstream oss;
  oss << std::setprecision(15) << value;
  return oss.str();
}

Eigen::Isometry3d parsePose(const std::vector<std::string> &tokens, int offset) {
  Eigen::Quaterniond q(std::stod(tokens[offset + 6]), std::stod(tokens[offset + 3]),
                       std::stod(tokens[offset + 4]), std::stod(tokens[offset + 5]));
  q.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(std::stod(tokens[offset]), std::stod(tokens[offset + 1]),
                                       std::stod(tokens[offset + 2]));
  return pose;
}

gtsam::Pose3 toGtsam(const Eigen::Isometry3d &pose) {
  return gtsam::Pose3(gtsam::Rot3(pose.rotation()), gtsam::Point3(pose.translation()));
}

Eigen::Isometry3d fromGtsam(const gtsam::Pose3 &pose) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = pose.rotation().matrix();
  out.translation() = pose.translation();
  return out;
}

bool readGraph(const fs::path &path, std::vector<PoseEntry> &poses, std::vector<EdgeEntry> &edges) {
  std::ifstream input(path);
  if (!input.is_open()) {
    std::cerr << "[ERROR] failed to open " << path << std::endl;
    return false;
  }

  std::string line;
  int max_id = -1;
  std::map<int, PoseEntry> pose_by_id;
  while (std::getline(input, line)) {
    const auto tokens = split(line);
    if (tokens.empty()) continue;
    if (tokens[0] == "VERTEX_SE3:QUAT" && tokens.size() >= 9) {
      PoseEntry p;
      p.id = std::stoi(tokens[1]);
      p.pose = parsePose(tokens, 2);
      pose_by_id[p.id] = p;
      max_id = std::max(max_id, p.id);
    } else if (tokens[0] == "EDGE_SE3:QUAT" && tokens.size() >= 10) {
      EdgeEntry e;
      e.i = std::stoi(tokens[1]);
      e.j = std::stoi(tokens[2]);
      e.measurement = parsePose(tokens, 3);
      e.info_tokens.assign(tokens.begin() + 10, tokens.end());
      edges.push_back(e);
    }
  }

  if (max_id < 0) return false;
  poses.assign(static_cast<size_t>(max_id + 1), PoseEntry());
  for (int i = 0; i <= max_id; ++i) {
    auto it = pose_by_id.find(i);
    if (it == pose_by_id.end()) {
      std::cerr << "[ERROR] missing pose id " << i << " in " << path << std::endl;
      return false;
    }
    poses[static_cast<size_t>(i)] = it->second;
  }
  return true;
}

std::unordered_map<int, GpsEntry> readGpsByOriginalId(const fs::path &original_g2o) {
  std::unordered_map<int, GpsEntry> gps;
  std::ifstream input(original_g2o);
  std::string line;
  while (std::getline(input, line)) {
    const auto tokens = split(line);
    if (tokens.size() < 15 || tokens[0] != "EDGE_DIS:VEC3") continue;
    GpsEntry e;
    e.original_id = std::stoi(tokens[1]);
    e.xyz = Eigen::Vector3d(std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]));
    e.information_diag = Eigen::Vector3d(std::stod(tokens[6]), std::stod(tokens[10]),
                                         std::stod(tokens[14]));
    gps[e.original_id] = e;
  }
  return gps;
}

std::vector<RouteRange> readRouteRanges(const fs::path &ranges_csv, const fs::path &manifest_csv) {
  std::unordered_map<std::string, int> original_start;
  std::ifstream ranges(ranges_csv);
  std::string line;
  std::getline(ranges, line);
  while (std::getline(ranges, line)) {
    const auto tokens = split(line);
    if (tokens.empty()) continue;
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) cells.push_back(cell);
    if (cells.size() >= 3) original_start[cells[0]] = std::stoi(cells[1]);
  }

  std::vector<RouteRange> out;
  std::ifstream manifest(manifest_csv);
  std::getline(manifest, line);
  while (std::getline(manifest, line)) {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) cells.push_back(cell);
    if (cells.size() < 4) continue;
    RouteRange r;
    r.route = cells[0];
    r.merged_start = std::stoi(cells[1]);
    r.count = std::stoi(cells[3]);
    r.original_start = original_start.at(r.route);
    out.push_back(r);
  }
  return out;
}

std::unordered_map<int, GpsEntry> mapGpsToMergedIds(
    const std::vector<RouteRange> &routes,
    const std::unordered_map<int, GpsEntry> &gps_by_original_id) {
  std::unordered_map<int, GpsEntry> out;
  for (const auto &route : routes) {
    for (int local = 0; local < route.count; ++local) {
      const int merged_id = route.merged_start + local;
      const int original_id = route.original_start + local;
      auto it = gps_by_original_id.find(original_id);
      if (it != gps_by_original_id.end()) out[merged_id] = it->second;
    }
  }
  return out;
}

double gpsRmse(const std::vector<PoseEntry> &poses, const std::unordered_map<int, GpsEntry> &gps) {
  double sum = 0.0;
  size_t count = 0;
  for (const auto &item : gps) {
    const int id = item.first;
    if (id < 0 || static_cast<size_t>(id) >= poses.size()) continue;
    const Eigen::Vector3d r = poses[id].pose.translation() - item.second.xyz;
    sum += r.squaredNorm();
    ++count;
  }
  return count > 0 ? std::sqrt(sum / count) : 0.0;
}

std::vector<PoseEntry> optimizeGps(const std::vector<PoseEntry> &initial,
                                   const std::vector<EdgeEntry> &edges,
                                   const std::unordered_map<int, GpsEntry> &gps,
                                   int iterations) {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  for (const auto &p : initial) values.insert(p.id, toGtsam(p.pose));

  const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, toGtsam(initial.front().pose), prior_noise));

  const auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.2, 0.2, 0.2).finished());
  for (const auto &edge : edges) {
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(edge.i, edge.j, toGtsam(edge.measurement),
                                                 odom_noise));
  }

  for (const auto &item : gps) {
    const int id = item.first;
    const auto &g = item.second;
    Eigen::Vector3d sigma;
    for (int k = 0; k < 3; ++k) {
      sigma[k] = g.information_diag[k] > 0.0 ? 1.0 / std::sqrt(g.information_diag[k]) : 1.0;
    }
    const auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << sigma.x(), sigma.y(), sigma.z()).finished());
    graph.add(gtsam::GPSFactor(id, gtsam::Point3(g.xyz), gps_noise));
  }

  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(iterations);
  params.setVerbosityLM("SUMMARY");
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, values, params).optimize();

  std::vector<PoseEntry> optimized = initial;
  for (auto &p : optimized) p.pose = fromGtsam(result.at<gtsam::Pose3>(p.id));
  return optimized;
}

bool writeGraph(const fs::path &path, const std::vector<PoseEntry> &poses,
                const std::vector<EdgeEntry> &edges) {
  std::ofstream output(path);
  if (!output.is_open()) return false;
  for (const auto &p : poses) {
    const Eigen::Quaterniond q(p.pose.rotation());
    output << "VERTEX_SE3:QUAT " << p.id << ' '
           << fmt(p.pose.translation().x()) << ' '
           << fmt(p.pose.translation().y()) << ' '
           << fmt(p.pose.translation().z()) << ' '
           << fmt(q.x()) << ' ' << fmt(q.y()) << ' '
           << fmt(q.z()) << ' ' << fmt(q.w()) << '\n';
  }
  for (const auto &e : edges) {
    const Eigen::Quaterniond q(e.measurement.rotation());
    output << "EDGE_SE3:QUAT " << e.i << ' ' << e.j << ' '
           << fmt(e.measurement.translation().x()) << ' '
           << fmt(e.measurement.translation().y()) << ' '
           << fmt(e.measurement.translation().z()) << ' '
           << fmt(q.x()) << ' ' << fmt(q.y()) << ' '
           << fmt(q.z()) << ' ' << fmt(q.w());
    for (const auto &token : e.info_tokens) output << ' ' << token;
    output << '\n';
  }
  output << "FIX 0\n";
  return true;
}

void writeDeltaCsv(const fs::path &path, const std::vector<PoseEntry> &before,
                   const std::vector<PoseEntry> &after,
                   const std::unordered_map<int, GpsEntry> &gps) {
  std::ofstream output(path);
  output << "id,dx,dy,dz,dt,before_gps_residual,after_gps_residual\n";
  for (size_t i = 0; i < before.size(); ++i) {
    const Eigen::Vector3d d = after[i].pose.translation() - before[i].pose.translation();
    double before_res = -1.0;
    double after_res = -1.0;
    auto it = gps.find(static_cast<int>(i));
    if (it != gps.end()) {
      before_res = (before[i].pose.translation() - it->second.xyz).norm();
      after_res = (after[i].pose.translation() - it->second.xyz).norm();
    }
    output << i << ',' << d.x() << ',' << d.y() << ',' << d.z() << ',' << d.norm() << ','
           << before_res << ',' << after_res << '\n';
  }
}

int main(int argc, char **argv) {
  if (argc < 5 || argc > 6) {
    std::cerr << "usage: gps_optimize MERGED_MAP_DIR ORIGINAL_G2O ROUTE_RANGES_CSV OUTPUT_DIR "
                 "[iterations=100]\n";
    return 2;
  }

  const fs::path map_dir = argv[1];
  const fs::path original_g2o = argv[2];
  const fs::path route_ranges = argv[3];
  const fs::path output_dir = argv[4];
  const int iterations = argc >= 6 ? std::stoi(argv[5]) : 100;
  fs::create_directories(output_dir / "Map");

  std::vector<PoseEntry> poses;
  std::vector<EdgeEntry> edges;
  if (!readGraph(map_dir / "pose_graph.g2o", poses, edges)) return 1;

  const auto gps_original = readGpsByOriginalId(original_g2o);
  const auto routes = readRouteRanges(route_ranges, map_dir.parent_path() / "manifest.csv");
  const auto gps_merged = mapGpsToMergedIds(routes, gps_original);
  std::cout << "[INFO] poses=" << poses.size() << " edges=" << edges.size()
            << " gps_factors=" << gps_merged.size() << std::endl;
  std::cout << "[INFO] gps_rmse_before=" << gpsRmse(poses, gps_merged) << std::endl;

  const auto optimized = optimizeGps(poses, edges, gps_merged, iterations);
  std::cout << "[INFO] gps_rmse_after=" << gpsRmse(optimized, gps_merged) << std::endl;

  if (!writeGraph(output_dir / "Map" / "pose_graph.g2o", optimized, edges)) return 1;
  fs::copy_file(output_dir / "Map" / "pose_graph.g2o",
                output_dir / "Map" / "pose_graph.gps_optimized.g2o",
                fs::copy_options::overwrite_existing);
  writeDeltaCsv(output_dir / "pose_delta_gps.csv", poses, optimized, gps_merged);
  fs::copy_file(map_dir.parent_path() / "manifest.csv", output_dir / "manifest.csv",
                fs::copy_options::overwrite_existing);
  if (fs::exists(map_dir / "origin.txt")) {
    fs::copy_file(map_dir / "origin.txt", output_dir / "Map" / "origin.txt",
                  fs::copy_options::overwrite_existing);
  }

  const fs::path src_pcd = map_dir / "pcd_buffer";
  const fs::path dst_pcd = output_dir / "Map" / "pcd_buffer";
  fs::create_directories(dst_pcd);
  for (size_t i = 0; i < poses.size(); ++i) {
    const fs::path src = src_pcd / (std::to_string(i) + ".pcd");
    const fs::path dst = dst_pcd / (std::to_string(i) + ".pcd");
    if (!fs::exists(dst)) fs::create_symlink(src, dst);
  }

  return 0;
}
