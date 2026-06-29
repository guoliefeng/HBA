/**
 * @file map_data_processor.cpp
 * @brief MapDataProcessor 实现
 */
#include "map_data_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = boost::filesystem;

MapDataProcessor::MapDataProcessor(const Config& config)
  : config_(config)
{
  initDefaultPaths();
}

void MapDataProcessor::initDefaultPaths()
{
  if(config_.pcd_buffer_dir.empty())
    config_.pcd_buffer_dir = config_.map_root + "/pcd_buffer";
  if(config_.pcd_output_dir.empty())
    config_.pcd_output_dir = config_.map_root + "/pcd";
  if(config_.g2o_path.empty())
    config_.g2o_path = config_.map_root + "/pose_graph.g2o";
  if(config_.pose_json_path.empty())
    config_.pose_json_path = config_.map_root + "/pose.json";
  if(config_.origin_json_path.empty())
    config_.origin_json_path = config_.map_root + "/origin.json";
}

void MapDataProcessor::appendSummary(const std::string& line)
{
  summary_ += line + "\n";
  std::cout << line << std::endl;
}

bool MapDataProcessor::validateIndexRange()
{
  if(config_.start_index < 0 || config_.end_index < 0)
  {
    appendSummary("[ERROR] index range must be non-negative");
    return false;
  }
  if(config_.start_index > config_.end_index)
  {
    appendSummary("[ERROR] start_index must be <= end_index");
    return false;
  }
  return true;
}

bool MapDataProcessor::parsePcdIndex(const std::string& filename, int& index)
{
  const std::string stem = fs::path(filename).stem().string();
  if(stem.empty()) return false;
  try
  {
    size_t pos = 0;
    long value = std::stol(stem, &pos);
    if(pos != stem.size() || value < 0) return false;
    index = static_cast<int>(value);
    return true;
  }
  catch(...)
  {
    return false;
  }
}

std::string MapDataProcessor::makePaddedFilename(int index, int fill_num)
{
  std::ostringstream ss;
  ss << std::setw(fill_num) << std::setfill('0') << index << ".pcd";
  return ss.str();
}

bool MapDataProcessor::parseG2o(std::map<int, PoseEntry>& poses)
{
  std::ifstream file(config_.g2o_path);
  if(!file.is_open())
  {
    appendSummary("[ERROR] cannot open g2o: " + config_.g2o_path);
    return false;
  }

  const std::string prefix = "VERTEX_SE3:QUAT";
  std::string line;
  int parsed = 0;
  int parsed_in_range = 0;
  while(std::getline(file, line))
  {
    if(line.compare(0, prefix.size(), prefix) != 0)
      continue;

    std::istringstream iss(line.substr(prefix.size()));
    PoseEntry pose;
    if(!(iss >> pose.index >> pose.tx >> pose.ty >> pose.tz
              >> pose.qx >> pose.qy >> pose.qz >> pose.qw))
    {
      appendSummary("[WARN] skip malformed g2o line: " + line);
      continue;
    }

    if(config_.use_index_range)
    {
      if(pose.index < config_.start_index || pose.index > config_.end_index)
        continue;
      parsed_in_range++;
    }

    poses[pose.index] = pose;
    parsed++;
  }

  if(config_.use_index_range)
  {
    appendSummary("[INFO] parsed g2o poses in range [" + std::to_string(config_.start_index)
                  + ", " + std::to_string(config_.end_index) + "]: "
                  + std::to_string(parsed_in_range));
  }
  else
  {
    appendSummary("[INFO] parsed g2o poses: " + std::to_string(parsed));
  }
  return !poses.empty();
}

bool MapDataProcessor::collectPcdFiles(std::vector<std::pair<int, std::string>>& indexed_files)
{
  indexed_files.clear();
  if(!fs::exists(config_.pcd_buffer_dir) || !fs::is_directory(config_.pcd_buffer_dir))
  {
    appendSummary("[ERROR] pcd_buffer not found: " + config_.pcd_buffer_dir);
    return false;
  }

  for(const auto& entry : fs::directory_iterator(config_.pcd_buffer_dir))
  {
    if(!fs::is_regular_file(entry.path())) continue;
    if(entry.path().extension() != ".pcd") continue;

    int index = -1;
    if(!parsePcdIndex(entry.path().filename().string(), index))
    {
      appendSummary("[WARN] skip non-index pcd: " + entry.path().string());
      continue;
    }

    if(config_.use_index_range)
    {
      if(index < config_.start_index || index > config_.end_index)
        continue;
    }

    indexed_files.emplace_back(index, entry.path().string());
  }

  std::sort(indexed_files.begin(), indexed_files.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });

  if(config_.use_index_range)
  {
    appendSummary("[INFO] pcd files in buffer within range: "
                  + std::to_string(indexed_files.size()));
  }
  else
  {
    appendSummary("[INFO] pcd files in buffer: " + std::to_string(indexed_files.size()));
  }
  return !indexed_files.empty();
}

bool MapDataProcessor::isPcdEmpty(const std::string& path)
{
  if(!fs::exists(path))
    return true;

  // 极小文件通常没有有效点云
  if(fs::file_size(path) < 128)
    return true;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if(pcl::io::loadPCDFile(path, cloud) != 0)
  {
    appendSummary("[WARN] failed to load pcd, treat as empty: " + path);
    return true;
  }
  return cloud.empty();
}

bool MapDataProcessor::buildAlignedOutput(
    const std::vector<std::pair<int, std::string>>& indexed_files,
    const std::map<int, PoseEntry>& poses,
    std::vector<std::pair<int, std::string>>& aligned_files,
    std::vector<PoseEntry>& output_poses,
    bool skip_empty_pcd)
{
  aligned_files.clear();
  output_poses.clear();
  int missing_pose = 0;
  int skipped_empty = 0;

  for(const auto& item : indexed_files)
  {
    if(skip_empty_pcd && isPcdEmpty(item.second))
    {
      skipped_empty++;
      appendSummary("[WARN] skip empty pcd index " + std::to_string(item.first));
      continue;
    }

    auto it = poses.find(item.first);
    if(it == poses.end())
    {
      missing_pose++;
      appendSummary("[WARN] missing g2o pose for pcd index " + std::to_string(item.first));
      continue;
    }

    const int out_idx = static_cast<int>(aligned_files.size());
    aligned_files.emplace_back(out_idx, item.second);
    PoseEntry pose = it->second;
    pose.index = out_idx;
    output_poses.push_back(pose);
  }

  if(!output_poses.empty())
  {
    appendSummary("[INFO] aligned output count: " + std::to_string(output_poses.size()));
    if(missing_pose > 0)
      appendSummary("[WARN] missing pose count: " + std::to_string(missing_pose));
    if(skipped_empty > 0)
      appendSummary("[INFO] skipped empty pcd: " + std::to_string(skipped_empty));
    return true;
  }

  appendSummary("[ERROR] no valid pose-pcd pairs");
  return false;
}

bool MapDataProcessor::removeEmptyAndMigrate(std::map<int, PoseEntry>& poses,
                                             std::vector<std::pair<int, std::string>>& indexed_files)
{
  removed_empty_count_ = 0;

  // 按索引升序检测空点云；删除后从高索引向低索引重命名，避免覆盖
  std::vector<int> empty_indices;
  for(const auto& item : indexed_files)
  {
    if(isPcdEmpty(item.second))
      empty_indices.push_back(item.first);
  }

  if(empty_indices.empty())
  {
    appendSummary("[INFO] no empty pcd found");
    return true;
  }

  std::sort(empty_indices.begin(), empty_indices.end());
  appendSummary("[INFO] empty pcd count: " + std::to_string(empty_indices.size()));

  for(int empty_idx : empty_indices)
  {
    auto file_it = std::find_if(indexed_files.begin(), indexed_files.end(),
                                [empty_idx](const auto& p){ return p.first == empty_idx; });
    if(file_it != indexed_files.end())
    {
      fs::remove(file_it->second);
      indexed_files.erase(file_it);
    }
    poses.erase(empty_idx);
    removed_empty_count_++;
  }

  // 重新收集 buffer 中剩余文件，按当前文件名索引排序
  indexed_files.clear();
  if(!collectPcdFiles(indexed_files))
    return false;

  // 若索引不连续，则在 buffer 内迁移文件名使其连续（从大到小重命名）
  std::vector<std::pair<int, std::string>> sorted = indexed_files;
  std::sort(sorted.begin(), sorted.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });

  const int target_count = static_cast<int>(sorted.size());
  bool need_migrate = false;
  for(int i = 0; i < target_count; ++i)
  {
    if(sorted[i].first != i)
    {
      need_migrate = true;
      break;
    }
  }

  if(!need_migrate)
  {
    appendSummary("[INFO] pcd_buffer indices already contiguous after deletion");
    return true;
  }

  // 建立 old_index -> path 映射
  std::map<int, std::string> index_to_path;
  for(const auto& item : sorted)
    index_to_path[item.first] = item.second;

  // 临时后缀，防止重命名冲突
  const std::string tmp_suffix = ".hba_tmp";
  std::map<int, std::string> old_to_tmp;
  for(const auto& kv : index_to_path)
  {
    const std::string tmp_path = kv.second + tmp_suffix;
    fs::rename(kv.second, tmp_path);
    old_to_tmp[kv.first] = tmp_path;
  }

  // 按新连续索引 0..N-1 写回，并同步迁移 poses
  std::map<int, PoseEntry> new_poses;
  indexed_files.clear();

  for(int new_idx = 0; new_idx < target_count; ++new_idx)
  {
    const int old_idx = sorted[new_idx].first;
    const std::string final_path = config_.pcd_buffer_dir + "/" + std::to_string(new_idx) + ".pcd";
    fs::rename(old_to_tmp[old_idx], final_path);
    indexed_files.emplace_back(new_idx, final_path);

    auto pose_it = poses.find(old_idx);
    if(pose_it != poses.end())
      new_poses[new_idx] = pose_it->second;
  }

  poses.swap(new_poses);
  for(auto& kv : poses)
    kv.second.index = kv.first;

  appendSummary("[INFO] migrated pcd_buffer filenames to contiguous indices 0.."
                + std::to_string(target_count - 1));
  return true;
}

bool MapDataProcessor::copyPcdWithPadding(const std::vector<std::pair<int, std::string>>& indexed_files)
{
  if(indexed_files.empty())
  {
    appendSummary("[ERROR] no pcd to copy");
    return false;
  }

  fs::create_directories(config_.pcd_output_dir);

  // 清空旧输出，避免残留文件干扰
  if(fs::exists(config_.pcd_output_dir))
  {
    for(const auto& entry : fs::directory_iterator(config_.pcd_output_dir))
    {
      if(fs::is_regular_file(entry.path()) && entry.path().extension() == ".pcd")
        fs::remove(entry.path());
    }
  }

  const int output_count = static_cast<int>(indexed_files.size());
  name_fill_num_ = std::max(1, static_cast<int>(std::log10(std::max(0, output_count - 1))) + 1);

  for(int new_idx = 0; new_idx < output_count; ++new_idx)
  {
    const std::string src = indexed_files[new_idx].second;
    const std::string dst = config_.pcd_output_dir + "/" + makePaddedFilename(new_idx, name_fill_num_);
    fs::copy_file(src, dst, fs::copy_option::overwrite_if_exists);
  }

  output_count_ = output_count;
  appendSummary("[INFO] copied pcd to: " + config_.pcd_output_dir);
  appendSummary("[INFO] output pcd count: " + std::to_string(output_count_));
  appendSummary("[INFO] name fill num: " + std::to_string(name_fill_num_));
  return true;
}

bool MapDataProcessor::writeOriginJson(const PoseEntry& origin)
{
  std::ofstream file(config_.origin_json_path, std::ios::trunc);
  if(!file.is_open())
  {
    appendSummary("[ERROR] cannot write origin.json: " + config_.origin_json_path);
    return false;
  }

  file << origin.tx << " " << origin.ty << " " << origin.tz << " "
       << origin.qw << " " << origin.qx << " " << origin.qy << " " << origin.qz;
  appendSummary("[INFO] wrote origin.json: " + config_.origin_json_path);
  return true;
}

bool MapDataProcessor::normalizePosesToFirstFrame(std::vector<PoseEntry>& poses)
{
  if(poses.empty())
  {
    appendSummary("[ERROR] no pose to normalize");
    return false;
  }

  const PoseEntry origin = poses[0];
  if(!writeOriginJson(origin))
    return false;

  Eigen::Quaterniond q0(origin.qw, origin.qx, origin.qy, origin.qz);
  q0.normalize();
  const Eigen::Vector3d t0(origin.tx, origin.ty, origin.tz);

  for(size_t i = 0; i < poses.size(); ++i)
  {
    Eigen::Quaterniond qi(poses[i].qw, poses[i].qx, poses[i].qy, poses[i].qz);
    qi.normalize();
    const Eigen::Vector3d ti(poses[i].tx, poses[i].ty, poses[i].tz);

    const Eigen::Quaterniond qn = q0.inverse() * qi;
    const Eigen::Vector3d tn = q0.inverse() * (ti - t0);

    poses[i].tx = tn.x();
    poses[i].ty = tn.y();
    poses[i].tz = tn.z();
    poses[i].qw = qn.w();
    poses[i].qx = qn.x();
    poses[i].qy = qn.y();
    poses[i].qz = qn.z();
  }

  appendSummary("[INFO] normalized poses to first frame (origin saved separately)");
  return true;
}

bool MapDataProcessor::writePoseJson(std::vector<PoseEntry>& poses)
{
  if(poses.empty())
  {
    appendSummary("[ERROR] no pose to write");
    return false;
  }

  if(!normalizePosesToFirstFrame(poses))
    return false;

  std::ofstream file(config_.pose_json_path, std::ios::trunc);
  if(!file.is_open())
  {
    appendSummary("[ERROR] cannot write pose.json: " + config_.pose_json_path);
    return false;
  }

  // HBA 格式: tx ty tz qw qx qy qz（已相对首帧归一化）
  for(size_t i = 0; i < poses.size(); ++i)
  {
    const PoseEntry& p = poses[i];
    file << p.tx << " " << p.ty << " " << p.tz << " "
         << p.qw << " " << p.qx << " " << p.qy << " " << p.qz;
    if(i + 1 < poses.size()) file << "\n";
  }

  appendSummary("[INFO] wrote pose.json: " + config_.pose_json_path);
  appendSummary("[INFO] pose count: " + std::to_string(poses.size()));
  return true;
}

bool MapDataProcessor::processIndexRange()
{
  if(!validateIndexRange())
    return false;

  appendSummary("[INFO] index range mode: [" + std::to_string(config_.start_index)
                + ", " + std::to_string(config_.end_index) + "]");

  std::map<int, PoseEntry> poses;
  if(!parseG2o(poses))
    return false;

  std::vector<std::pair<int, std::string>> indexed_files;
  if(!collectPcdFiles(indexed_files))
    return false;

  std::vector<std::pair<int, std::string>> aligned_files;
  std::vector<PoseEntry> output_poses;
  if(!buildAlignedOutput(indexed_files, poses, aligned_files, output_poses, true))
    return false;

  if(!copyPcdWithPadding(aligned_files))
    return false;

  if(!writePoseJson(output_poses))
    return false;

  appendSummary("[DONE] index range extract success");
  appendSummary("[DONE] output count: " + std::to_string(output_count_));
  return true;
}

bool MapDataProcessor::processFull()
{
  std::map<int, PoseEntry> poses;
  if(!parseG2o(poses))
    return false;

  std::vector<std::pair<int, std::string>> indexed_files;
  if(!collectPcdFiles(indexed_files))
    return false;

  if(!removeEmptyAndMigrate(poses, indexed_files))
    return false;

  indexed_files.clear();
  if(!collectPcdFiles(indexed_files))
    return false;

  std::vector<std::pair<int, std::string>> aligned_files;
  std::vector<PoseEntry> output_poses;
  if(!buildAlignedOutput(indexed_files, poses, aligned_files, output_poses, false))
    return false;

  if(!copyPcdWithPadding(aligned_files))
    return false;

  if(!writePoseJson(output_poses))
    return false;

  appendSummary("[DONE] preprocess success");
  appendSummary("[DONE] removed empty: " + std::to_string(removed_empty_count_));
  return true;
}

bool MapDataProcessor::process()
{
  summary_.clear();
  removed_empty_count_ = 0;
  output_count_ = 0;
  name_fill_num_ = 0;

  appendSummary("[INFO] map_root: " + config_.map_root);
  appendSummary("[INFO] pcd_buffer: " + config_.pcd_buffer_dir);
  appendSummary("[INFO] pcd output: " + config_.pcd_output_dir);
  appendSummary("[INFO] g2o: " + config_.g2o_path);

  if(config_.use_index_range)
    return processIndexRange();
  return processFull();
}

// ---------------------------------------------------------------------------
// 独立可执行入口（不修改项目 CMakeLists 时可直接编译本文件）
// ---------------------------------------------------------------------------
#ifndef MAP_DATA_PROCESSOR_NO_MAIN
int main(int argc, char** argv)
{
  MapDataProcessor::Config config;
  if(argc > 1) config.map_root = argv[1];
  if(argc >= 4)
  {
    config.use_index_range = true;
    config.start_index = std::stoi(argv[2]);
    config.end_index = std::stoi(argv[3]);
  }

  MapDataProcessor processor(config);
  const bool ok = processor.process();
  if(!ok)
  {
    std::cerr << processor.summary();
    return 1;
  }
  return 0;
}
#endif
