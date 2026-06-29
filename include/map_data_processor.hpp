/**
 * @file map_data_processor.hpp
 * @brief 地图数据预处理：PCD 重命名/拷贝、G2O 位姿解析、空点云剔除与索引连续化
 */
#ifndef MAP_DATA_PROCESSOR_HPP
#define MAP_DATA_PROCESSOR_HPP

#include <string>
#include <vector>
#include <map>
#include <cstdint>

/**
 * @brief 地图数据预处理器
 *
 * 输入目录结构：
 *   map_root/pcd_buffer/*.pcd
 *   map_root/pose_graph.g2o
 *
 * 输出：
 *   map_root/pcd/NNNNN.pcd  （零填充、连续编号）
 *   map_root/pose.json      （tx ty tz qw qx qy qz，已相对首帧归一化）
 *   map_root/origin.json    （首帧原始位姿，归一化前）
 */
class MapDataProcessor
{
public:
  struct PoseEntry
  {
    int index = -1;
    double tx = 0, ty = 0, tz = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
  };

  struct Config
  {
    std::string map_root = "/home/jiang/project/map";
    std::string pcd_buffer_dir;   // 默认 map_root/pcd_buffer
    std::string pcd_output_dir;   // 默认 map_root/pcd
    std::string g2o_path;         // 默认 map_root/pose_graph.g2o
    std::string pose_json_path;   // 默认 map_root/pose.json
    std::string origin_json_path; // 默认 map_root/origin.json
    /** @brief 是否按首尾索引截取子集（true 时仅处理 [start_index, end_index]） */
    bool use_index_range = false;
    int start_index = 0;
    int end_index = 0;
  };

  explicit MapDataProcessor(const Config& config);

  /** @brief 执行预处理：全量或索引范围模式由 Config 决定 */
  bool process();

  /** @brief 获取处理摘要日志 */
  const std::string& summary() const { return summary_; }

  int removed_empty_count() const { return removed_empty_count_; }
  int output_count() const { return output_count_; }
  int name_fill_num() const { return name_fill_num_; }

private:
  Config config_;
  std::string summary_;
  int removed_empty_count_ = 0;
  int output_count_ = 0;
  int name_fill_num_ = 0;

  void initDefaultPaths();

  /** @brief 校验首尾索引参数 */
  bool validateIndexRange();

  /** @brief 全量预处理（含空点云剔除与 buffer 内重编号） */
  bool processFull();

  /** @brief 按索引范围截取子集（不修改 pcd_buffer） */
  bool processIndexRange();

  /** @brief 解析 pose_graph.g2o 中 VERTEX_SE3:QUAT 行 */
  bool parseG2o(std::map<int, PoseEntry>& poses);

  /** @brief 扫描 pcd_buffer，返回按索引排序的 PCD 文件路径 */
  bool collectPcdFiles(std::vector<std::pair<int, std::string>>& indexed_files);

  /** @brief 判断 PCD 是否无有效点云 */
  bool isPcdEmpty(const std::string& path);

  /** @brief 从文件名提取数字索引，如 "10005.pcd" -> 10005 */
  static bool parsePcdIndex(const std::string& filename, int& index);

  /** @brief 生成零填充文件名（不含路径），如 5 -> "00005.pcd" */
  static std::string makePaddedFilename(int index, int fill_num);

  /** @brief 删除空点云并在 pcd_buffer 中迁移后续文件名，保证索引连续 */
  bool removeEmptyAndMigrate(std::map<int, PoseEntry>& poses,
                             std::vector<std::pair<int, std::string>>& indexed_files);

  /** @brief 将有效 PCD 以零填充文件名拷贝到 pcd 目录 */
  bool copyPcdWithPadding(const std::vector<std::pair<int, std::string>>& indexed_files);

  /** @brief 写入首帧原始位姿到 origin.json */
  bool writeOriginJson(const PoseEntry& origin);

  /**
   * @brief 以首帧为参考归一化位姿，并写入 origin.json
   * pose.json 中第 0 帧为 t=0、R=I，其余为相对首帧位姿
   */
  bool normalizePosesToFirstFrame(std::vector<PoseEntry>& poses);

  /** @brief 写 pose.json（写入前自动做首帧归一化） */
  bool writePoseJson(std::vector<PoseEntry>& poses);

  /**
   * @brief 按原始索引对齐 PCD 与位姿，构建连续输出序列
   * @param indexed_files 输入：(原始索引, 路径)，输出：(0..N-1, 路径)
   * @param poses 原始索引 -> 位姿
   * @param output_poses 按输出顺序排列的位姿
   */
  bool buildAlignedOutput(const std::vector<std::pair<int, std::string>>& indexed_files,
                          const std::map<int, PoseEntry>& poses,
                          std::vector<std::pair<int, std::string>>& aligned_files,
                          std::vector<PoseEntry>& output_poses,
                          bool skip_empty_pcd);

  void appendSummary(const std::string& line);
};

#endif
