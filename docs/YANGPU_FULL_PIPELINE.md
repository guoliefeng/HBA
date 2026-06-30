# Yangpu 13 Routes Full Pipeline

本文档整理当前洋浦数据从原始 `pose_graph.g2o + pcd_buffer` 到 HBA、多路线对齐、GPS 因子优化、重建 `map.pcd` 的完整流程。

## 1. 总体流程

```text
/home/glf/dataDisk/hainan/yangpu/HBA
  pose_graph.g2o + origin.txt + pcd_buffer/
        |
        | 1) 按哨兵空帧切成 13 条路线
        | 2) map_preprocess：每条路线生成 HBA 输入
        v
routes/route_xx_xxxxx_xxxxx/
  pcd/ + pose_before_hba.json + origin.json
        |
        | 3) hba：每条路线单独优化
        v
manual_hba/route_xx/Map/
  pose_graph.g2o + pcd_buffer/ + map.pcd
        |
        | 4) 轨迹间点云匹配/CloudCompare：估计 route_i -> 已合并地图 的 T
        | 5) 将 route_i 的 pose_graph 顶点左乘 T，pcd_buffer 连续链接
        v
manual_hba/route_01_..._13_cloudcompare_track_aligned_pose_graph/Map/
  pose_graph.g2o + pcd_buffer/ + map.pcd
        |
        | 6) gps_optimize：原始 GPS 因子残差最小化
        | 7) 用优化后的 pose_graph 重新拼 map.pcd
        v
manual_hba/route_01_..._13_cloudcompare_gps_optimized_pose_graph/Map/
  pose_graph.g2o + map.pcd + raw_map.pcd + ground_map.pcd
```

核心原则：

- HBA 只处理单条路线内的局部一致性，降低路线内重影；
- 路线之间不直接依赖 HBA 自动对齐，而是用点云匹配或 CloudCompare 得到刚体变换 `T`；
- 多路线拼接时不改原始点云文件，只改每帧 pose，并把 `pcd_buffer` 做连续编号链接；
- 最后用原始 g2o 里的 `EDGE_DIS:VEC3` GPS 因子约束全局漂移，再按优化 pose 重建地图。

## 2. 原始输入

原始数据目录：

```text
/home/glf/dataDisk/hainan/yangpu/HBA/
├── pose_graph.g2o
├── origin.txt
└── pcd_buffer/
```

来源：

```bash
cd ~/proj/gps_mapping_ws
roslaunch fast_lio_sam_loop run_yangpu_inc.launch
```

原始 `pose_graph.g2o` 中有三类关键数据：

```text
VERTEX_SE3:QUAT id tx ty tz qx qy qz qw
EDGE_SE3:QUAT id_i id_j relative_pose information
EDGE_DIS:VEC3 id scale_id gps_x gps_y gps_z information
```

其中 `EDGE_DIS:VEC3` 是 GPS 位置因子，后续 GPS 优化会重新使用。

## 3. 路线分段与 HBA 输入

洋浦数据中有 13 个单点全零 PCD，作为路线分隔哨兵帧。分段结果记录在：

```text
docs/YANGPU_ROUTES.md
/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_ranges.csv
```

每条路线执行 `map_preprocess` 后，得到：

```text
routes/route_xx_xxxxx_xxxxx/
├── pcd/
├── pose.json
├── pose_before_hba.json
└── origin.json
```

`map_preprocess` 做的事情：

- 从原始 `pcd_buffer/` 读取对应索引范围的 PCD；
- 跳过哨兵空帧；
- 从原始 `pose_graph.g2o` 读取对应 `VERTEX_SE3:QUAT`；
- 重新编号为 `0 ... N-1`；
- 将首帧作为局部原点，生成 HBA 使用的相对 `pose.json`；
- 保存首帧原始位姿到 `origin.json`。

## 4. 单路线 HBA 优化

每条路线独立运行 HBA：

```bash
roslaunch hba hba.launch
```

输入：

```text
route_xx/
├── pcd/
└── pose.json              # HBA 前的相对位姿
```

输出：

```text
route_xx/
├── pose_before_hba.json   # 备份，优化前
├── pose_after_hba.json    # HBA 后
└── pose.json              # 当前与 pose_after_hba.json 一致
```

HBA 的目标是利用点云局部几何约束优化每条路线内部位姿，让同一路线内的点云重影减少。但它不保证多条路线之间天然对齐。

## 5. HBA 结果反推成 Map/pose_graph.g2o

为了后续用 `internal_auto_mapping` 或自定义工具继续处理，需要把 HBA 后的 `pose.json + origin.json` 重新转成 `Map/pose_graph.g2o`。

输出约定：

```text
manual_hba/route_xx/Map/
├── pose_graph.g2o
├── pose_graph.hba_consistent.g2o
├── pcd_buffer/
├── map.pcd
├── raw_map.pcd
└── ground_map.pcd
```

这里的 `pose_graph.g2o` 与原始 g2o 的关系：

- 顶点 `VERTEX_SE3:QUAT` 来自 HBA 优化后的全局 pose；
- 相邻边 `EDGE_SE3:QUAT` 按优化后相邻顶点重新计算或保留一致关系；
- `pcd_buffer/` 是连续编号的帧点云；
- `map.pcd` 是按该 pose graph 重新拼出的地图。

## 6. 多路线点云匹配与 T 记录

路线间对齐采用增量合并：

```text
route_01 + route_02 -> route_01_02
route_03 registered 到 route_01_02
route_04 registered 到 route_01_02_03
...
route_13 registered 到 route_01_..._12
```

对每条新路线，需要估计：

```text
T_route_i_to_current_map
```

含义：

```text
p_current_map = T_route_i_to_current_map * p_route_i
```

T 的来源有两种：

- `track_align`：读取两个 Map 目录的 `map_seg.pcd` 或 `map.pcd`，用 GICP 估计 T；
- CloudCompare：手动裁剪/配准后，把 CloudCompare 输出的 4x4 矩阵作为 T。

CloudCompare 粘贴出来的矩阵通常是文本四舍五入结果，3x3 旋转块可能不是严格正交。写回 g2o 时会对旋转块做 SVD 正交化，平移保持不变。

## 7. 用 T 拼接 pose_graph 和 pcd_buffer

对 route_i 追加到累计地图时，执行三件事。

第一，变换 route_i 的所有顶点：

```text
P_i_aligned = T_route_i_to_current_map * P_i_hba
```

写成新的：

```text
VERTEX_SE3:QUAT merged_id tx ty tz qx qy qz qw
```

第二，保留 route_i 内部相邻边，只做 ID 偏移：

```text
EDGE_SE3:QUAT local_i local_j ...
```

变成：

```text
EDGE_SE3:QUAT merged_i merged_j ...
```

第三，把 `pcd_buffer` 做连续编号链接：

```text
Map/pcd_buffer/
├── 0.pcd
├── 1.pcd
├── ...
└── 22158.pcd
```

点云文件本身不被变换；变换体现在 `pose_graph.g2o` 的每帧 pose 中。

每次追加都会记录：

```text
T_route_xx_to_....txt          # 实际使用的正交化 T
T_route_xx_to_...._raw.txt     # 用户或 CloudCompare 给的原始 T
manifest.csv                   # 每条路线在合并图中的 ID 范围
README.md
```

当前 01-13 CloudCompare 对齐结果：

```text
/home/glf/dataDisk/hainan/yangpu/HBA/manual_hba/
route_01_02_03_04_05_06_07_08_09_10_11_12_13_cloudcompare_track_aligned_pose_graph/
```

其中：

```text
Map/pose_graph.g2o
Map/pcd_buffer/
Map/map.pcd
Map/raw_map.pcd
Map/ground_map.pcd
manifest.csv
```

## 8. 重建 map.pcd

重建地图时读取：

```text
Map/pose_graph.g2o
Map/pcd_buffer/*.pcd
```

对每帧：

```text
global_cloud_i = pose_i * local_cloud_i
```

然后合并、滤波并写出：

```text
Map/raw_map.pcd      # 0.1m 体素左右的全量地图
Map/ground_map.pcd   # z <= 0 的地面/低矮点地图
Map/map.pcd          # 0.35m 左右体素，下游 UI/manual_opt 常用
```

`internal_auto_mapping manual_opt` 需要的通常是：

```text
Map/pose_graph.g2o
Map/map.pcd
Map/pcd_buffer/
Map/origin.txt
```

## 9. GPS 因子优化

多路线点云对齐后，地图局部看起来可能对齐，但全局会偏离原始 GPS 轨迹。最后一步用原始 g2o 中的 GPS 因子做全局约束。

工具：

```text
source/gps_optimize.cpp
devel/lib/hba/gps_optimize
```

输入：

```text
MERGED_MAP_DIR      # 例如 01-13 CloudCompare 对齐结果的 Map/
ORIGINAL_G2O        # /home/glf/dataDisk/hainan/yangpu/HBA/Map/pose_graph.g2o
ROUTE_RANGES_CSV    # /home/glf/dataDisk/hainan/yangpu/HBA/routes/route_ranges.csv
OUTPUT_DIR
```

命令：

```bash
/home/glf/proj/HBA_ws/devel/lib/hba/gps_optimize \
  /home/glf/dataDisk/hainan/yangpu/HBA/manual_hba/route_01_02_03_04_05_06_07_08_09_10_11_12_13_cloudcompare_track_aligned_pose_graph/Map \
  /home/glf/dataDisk/hainan/yangpu/HBA/Map/pose_graph.g2o \
  /home/glf/dataDisk/hainan/yangpu/HBA/routes/route_ranges.csv \
  /home/glf/dataDisk/hainan/yangpu/HBA/manual_hba/route_01_02_03_04_05_06_07_08_09_10_11_12_13_cloudcompare_gps_optimized_pose_graph \
  100
```

优化图包含：

- 变量：每帧 `Pose3`；
- 先验：固定第 0 帧，防止整体自由漂移；
- BetweenFactor：合并后的 `EDGE_SE3:QUAT`；
- GPSFactor：原始 `EDGE_DIS:VEC3` 映射到当前合并 ID 后的 GPS 位置观测。

GPS 残差：

```text
r_i = pose_i.translation - gps_i
```

当前 `/var/params/gps_baesd_mapping_config/fsk.yaml` 中：

```text
t_lidar_gps = [0, 0, 0]
```

因此残差就是 pose 平移和 GPS 位置的差。如果后续杆臂不为 0，应改成：

```text
r_i = pose_i.translation + pose_i.rotation * t_lidar_gps - gps_i
```

GPS ID 映射依赖：

```text
route_ranges.csv   # 原始每条路线 start_index
manifest.csv       # 合并图中每条路线 start_id
```

因为每条路线跳过了起始哨兵空帧，当前约定是：

```text
original_id = route_original_start + local_id
merged_id   = route_merged_start + local_id
```

## 10. GPS 优化输出

当前 GPS 优化结果目录：

```text
/home/glf/dataDisk/hainan/yangpu/HBA/manual_hba/
route_01_02_03_04_05_06_07_08_09_10_11_12_13_cloudcompare_gps_optimized_pose_graph/
```

主要文件：

```text
Map/pose_graph.g2o
Map/pose_graph.gps_optimized.g2o
Map/pcd_buffer/
Map/map.pcd
Map/raw_map.pcd
Map/ground_map.pcd
pose_delta_gps.csv
gps_optimize.log
rebuild_map.log
manifest.csv
```

本次优化统计：

```text
poses:       22159
SE3 edges:   22146
GPS factors: 22057
GPS RMSE:    2.33337 m -> 0.0454054 m
mean pose translation delta: 1.827 m
max pose translation delta:  7.965 m
map.pcd points: 6998613
```

## 11. 各阶段数据语义

| 阶段 | 位姿文件 | 点云目录 | 坐标系/语义 |
|------|----------|----------|-------------|
| 原始 fast_lio_sam_loop | `pose_graph.g2o` | `pcd_buffer/` | 原始全局图，含 GPS 因子 |
| map_preprocess 后 | `pose_before_hba.json` | `pcd/` | 单路线局部首帧归一化 |
| HBA 后 | `pose_after_hba.json` | `pcd/` | 单路线局部 HBA 优化 |
| HBA 反推 g2o | `manual_hba/route_xx/Map/pose_graph.g2o` | `Map/pcd_buffer/` | 单路线全局 pose graph |
| 多路线对齐 | `...cloudcompare_track_aligned.../Map/pose_graph.g2o` | `Map/pcd_buffer/` | 点云匹配 T 对齐后的多路线图 |
| GPS 优化 | `...gps_optimized.../Map/pose_graph.g2o` | `Map/pcd_buffer/` | 同时满足路线间点云对齐和 GPS 全局约束 |

## 12. 注意事项

- `hba` 会覆盖 `pose.json`，必须保留 `pose_before_hba.json` 和 `pose_after_hba.json`；
- CloudCompare 的 T 要明确方向：本流程使用 `registered route_i -> fixed current_map`；
- 追加路线时只变换 pose，不变换每帧原始 PCD；
- `manifest.csv` 是后续 GPS 因子映射和排查 ID 的关键文件；
- GPS 优化会改善全局 GPS 残差，但可能重新引入局部点云重影，需要用 RViz/CloudCompare 检查；
- 如果 GPS 权重过强，地图会被拉向 GPS；如果点云对齐更重要，应调小 GPS 信息矩阵或增加可靠 loop/对齐约束。
