# HBA 项目需求文档

> 本文档记录在本仓库开发过程中**新增或明确的需求**，便于后续追溯、评审与验收。  
> 原始 HBA 算法需求见 [README.md](../README.md) 及论文引用。  
> 工具使用顺序、输入输出与运行命令见 [WORKFLOW.md](./WORKFLOW.md)。

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-06-27 | 初版：地图数据预处理、代码注释、可视化修复、已知问题 |
| 1.1 | 2026-06-27 | 新增索引范围截取功能 REQ-MAP-007 |
| 1.2 | 2026-06-27 | 新增首帧归一化与 origin.json REQ-MAP-008 |

---

## 1. 背景

在将自采集地图数据（`pose_graph.g2o` + 原始 PCD）接入 HBA 流程时，需要一套**独立的数据预处理工具**，将外部格式转换为 HBA 要求的目录结构，并处理异常 PCD。同时在使用过程中暴露了可视化与主程序在 ROS 时间配置下的行为问题，一并记录备查。

---

## 2. 地图数据预处理（MapDataProcessor）

### 2.1 需求编号：REQ-MAP-001

**标题**：实现 C++ 地图数据预处理类  

**优先级**：高  

**状态**：已实现  

**描述**：单独提供 `.hpp` / `.cpp`，不修改 HBA 原有核心逻辑文件（`hba.cpp`、`ba.hpp` 等），实现从 g2o + 原始 PCD 到 HBA 输入格式的转换。

**实现文件**：

| 文件 | 说明 |
|------|------|
| `include/map_data_processor.hpp` | 类声明 |
| `source/map_data_processor.cpp` | 类实现及可执行入口 `main` |

**编译方式**（独立编译，未写入 `CMakeLists.txt`）：

```bash
cd /home/jiang/code/opensource_ws/src/HBA
g++ -std=c++17 -O2 -I include $(pkg-config --cflags pcl_io-1.10) \
  source/map_data_processor.cpp -o bin/map_preprocess \
  $(pkg-config --libs pcl_io-1.10) -lboost_filesystem -lboost_system
```

**运行方式**：

```bash
# 全量预处理
./bin/map_preprocess [map_root]

# 按首尾索引截取子集（闭区间 [start_index, end_index]）
./bin/map_preprocess [map_root] [start_index] [end_index]

# 示例：提取索引 100~199 共 100 帧
./bin/map_preprocess /home/jiang/project/map 100 199
```

---

### 2.2 需求编号：REQ-MAP-002

**标题**：PCD 文件名零填充并输出至 `pcd/` 目录  

**优先级**：高  

**状态**：已实现  

**输入**：`{map_root}/pcd_buffer/*.pcd`（文件名可为非零填充，如 `0.pcd`、`10001.pcd`）  

**输出**：`{map_root}/pcd/NNNNN.pcd`  

**规则**：

1. 按文件名中的数字索引排序；
2. 剔除空点云并完成索引连续化后，重新编号为 `0 … N-1`；
3. 零填充位数由输出帧数自动计算：  
   `fill_num = max(1, floor(log10(N-1)) + 1)`  
   例：22172 帧 → 5 位 → `00000.pcd` … `22171.pcd`；
4. 输出前清空 `pcd/` 目录下已有 `.pcd`，避免残留文件干扰。

**与 HBA 的衔接**：`hba.launch` 中 `pcd_name_fill_num` 需与填充位数一致（如 5）。

---

### 2.3 需求编号：REQ-MAP-003

**标题**：解析 `pose_graph.g2o` 并生成 `pose.json`  

**优先级**：高  

**状态**：已实现  

**输入**：`{map_root}/pose_graph.g2o`  

**解析规则**：

- 仅处理以 `VERTEX_SE3:QUAT` 开头的行；
- 字段顺序：`index tx ty tz qx qy qz qw`（g2o 四元数顺序为 x,y,z,w）。

**输出**：`{map_root}/pose.json`  

**格式**（与 `mypcl::read_pose` 一致，每行一帧）：

```text
tx ty tz qw qx qy qz
```

**说明**：

- 写入 `pose.json` 前会自动做**首帧归一化**（见 REQ-MAP-008）；`origin.json` 保存归一化前的首帧位姿；
- 输入 g2o 原始位姿**不要求**第 0 帧为零；
- 输出帧顺序与连续化后的 PCD 索引一一对应。

---

### 2.4 需求编号：REQ-MAP-004

**标题**：剔除空 PCD 并保持索引连续  

**优先级**：高  

**状态**：已实现  

**空点云判定**：

1. 文件大小 &lt; 128 字节；或  
2. PCL 加载失败；或  
3. 加载后 `cloud.empty()`。

**处理流程**：

1. 在 `pcd_buffer/` 中**删除**空点云文件；
2. 在 `pcd_buffer/` 内将剩余文件重命名为连续索引 `0.pcd, 1.pcd, …`（通过临时后缀 `.hba_tmp` 避免覆盖）；
3. 同步删除 `pose` 映射中对应索引，并将剩余位姿按新索引重排；
4. 将有效数据拷贝至 `pcd/`（零填充命名）并写入 `pose.json`。

**验收用例**（已在 `/tmp/hba_map_test` 验证）：

- 5 帧中第 2 帧为空 → 输出 4 帧，索引 0–3，位姿与原始非空帧对应关系正确。

---

### 2.5 需求编号：REQ-MAP-005

**标题**：预处理结果目录结构  

**优先级**：中  

**状态**：已实现  

**目标目录结构**（HBA 可直接使用）：

```text
{map_root}/
├── pcd/
│   ├── 00000.pcd
│   ├── 00001.pcd
│   └── ...
├── pose.json      # 相对首帧归一化
└── origin.json    # 首帧原始位姿（归一化前）
```

**参考 launch 配置**（`launch/hba.launch`）：

```xml
<param name="data_path" type="string" value="/home/jiang/project/map/"/>
<param name="pcd_name_fill_num" type="int" value="5"/>
```

---

### 2.6 需求编号：REQ-MAP-006

**标题**：处理日志与统计  

**优先级**：低  

**状态**：已实现  

**要求**：处理结束后输出摘要，包括：

- 解析 g2o 位姿数量；
- `pcd_buffer` 文件数；
- 删除的空 PCD 数量；
- 输出 PCD 数量及零填充位数；
- 缺失 g2o 位姿的告警条数。

可通过 `MapDataProcessor::summary()` 获取完整日志。

---

### 2.7 需求编号：REQ-MAP-007

**标题**：按首尾索引截取 PCD 与位姿子集  

**优先级**：高  

**状态**：已实现  

**描述**：在不全量处理 22172 帧的场景下，按指定索引闭区间从 `pcd_buffer` 与 `pose_graph.g2o` 提取子集，输出至 `pcd/` 与 `pose.json`，便于调试 HBA 或分段处理。

**命令行参数**：

```bash
./bin/map_preprocess <map_root> <start_index> <end_index>
```

**规则**：

1. **PCD**：从 `{map_root}/pcd_buffer/` 中选取文件名索引 ∈ `[start_index, end_index]` 的所有 `.pcd`；
2. **G2O**：解析 `VERTEX_SE3:QUAT` 行，仅保留 `index` ∈ `[start_index, end_index]` 的位姿；
3. **对齐**：按原始索引升序匹配 PCD 与位姿；缺 pose 或空 PCD 的帧跳过并告警；
4. **输出**：重新编号为 `0 … N-1`，零填充拷贝至 `{map_root}/pcd/`，并写入 `{map_root}/pose.json`；
5. **不修改** `pcd_buffer/` 源文件（只读截取，区别于全量模式的空点云删除与重命名）。

**输出格式**：与 REQ-MAP-002、REQ-MAP-003 相同。

**配置方式**（代码调用）：

```cpp
MapDataProcessor::Config config;
config.map_root = "/home/jiang/project/map";
config.use_index_range = true;
config.start_index = 100;
config.end_index = 199;
MapDataProcessor processor(config);
processor.process();
```

**验收示例**：

```bash
./bin/map_preprocess /home/jiang/project/map 1 10
# 期望：跳过空 0.pcd，输出 1~10 中有效帧至 pcd/，pose.json 行数与 pcd 数量一致
```

---

### 2.8 需求编号：REQ-MAP-008

**标题**：`pose.json` 首帧归一化，首帧原始位姿写入 `origin.json`  

**优先级**：高  

**状态**：已实现  

**描述**：预处理输出 `pose.json` 时，以**输出序列的第 0 帧**（对齐后的首帧）为参考系进行归一化；归一化前的首帧绝对位姿单独保存，便于还原世界坐标或追溯。

**输出文件**：

| 文件 | 内容 |
|------|------|
| `origin.json` | 首帧**归一化前**的位姿，单行 |
| `pose.json` | 首帧归一化后的全部位姿，每帧一行 |

**格式**（两者相同）：

```text
tx ty tz qw qx qy qz
```

**归一化公式**（与 HBA `mypcl::write_pose` 一致）：

- 记首帧为 \((R_0, t_0)\)
- 第 \(i\) 帧：\(R_i' = R_0^{-1} R_i\)，\(t_i' = R_0^{-1}(t_i - t_0)\)
- 归一化后第 0 帧：\(t_0' \approx 0\)，\(R_0' \approx I\)

**还原世界坐标**（已知 `origin.json` 与 `pose.json` 第 \(i\) 行）：

```text
t_i = R_0 * t_i' + t_0
R_i = R_0 * R_i'
```

**默认路径**：`{map_root}/origin.json`

**适用范围**：全量预处理与索引范围截取（REQ-MAP-007）均生效；截取模式下“首帧”指输出序列的第 0 帧（即截取范围内的第一帧有效数据）。

**验收**：

```bash
./bin/map_preprocess /home/jiang/project/map 1 10
head -1 /home/jiang/project/map/origin.json   # 应接近 g2o index=1 的原始位姿
head -1 /home/jiang/project/map/pose.json     # 第 0 帧 t≈0, q≈(1,0,0,0)
```

---

## 3. 代码注释（REQ-DOC-001）

**标题**：为整个项目添加中文注释  

**优先级**：中  

**状态**：已实现  

**范围**：

| 文件 | 状态 |
|------|------|
| `include/tools.hpp` | 已注释 |
| `include/mypcl.hpp` | 已注释 |
| `include/hba.hpp` | 已注释 |
| `include/ba.hpp` | 已注释 |
| `include/map_data_processor.hpp` | 已注释 |
| `source/hba.cpp` | 已注释 |
| `source/visualize.cpp` | 已注释 |
| `source/map_data_processor.cpp` | 已注释 |

**原则**：注释说明模块职责、算法步骤与非显然逻辑，避免逐行冗余说明。

---

## 4. 可视化节点修复（REQ-VIZ-001）

**标题**：修复 `visualize_map` 在 `use_sim_time=true` 下卡死  

**优先级**：高  

**状态**：已实现  

**现象**：首帧 publish 完成后循环不再推进；根因为 `ros::Duration::sleep()` 在仿真时钟不推进时永久阻塞。  

**修改**（`source/visualize.cpp`）：

- `ros::Duration(0.001).sleep()` → `ros::WallDuration(0.001).sleep()`；
- `ros::Rate` → `ros::WallRate`；
- 循环内增加 `ros::spinOnce()`。

**建议**：启动前执行 `rosparam set use_sim_time false`，或在 launch 中显式关闭。

---

## 5. 运行与集成说明

### 5.1 推荐数据处理流程

```text
1. map_preprocess  预处理 g2o + pcd_buffer
2. roslaunch hba hba.launch          运行 HBA 优化
3. roslaunch hba visualize.launch    可视化（可选）
```

### 5.2 大规模数据集参考（`/home/jiang/project/map`）

| 项目 | 数值 |
|------|------|
| 原始帧数 | 22172 |
| g2o 顶点数 | 22172 |
| 预处理后 `pcd_name_fill_num` | 5 |
| HBA Layer1 `pose_size` | 22172 |
| HBA Layer3 关键帧 | 886 |

---

## 6. 已知问题与待办（追溯用）

> 来自代码审查与运行现象，**尚未作为正式需求修复**，记录于此便于后续迭代。

| 编号 | 类别 | 描述 | 影响 |
|------|------|------|------|
| ISSUE-001 | 算法 | 局部 BA 优化后的 `x_buf` 未写回 `layer.pose_vec`，上层位姿与 BA 后点云不一致 | 精度 / PGO 一致性 |
| ISSUE-002 | 算法 | 降采样在 BA 迭代循环内重复执行，点云逐轮变稀 | 优化质量 |
| ISSUE-003 | I/O | `mypcl::read_pose` 使用 `while(!file.eof())` 可能多读一条无效位姿 | 帧数错位 |
| ISSUE-004 | PGO | 尾部不完整窗口的 Hessian 与 PGO `cnt` 索引可能不匹配 | 约束错误 |
| ISSUE-005 | 健壮性 | `remove_outlier` 体素数过少时数组索引可能为 -1 | 崩溃风险 |
| ISSUE-006 | ROS | HBA 使用 `ros::Time` 计时，在 `use_sim_time=true` 时显示 0.00s / -nan% | 日志误导 |
| ISSUE-007 | 可视化 | Marker 四元数 `orientation.z` 误赋为 `q.x()` | 显示错误 |
| ISSUE-008 | 集成 | `map_preprocess` 未加入 `CMakeLists.txt`，需手动 g++ 编译 | 部署不便 |

---

## 7. 验收记录

| 需求 | 验收方式 | 结果 | 日期 |
|------|----------|------|------|
| REQ-MAP-001~006 | `/home/jiang/project/map` 全量运行 | 22172 帧输出，`pose.json` 与 g2o 第 0 帧一致 | 2026-06-27 |
| REQ-MAP-007 | `map_preprocess ... 1 10` 索引截取 | 输出子集至 `pcd/`，pose 与 pcd 对齐 | 2026-06-27 |
| REQ-MAP-008 | 同上 + 检查 `origin.json` | 首帧原始位姿保存，pose.json 第 0 帧归一化 | 2026-06-27 |
| REQ-MAP-004 | `/tmp/hba_map_test` 空 PCD 用例 | 5→4 帧，重编号正确 | 2026-06-27 |
| REQ-VIZ-001 | `use_sim_time=true` 下循环可继续 | 改用 WallDuration 后不再卡死 | 2026-06-27 |
| REQ-DOC-001 | `catkin_make --pkg hba` | 编译通过 | 2026-06-27 |

---

## 8. 变更记录

| 日期 | 变更内容 |
|------|----------|
| 2026-06-27 | 创建文档；记录 MapDataProcessor、代码注释、visualize 修复及已知问题 |
| 2026-06-27 | 新增 REQ-MAP-007：首尾索引截取 PCD/位姿子集 |
| 2026-06-27 | 新增 REQ-MAP-008：pose.json 首帧归一化 + origin.json |
