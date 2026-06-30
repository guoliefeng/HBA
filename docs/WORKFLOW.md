# HBA 工具使用顺序与输入输出

本文档说明当前仓库中三个主要工具的使用顺序、输入输出和关键参数对应关系。

## 1. 总体顺序

推荐流程：

```text
原始地图数据
  |
  | 1) map_preprocess
  v
HBA 标准输入目录：pcd/ + pose.json
  |
  | 2) hba
  v
优化后的 pose.json
  |
  | 3) visualize_map
  v
ROS/RViz 可视化话题
```

如果数据已经满足 HBA 输入格式，可以跳过 `map_preprocess`，直接运行 `hba`。

洋浦数据的 13 条路线分段记录见 [YANGPU_ROUTES.md](./YANGPU_ROUTES.md)。

洋浦 13 条路线从原始 `pose_graph.g2o + pcd_buffer` 到 HBA、多路线 CloudCompare 对齐、GPS 因子优化、重建 `map.pcd` 的完整流程见 [YANGPU_FULL_PIPELINE.md](./YANGPU_FULL_PIPELINE.md)。

## 2. 编译

在工作区根目录编译：

```bash
cd /home/glf/proj/HBA_ws
catkin_make
source devel/setup.zsh
```

编译后可执行文件位于：

```text
devel/lib/hba/map_preprocess
devel/lib/hba/hba
devel/lib/hba/visualize_map
```

## 3. 第一步：map_preprocess

### 3.1 用途

`map_preprocess` 将外部地图数据转换为 HBA 可以直接读取的目录结构。它主要处理：

- 从 `pose_graph.g2o` 提取位姿；
- 从 `pcd_buffer/` 读取原始 PCD；
- 剔除空 PCD；
- 将 PCD 重编号为连续索引；
- 将 PCD 文件名改成零填充格式；
- 生成首帧归一化后的 `pose.json`；
- 保存归一化前首帧位姿到 `origin.json`。

### 3.2 输入

默认输入目录结构：

```text
{map_root}/
├── pcd_buffer/
│   ├── 0.pcd
│   ├── 1.pcd
│   └── ...
└── pose_graph.g2o
```

`pose_graph.g2o` 只解析如下行：

```text
VERTEX_SE3:QUAT index tx ty tz qx qy qz qw
```

### 3.3 输出

输出目录结构：

```text
{map_root}/
├── pcd/
│   ├── 0000.pcd
│   ├── 0001.pcd
│   └── ...
├── pose.json
└── origin.json
```

`pose.json` 每行一帧：

```text
tx ty tz qw qx qy qz
```

注意：`pose.json` 已经以第 0 帧为参考做归一化；原始第 0 帧位姿保存在 `origin.json`。

### 3.4 运行命令

全量预处理：

```bash
rosrun hba map_preprocess /path/to/map_root
```

按原始索引截取闭区间 `[start_index, end_index]`：

```bash
rosrun hba map_preprocess /path/to/map_root 100 199
```

全量模式会清理空 PCD，并可能重命名 `pcd_buffer/` 内文件。索引截取模式只读 `pcd_buffer/`，不会修改源 PCD。

### 3.5 关键输出参数

运行结束日志会打印：

- 输出 PCD 数量；
- 删除或跳过的空 PCD 数量；
- `name_fill_num`，也就是 PCD 文件名零填充位数。

这个 `name_fill_num` 必须同步写入后续 launch 文件里的 `pcd_name_fill_num`。

例如输出为：

```text
[INFO] pcd name fill num: 5
```

则需要设置：

```xml
<param name="pcd_name_fill_num" type="int" value="5"/>
```

## 4. 第二步：hba

### 4.1 用途

`hba` 是主优化程序，执行分层 LiDAR Bundle Adjustment 和最后的 GTSAM 位姿图优化。

### 4.2 输入

由 `launch/hba.launch` 中的 `data_path` 指定地图目录：

```xml
<param name="data_path" type="string" value="/path/to/map_root/"/>
```

该目录必须包含：

```text
{data_path}/
├── pcd/
│   ├── 0000.pcd
│   ├── 0001.pcd
│   └── ...
└── pose.json
```

其中：

- `pcd/` 文件名必须从 0 连续编号；
- 文件名零填充位数必须等于 `pcd_name_fill_num`；
- `pose.json` 行数应与有效 PCD 数量一致；
- `pose.json` 格式为 `tx ty tz qw qx qy qz`。

### 4.3 输出

`hba` 的主要输出是覆盖写回：

```text
{data_path}/pose.json
```

这个文件会变成优化后的位姿，仍然以第 0 帧为参考系，格式不变。

重要：`hba` 会直接覆盖输入的 `pose.json`。如果需要保留优化前位姿，请运行前手动备份，例如：

```bash
cp /path/to/map_root/pose.json /path/to/map_root/pose_before_hba.json
```

### 4.4 运行命令

先修改 `launch/hba.launch`：

```xml
<param name="data_path" type="string" value="/path/to/map_root/"/>
<param name="pcd_name_fill_num" type="int" value="5"/>
<param name="total_layer_num" type="int" value="3"/>
<param name="thread_num" type="int" value="16"/>
```

然后运行：

```bash
roslaunch hba hba.launch
```

### 4.5 关键参数

`data_path`：
地图根目录，末尾建议保留 `/`。程序内部会拼接 `pose.json` 和 `pcd/`。

`pcd_name_fill_num`：
PCD 文件名零填充位数。`00000.pcd` 对应 5，`0.pcd` 对应 0。

`total_layer_num`：
HBA 金字塔层数。默认 3。

`thread_num`：
并行线程数。程序会根据窗口数量自动下调实际线程数。

## 5. 第三步：visualize_map

### 5.1 用途

`visualize_map` 读取优化后的 `pose.json` 和 `pcd/`，逐帧变换点云并发布 ROS 话题，用于 RViz 查看地图和轨迹。

### 5.2 输入

由 `launch/visualize.launch` 中的 `file_path` 指定地图目录：

```xml
<param name="file_path" type="string" value="/path/to/map_root/"/>
```

该目录应包含 HBA 优化后的：

```text
{file_path}/
├── pcd/
└── pose.json
```

`pcd_name_fill_num` 仍需与 `pcd/` 文件名一致。

### 5.3 输出

该节点不写文件，只发布 ROS 话题：

```text
/cloud_map          sensor_msgs/PointCloud2
/cloud_debug        sensor_msgs/PointCloud2
/poseArrayTopic     geometry_msgs/PoseArray
/trajectory_marker  visualization_msgs/Marker
/pose_number        visualization_msgs/MarkerArray
```

### 5.4 运行命令

先修改 `launch/visualize.launch`：

```xml
<param name="file_path" type="string" value="/path/to/map_root/"/>
<param name="pcd_name_fill_num" type="int" value="5"/>
<param name="downsample_size" type="double" value="0.2"/>
<param name="marker_size" type="double" value="0.5"/>
```

然后运行：

```bash
roslaunch hba visualize.launch
```

程序启动后会打印：

```text
push enter to view
```

此时需要在终端按回车，节点才会开始逐帧加载、发布点云。

## 6. 文件读写汇总

| 工具 | 读取 | 写入/发布 |
|------|------|-----------|
| `map_preprocess` | `{map_root}/pcd_buffer/*.pcd`，`{map_root}/pose_graph.g2o` | `{map_root}/pcd/*.pcd`，`{map_root}/pose.json`，`{map_root}/origin.json` |
| `hba` | `{data_path}/pcd/*.pcd`，`{data_path}/pose.json` | 覆盖 `{data_path}/pose.json` |
| `visualize_map` | `{file_path}/pcd/*.pcd`，`{file_path}/pose.json` | ROS 话题，不写磁盘文件 |

## 7. 常见检查项

运行 `hba` 前建议检查：

```bash
ls /path/to/map_root/pcd | head
wc -l /path/to/map_root/pose.json
```

确认：

- `pcd/` 文件名从 `0` 开始连续；
- `pcd_name_fill_num` 与实际文件名一致；
- `pose.json` 行数与 PCD 帧数一致；
- `data_path` 和 `file_path` 指向同一个地图根目录；
- `hba` 运行前已经备份原始 `pose.json`，如果后续还需要对比优化前后结果。
