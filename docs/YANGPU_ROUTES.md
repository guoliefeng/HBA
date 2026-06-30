# Yangpu HBA 路线分段记录

从这些分段继续到 HBA、路线间 T 对齐、合并 pose graph、GPS 因子优化和最终 `map.pcd` 重建的完整流程见 [YANGPU_FULL_PIPELINE.md](./YANGPU_FULL_PIPELINE.md)。

输入目录：

```text
/home/glf/dataDisk/hainan/yangpu/HBA
```

该目录由 `~/proj/gps_mapping_ws` 中的：

```bash
roslaunch fast_lio_sam_loop run_yangpu_inc.launch
```

生成，包含：

```text
pose_graph.g2o
origin.txt
pcd_buffer/
```

## 分段依据

`pcd_buffer/` 中存在 13 个单点全零 PCD，文件大小均为 273 字节：

```text
0.pcd
6099.pcd
10051.pcd
10843.pcd
11977.pcd
12810.pcd
13588.pcd
14269.pcd
15500.pcd
16355.pcd
17260.pcd
18266.pcd
20327.pcd
```

这些帧被作为路线起始分隔帧处理。`map_preprocess` 已补充过滤逻辑：若 PCD 只有 1 个点且坐标为 `(0, 0, 0)`，则视为空/哨兵帧跳过。因此每条路线输出帧数比输入闭区间长度少 1。

## 输出目录

所有分段输出位于：

```text
/home/glf/dataDisk/hainan/yangpu/HBA/routes/
```

每个路线目录中包含：

```text
pcd/          # 重新从 0 连续编号后的 PCD
pose.json     # 当前为 HBA 优化后的位姿
origin.json   # 该路线首个有效帧归一化前的位姿
preprocess.log
hba.log
pose_before_hba.json
pose_after_hba.json
visualize_before/
visualize_after/
```

同时保留以下符号链接，指向原始输入：

```text
pose_graph.g2o -> /home/glf/dataDisk/hainan/yangpu/HBA/pose_graph.g2o
pcd_buffer     -> /home/glf/dataDisk/hainan/yangpu/HBA/pcd_buffer
origin.txt     -> /home/glf/dataDisk/hainan/yangpu/HBA/origin.txt
```

## 路线范围与输出统计

| 路线 | 原始索引闭区间 | 输出目录 | 输出帧数 | pcd_name_fill_num |
|------|----------------|----------|----------|-------------------|
| 01 | `[0, 6098]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_01_00000_06098` | 6098 | 4 |
| 02 | `[6099, 10050]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_02_06099_10050` | 3951 | 4 |
| 03 | `[10051, 10842]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_03_10051_10842` | 791 | 3 |
| 04 | `[10843, 11976]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_04_10843_11976` | 1133 | 4 |
| 05 | `[11977, 12809]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_05_11977_12809` | 832 | 3 |
| 06 | `[12810, 13587]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_06_12810_13587` | 777 | 3 |
| 07 | `[13588, 14268]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_07_13588_14268` | 680 | 3 |
| 08 | `[14269, 15499]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_08_14269_15499` | 1230 | 4 |
| 09 | `[15500, 16354]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_09_15500_16354` | 854 | 3 |
| 10 | `[16355, 17259]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_10_16355_17259` | 904 | 3 |
| 11 | `[17260, 18265]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_11_17260_18265` | 1005 | 4 |
| 12 | `[18266, 20326]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_12_18266_20326` | 2060 | 4 |
| 13 | `[20327, 22171]` | `/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_13_20327_22171` | 1844 | 4 |

机器生成的统计文件：

```text
/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_ranges.csv
/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_summary.csv
/home/glf/dataDisk/hainan/yangpu/HBA/routes/hba_before_after_comparison.csv
```

## HBA 优化前后文件约定

运行 HBA 时，程序会覆盖路线目录下的 `pose.json`。为避免丢失优化前位姿，当前已按如下约定保存：

```text
pose_before_hba.json  # map_preprocess 生成的优化前位姿
pose_after_hba.json   # HBA 优化后的位姿
pose.json             # 当前与 pose_after_hba.json 内容一致
```

后续使用 `visualize_map` 对比时，建议不要手动覆盖 `pose.json`，直接把 `file_path` 指向下面两个只读入口：

```text
visualize_before/  # pose.json -> ../pose_before_hba.json, pcd -> ../pcd
visualize_after/   # pose.json -> ../pose_after_hba.json,  pcd -> ../pcd
```

例如 route 01：

```xml
<param name="file_path" type="string"
  value="/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_01_00000_06098/visualize_before/"/>
```

或：

```xml
<param name="file_path" type="string"
  value="/home/glf/dataDisk/hainan/yangpu/HBA/routes/route_01_00000_06098/visualize_after/"/>
```
