# 配置迁移映射

本表记录 `dual_car_refactor/scripts/dual_car_config.py` 到新配置结构的等价映射。
除表中明确说明的结构组装外，所有叶子键名称和值保持不变。

## `COMMON` 映射

| 旧配置路径 | 新配置位置 | 加载后的兼容路径 |
|---|---|---|
| `COMMON["paths"]["audio_dir"]` | `config.py` 使用包路径定位 `resources/audio` | `COMMON["paths"]["audio_dir"]` |
| `COMMON["paths"]["board2_template_dir"]` | `config.py` 使用包路径定位 `resources/board2` | `COMMON["paths"]["board2_template_dir"]` |
| `COMMON["topics"][*]` | `communication.yaml: topics.*` | `COMMON["topics"][*]` |
| `COMMON["states"][*]` | `config.py: STATES` | `COMMON["states"][*]` |
| `COMMON["board1"][*]` | `vision.yaml: board1.*` | `COMMON["board1"][*]` |
| `COMMON["board2"][*]` | `vision.yaml: board2.*` | `COMMON["board2"][*]` |
| `COMMON["detect"][*]` | `vision.yaml: detect.*` | `COMMON["detect"][*]` |
| `COMMON["nav"]["euler_angles"]` | `waypoints.yaml: euler_angles` | `COMMON["nav"]["euler_angles"]` |
| `COMMON["nav"]["waypoints"][*]` | `waypoints.yaml: waypoints.*` | `COMMON["nav"]["waypoints"][*]` |
| `COMMON["nav"]` 的其他叶子键 | `strategy.yaml: nav.*` | `COMMON["nav"][*]` |
| `COMMON["referee"][*]` | `communication.yaml: referee.*` | `COMMON["referee"][*]` |
| `COMMON["dual_tcp"][*]` | `communication.yaml: dual_tcp.*` | `COMMON["dual_tcp"][*]` |
| `COMMON["lab_info"][*]` | `strategy.yaml: lab_info.*` | `COMMON["lab_info"][*]` |
| `COMMON["window_log_name"][*]` | `strategy.yaml: window_log_name.*` | `COMMON["window_log_name"][*]` |

状态编号逐项保持：

| 键 | 值 |
|---|---:|
| `WAIT_TURN` | 8 |
| `GO_TO_BOARD1` | 9 |
| `BOARD1_RECOGNIZING` | 10 |
| `GO_TO_PICKUP_WINDOWS` | 11 |
| `GO_TO_BOARD2` | 12 |
| `BOARD2_RECOGNIZING` | 13 |
| `GO_TO_LAB_WINDOW` | 14 |
| `GO_BACK_HOME` | 15 |

## `CARS` 映射

以下映射分别应用于车号 1 和 2：

| 旧配置路径 | 新配置位置 | 加载后的兼容路径 |
|---|---|---|
| `CARS[N]["car_id"]` | `strategy.yaml: cars.N.car_id` | `CARS[N]["car_id"]` |
| `CARS[N]["peer_id"]` | `strategy.yaml: cars.N.peer_id` | `CARS[N]["peer_id"]` |
| `CARS[N]["start_active"]` | `strategy.yaml: cars.N.start_active` | `CARS[N]["start_active"]` |
| `CARS[N]["use_peer_board1_result"]` | `strategy.yaml: cars.N.use_peer_board1_result` | `CARS[N]["use_peer_board1_result"]` |
| `CARS[N]["camera_url"]` | `vision.yaml: camera_url.N` | `CARS[N]["camera_url"]` |
| `CARS[N]["home_pose"]` | `waypoints.yaml: home_pose.N` | `CARS[N]["home_pose"]` |
| `CARS[N]["tcp"]["local_port"]` | `communication.yaml: cars.N.local_port` | `CARS[N]["tcp"]["local_port"]` |
| `CARS[N]["tcp"]["peer_ip"]` | `communication.yaml: cars.N.peer_ip` | `CARS[N]["tcp"]["peer_ip"]` |
| `CARS[N]["tcp"]["peer_port"]` | `communication.yaml: cars.N.peer_port` | `CARS[N]["tcp"]["peer_port"]` |

## 类型保持

- `waypoints.*`、`home_pose.*` 和 `board2.roi` 在 YAML 中使用序列保存，加载后恢复为 tuple。
- `euler_angles` 和 `template_scales` 加载后保持 list。
- YAML 的数字车号和 `lab_info` 窗口编号保持 int 键。
- `shared_token: null` 加载后保持 `None`。
- Python 2 下，YAML 产生的 unicode 字符串递归转换为 UTF-8 `str`，与旧源码的 UTF-8 字符串字面量类型保持一致。
- 两个资源路径只改变同一资源的定位方式：由旧业务包绝对路径改为当前 ROS 包内目录，不改变模板或音频文件名。
