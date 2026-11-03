# Femto Bolt 导航启动配置

## 使用

    cd /home/yan/ros2_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch scout_mini_nav bringup_launch.py

默认启动相机，读取标定结果：

/home/yan/ros2_ws/src/scout_sensor_calibration/results/20261028_215109_696457/extrinsics.yaml

可用 camera_calibration_file:=/绝对路径/extrinsics.yaml 覆盖；只读取明确指定文件，
不会自动选取最新结果。选择 extrinsics.yaml，不选择初值 calibration.yaml 或诊断数据。

## 默认输出与对齐

| 项目 | 默认配置/实际输出 |
| --- | --- |
| 深度采集 | 640×576，15 fps，Y16 |
| 彩色采集 | 1280×720，15 fps，USB MJPG；ROS 输出 rgb8 |
| D2C | 开启；SDK 软件对齐 SW，目标 COLOR |
| 对齐深度 | 1280×720，16UC1，毫米；0 表示无效深度 |
| 对齐坐标系 | 深度图和可选深度点云均为 camera_color_optical_frame |
| 内参/畸变 | 对齐深度采用目标彩色模型，包含 raw 彩色图的目标畸变 |
| 帧同步 | 开启 SDK 时间戳配帧，full_frame 完整帧组；不等同外部硬触发 |
| IR | 默认关闭图像流，深度测量与激光发射保持工作 |
| 深度点云/彩色点云 | 默认均关闭；深度点云可通过参数启用 |
| 图像 QoS | sensor_data：best effort、volatile、保留最近 5 帧 |
| CameraInfo QoS | default：可靠传输；使用同路 camera_info 解释图像 |
| 空间降采样/额外软件滤波 | 未开启 |
| 相机 IMU、D2C 查看器、额外去畸变图 | 默认关闭 |
| 时间域/TF | system；工厂内部 TF 通过 /tf_static 发布 |
| 温度等诊断查询 | 每 5 秒一次 |

当前设备 firmware=1.0.9，SDK=2.7.6；实测查询该彩色模式下 HW D2C 支持数为 0，
SW 支持数为 14，640×576@15 是其中之一，因此默认 SW。
驱动 JPEG 的 NV 硬件解码编译开关未开启，不能把当前软件 D2C 或 MJPG 解码称为硬件/GPU 对齐。

软件对齐默认缺省 TargetDistortion=0，而彩色 image_raw 带有畸变。
驱动新增可选参数 align_target_distortion（驱动默认 false），导航显式设置 true，
让 SDK 使用 raw 彩色的目标畸变；不额外生成去畸变彩色图。
独立标定和 MATLAB 采集启动文件仍沿用各自配置。

对齐后的深度图与彩色图共享像素网格，但视差、遮挡和无效深度会留下空洞；
不要将缺失值当成实际距离。图像时间戳仍来自各自传感器，并非强行改成相同时间。
best effort 图像订阅应使用 SensorDataQoS；已核查现有 VLN capture_image 订阅兼容。
如需可靠图像，可设置 camera_color_qos:=default camera_depth_qos:=default。

## launch 常用覆盖参数

| 参数 | 默认值/含义 |
| --- | --- |
| start_camera | true；false 同时关闭相机及安装 TF |
| camera_calibration_file | 上述标定结果绝对路径 |
| camera_name | camera；必须匹配结果的 frames.camera_link |
| camera_serial_number | 空；自动选择 |
| camera_usb_port | 空；不固定 USB 端口 |
| camera_depth_registration | true；false 可恢复原生深度图 |
| camera_align_mode | SW；当前设备模式不支持 HW |
| camera_enable_frame_sync | true；开启时深度和彩色 fps 必须相同 |
| camera_enable_ir | false；需要 IR 图像时置 true |
| camera_enable_point_cloud | false；需要深度点云时置 true |
| camera_ordered_pc | false；省略无效点，不进行空间降采样 |
| camera_color_qos / camera_depth_qos | sensor_data / sensor_data |
| camera_depth_width / height / fps | 640 / 576 / 15；表示原生采集模式 |
| camera_color_width / height / fps | 1280 / 720 / 15 |
| camera_ir_width / height / fps | 640 / 576 / 15；仅在 IR 开启时输出 |
| use_sim_time | false；使用导航总开关传入相机和安装 TF |

例如仅额外开启点云：

    ros2 launch scout_mini_nav bringup_launch.py camera_enable_point_cloud:=true

不使用对齐：

    ros2 launch scout_mini_nav bringup_launch.py camera_depth_registration:=false

查看全部对外参数：

    ros2 launch scout_mini_nav bringup_launch.py --show-args

## 安装 TF

发布直接的 base_link → camera_link 静态 TF，节点名 camera_mount_tf。

T_base_camera_link = T_base_lidar · inverse(T_camera_lidar) · T_camera_camera_link

T_base_lidar 共用导航入口当前的 LIDAR_MOUNT；另外两项读取标定结果的规范矩阵
和已保存工厂内部 TF，不把 RGB 光学系当成相机根，不把变换平移误作相机位姿。
相机驱动继续拥有所有内部 TF，不再发布另一条 lidar → camera_link。

默认 IR 关闭时：

    base_link
    ├── lidar
    └── camera_link
        └── camera_depth_frame
            ├── camera_depth_optical_frame
            └── camera_color_frame
                └── camera_color_optical_frame

D2C 只改变数据的投影坐标，不删除或改写物理深度光学坐标系。
开启 IR 后，驱动再增加 camera_ir_frame 和 camera_ir_optical_frame。

    ros2 run tf2_ros tf2_echo base_link camera_link
    ros2 topic echo /camera/depth/camera_info --once
    ros2 topic echo /camera/depth/image_raw --field header --once

## 全部默认驱动启动配置

权威配置位于 params/femto_bolt.yaml。
driver_arguments 完整覆盖当前官方 femto_bolt.launch.py 的 99 个声明项；
node_parameters 通过相机作用域 SetParameter 传给组件。
常用 camera_* 命令行参数覆盖对应配置；use_sim_time 由导航入口单独传入。
-1 等值代表使用设备/SDK 默认设置；关闭的滤波、IR、IMU 参数不会启动相应功能。

```yaml
driver_arguments:
  camera_name: camera
  depth_registration: true
  serial_number: ''
  usb_port: ''
  device_num: 1
  uvc_backend: libuvc
  product_id: ''
  enable_point_cloud: false
  cloud_frame_id: ''
  enable_colored_point_cloud: false
  point_cloud_qos: sensor_data
  connection_delay: 100
  color_width: 1280
  color_height: 720
  color_fps: 15
  color_format: MJPG
  enable_color: true
  color_flip: false
  color_qos: sensor_data
  color_camera_info_qos: default
  enable_color_auto_exposure: true
  color_ae_max_exposure: -1
  color_exposure: -1
  color_gain: -1
  color_brightness: -1
  enable_color_auto_white_balance: true
  color_white_balance: -1
  depth_width: 640
  depth_height: 576
  depth_fps: 15
  depth_format: Y16
  enable_depth: true
  depth_flip: false
  depth_qos: sensor_data
  depth_camera_info_qos: default
  ir_width: 640
  ir_height: 576
  ir_fps: 15
  ir_format: Y16
  enable_ir: false
  ir_flip: false
  ir_qos: sensor_data
  ir_camera_info_qos: default
  enable_ir_auto_exposure: true
  ir_ae_max_exposure: -1
  ir_exposure: -1
  ir_gain: -1
  ir_brightness: -1
  enable_sync_output_accel_gyro: false
  enable_accel: false
  accel_rate: 200hz
  accel_range: 4g
  enable_gyro: false
  gyro_rate: 200hz
  gyro_range: 1000dps
  linear_accel_cov: 0.01
  angular_vel_cov: 0.01
  publish_tf: true
  tf_publish_rate: 0.0
  ir_info_url: ''
  color_info_url: ''
  log_level: none
  log_file_name: ''
  enable_publish_extrinsic: false
  enable_d2c_viewer: false
  enable_ldp: true
  enable_noise_removal_filter: false
  enable_decimation_filter: false
  enable_spatial_filter: false
  enable_temporal_filter: false
  enable_hole_filling_filter: false
  enable_threshold_filter: false
  noise_removal_filter_min_diff: 10
  noise_removal_filter_max_size: 50
  decimation_filter_scale: -1
  spatial_filter_alpha: -1.0
  spatial_filter_diff_threshold: -1
  spatial_filter_magnitude: -1
  spatial_filter_radius: -1
  temporal_filter_diff_threshold: -1.0
  temporal_filter_weight: -1.0
  hole_filling_filter_mode: ''
  threshold_filter_max: -1
  threshold_filter_min: -1
  sync_mode: standalone
  enable_frame_sync: true
  depth_delay_us: 0
  color_delay_us: 0
  trigger2image_delay_us: 0
  trigger_out_delay_us: 0
  trigger_out_enabled: false
  ordered_pc: false
  enable_depth_scale: true
  align_mode: SW
  align_target_stream: COLOR
  laser_energy_level: -1
  enable_heartbeat: false
  time_domain: system
  device_preset: Custom
node_parameters:
  frame_aggregate_mode: full_frame
  diagnostic_period: 5.0
  align_target_distortion: true
  enable_color_undistortion: false
  point_cloud_decimation_filter_factor: 1
  enable_laser: true
```

## 本次验证记录（2026-10-28）

- 编译：orbbec_camera、scout_mini_nav 成功。
- 22 项启动检查通过；仅相机实机测试的 3 个场景全部通过，未启动底盘或导航。
- 默认配置 RGB/深度约 14–15 fps；10 秒采样，相机组件约占单核 85%，RSS 约 211 MB。
- IR 开启对照同样正常输出深度，约占单核 88%、RSS 约 217 MB；
  该单次采样用于验证功能和资源量级，不作为固定性能保证。
- 可选点云的 frame_id 随 D2C 变为 camera_color_optical_frame。
- 默认 IR 关闭时，相机只有 5 个 TF 坐标系；开启 IR 后增加两个 IR 坐标系。
- base_link → camera_link 实测值与标定矩阵一致；D2C 不改变安装外参。
- 实机参数服务导出 329 项，包含驱动内部默认值和自动生成的坐标系参数。

完整运行参数快照：

/home/yan/ros2_ws/build/scout_mini_nav/qa/d2c_20261028_222923/aligned_default/runtime_parameters.yaml

实机日志、对照结果和构建日志：

/home/yan/ros2_ws/build/scout_mini_nav/qa/d2c_20261028_222923

相机运行时也可以直接查看：

    ros2 param dump /camera/camera
