#!/usr/bin/env python3
"""Scout Mini、Livox MID360、FAST-LIO2 与 Femto Bolt 的统一启动入口。

启动数据链路为：MID360 驱动发布雷达和 IMU 数据，FAST-LIO2 消费这两路
数据并持续发布里程计与点云；可选启动 Scout Mini 底盘节点，发布底盘里程计。
两路里程计由 robot_localization 融合，无论建图模式是否开启都持续运行。
建图模式控制 FAST-LIO2 的全局地图发布与 PCD 累积保存，不会停止激光惯性
里程计。关闭建图模式时，FAST-LIO2 仍维护算法所需的内部局部地图。
可选启动 Nav2 导航栈与二维地图服务器；地图定位沿用已有 map → odom。
Femto Bolt 读取 camera_calibration_file 指定的外参结果，发布 base_link → camera_link
安装 TF；camera_link 到各图像/光学坐标系的工厂外参由官方相机驱动发布。
默认由驱动将深度对齐到彩色图像；IR 图像流默认关闭，完整配置见 params/femto_bolt.yaml。
"""

import importlib.util
import math
import os
from datetime import datetime

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# base_link → lidar 静态 TF 的位姿；也用于转换节点，两个使用方共享安装参数。
# 作为点坐标变换：p_base = R_base_lidar*p_lidar + t_base_lidar。
LIDAR_MOUNT = {
    "translation": [0.0, 0.0, 0.145],
    "rpy": [0.0, 0.0, -math.pi / 2],
}


def _load_camera_support(nav_share):
    """从同一包的安装目录加载相机启动辅助代码。"""
    spec = importlib.util.spec_from_file_location(
        "scout_femto_bolt_support", os.path.join(nav_share, "launch", "femto_bolt_support.py"))
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    return support


def _prepare_camera(context, nav_share):
    return _load_camera_support(nav_share).prepare_camera(context, LIDAR_MOUNT)


def _create_camera(context, nav_share):
    return _load_camera_support(nav_share).create_camera(context)


def _create_lidar_odom_transformer(context):
    """从实际 FAST-LIO 配置读取 lidar → IMU 外参，构造车体里程计转换节点。

    数组按行排列。固定外参来自同一 YAML；下方 FAST-LIO Node 同时关闭在线
    外参估计，避免内部外参更新后外部转换节点仍使用启动时的旧值。
    在底盘启动前执行，以便参数文件错误时立即终止启动。
    """
    config_path = LaunchConfiguration("fast_lio_config").perform(context)
    transform_config = LaunchConfiguration("lio_transform_config").perform(context)
    if not os.path.isfile(transform_config):
        raise RuntimeError(f"里程计转换配置不存在：{transform_config}")
    with open(config_path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise RuntimeError("FAST-LIO 配置必须是 ROS 参数映射")

    def flatten_parameters(data, prefix=""):
        """把嵌套 ROS 参数转换为点号名称，保留数组值以匹配 ROS 参数覆盖规则。"""
        flat = {}
        for key, value in data.items():
            name = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                flat.update(flatten_parameters(value, name))
            else:
                flat[name] = value
        return flat

    parameters = {}
    for scope in ("/**", "laser_mapping", "/laser_mapping"):
        block = document.get(scope, {}).get("ros__parameters", {})
        parameters.update(flatten_parameters(block))
    try:
        translation = [float(v) for v in parameters["mapping.extrinsic_T"]]
        rotation = [float(v) for v in parameters["mapping.extrinsic_R"]]
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("FAST-LIO 配置缺少有效的 mapping.extrinsic_T/R") from exc
    if len(translation) != 3 or len(rotation) != 9 or not all(
        math.isfinite(v) for v in translation + rotation
    ):
        raise RuntimeError("extrinsic_T 必须为 3 项，extrinsic_R 必须为 9 项有限数值")
    # 列向量正交且右手系；错误矩阵应在启动硬件前被发现。
    columns = [[rotation[3 * row + col] for row in range(3)] for col in range(3)]
    for i in range(3):
        for j in range(3):
            dot = sum(columns[i][k] * columns[j][k] for k in range(3))
            if abs(dot - float(i == j)) > 1e-6:
                raise RuntimeError("extrinsic_R 不是正交旋转矩阵")
    r = rotation
    determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6])
    if abs(determinant - 1.0) > 1e-6:
        raise RuntimeError("extrinsic_R 必须满足 det(R)=+1")

    return [Node(
        package="scout_mini_nav", executable="lidar_odom_transformer",
        name="lidar_odom_transformer", output="screen",
        parameters=[
            transform_config,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                # p_imu = R_imu_lidar*p_lidar + t_imu_lidar；不要对原配置额外求逆。
                "t_imu_lidar": translation,
                "R_imu_lidar": rotation,
                "t_base_lidar": LIDAR_MOUNT["translation"],
                "rpy_base_lidar": LIDAR_MOUNT["rpy"],
            },
        ],
    )]


def _create_relocalizer(context):
    """先检查重定位输入，再启动仅维护 map → odom 的独立节点."""
    if LaunchConfiguration("use_relocalization").perform(context).lower() != "true":
        return []
    share = get_package_share_directory("scout_mini_relocalization")
    config = LaunchConfiguration("relocalization_config").perform(context)
    if not config:
        config = os.path.join(share, "config", "relocalization.yaml")
    spec = importlib.util.spec_from_file_location(
        "relocalization_support", os.path.join(share, "launch", "relocalization_support.py"))
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    return [support.create_relocalizer(
        LaunchConfiguration("prior_pcd_file").perform(context),
        config,
        LaunchConfiguration("fast_lio_config").perform(context),
        ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    )]


def _prepare_fusion(context):
    """核对融合输入与 TF 发布者，并从同一 YAML 取得底盘速度测量方差。"""
    config_path = LaunchConfiguration("ekf_config").perform(context)
    if not os.path.isfile(config_path):
        raise RuntimeError(f"EKF 参数文件不存在：{config_path}")
    with open(config_path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise RuntimeError("EKF 配置必须为 ROS 参数映射")
    ekf = document.get("ekf_filter_node", {}).get("ros__parameters", {})
    wheel = document.get("/scout", {}).get("ros__parameters", {})
    with open(LaunchConfiguration("lio_transform_config").perform(context),
              "r", encoding="utf-8") as stream:
        lio_document = yaml.safe_load(stream)
    lio = lio_document.get("lidar_odom_transformer", {}).get("ros__parameters", {})
    if (ekf.get("world_frame") != "odom" or ekf.get("odom_frame") != "odom"
            or ekf.get("base_link_frame") != "base_link"
            or lio.get("output_world_frame") != "odom"
            or lio.get("base_frame") != "base_link"):
        raise RuntimeError("EKF 与 LIO 转换器必须共同使用 odom → base_link")
    if ekf.get("odom0") != "/odom" or ekf.get("odom1") != "/lio/odometry":
        raise RuntimeError("融合输入必须为 /odom 与 /lio/odometry")
    wheel_mask = ekf.get("odom0_config", [])
    lio_mask = ekf.get("odom1_config", [])
    if len(wheel_mask) != 15 or [i for i, enabled in enumerate(wheel_mask) if enabled] != [6, 11]:
        raise RuntimeError("轮速只能融合 vx 与 vyaw，即 odom0_config 的第 7、12 项")
    if len(lio_mask) != 15 or any(lio_mask[6:]):
        raise RuntimeError("本机 LIO 仅提供位姿，不能选择其 twist 或加速度")
    if ekf.get("publish_tf", True) and (
            LaunchConfiguration("publish_base_tf").perform(context).lower() == "true"
            or lio.get("publish_tf", False)):
        raise RuntimeError("EKF 已发布 odom → base_link；底盘和 LIO 的动态 TF 必须关闭")

    actions = []
    for name in ("linear_velocity_variance", "angular_velocity_variance"):
        value = wheel.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise RuntimeError(f"/scout 的 {name} 必须是有限正方差")
        # Include 传递字符串，由底盘 launch 的 ParameterValue 显式转换为 double。
        actions.append(SetLaunchConfiguration(name, str(float(value))))
    return actions


def _validate_and_prepare_map_path(context):
    """在启动硬件节点前检查 PCD 保存参数，并按需创建目标目录。

    建图模式开启时，要求保存路径为绝对路径且以 .pcd 结尾；若父目录
    尚不存在，则在用户指定的目标路径下创建它。关闭建图时不创建目录。
    路径验证在 GroupAction 的节点启动之前执行，可避免参数错误时先启动底盘。
    """
    mapping_mode = LaunchConfiguration("mapping_mode").perform(context).lower()
    map_save_path = os.path.expanduser(
        LaunchConfiguration("map_save_path").perform(context)
    )

    if mapping_mode not in ("true", "false"):
        raise RuntimeError("mapping_mode 只能设置为 true 或 false")

    if mapping_mode == "true":
        if not os.path.isabs(map_save_path):
            raise RuntimeError(
                "建图模式要求 map_save_path 使用绝对路径，例如 "
                "/home/yan/ros2_ws/maps/scout_mini.pcd"
            )
        if not map_save_path.lower().endswith(".pcd"):
            raise RuntimeError("map_save_path 必须指向 .pcd 文件")

        map_directory = os.path.dirname(map_save_path)
        try:
            os.makedirs(map_directory, exist_ok=True)
        except OSError as exc:
            raise RuntimeError(
                f"无法创建 PCD 地图目录 {map_directory}: {exc}"
            ) from exc

    config_path = LaunchConfiguration("fast_lio_config").perform(context)
    if not os.path.isfile(config_path):
        raise RuntimeError(f"FAST-LIO2 参数文件不存在：{config_path}")

    return [SetLaunchConfiguration("resolved_map_save_path", map_save_path)]


def _create_declare_group(fast_lio_share, nav_share):
    """统一声明启动参数，供后续启动组使用。"""
    # 默认 PCD 文件名按本次启动时间生成。
    map_filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".pcd"
    return GroupAction(
        scoped=False,
        actions=[
            *_load_camera_support(nav_share).declare_arguments(),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
                description="是否使用 ROS 仿真时钟；实机运行保持 false",
            ),
            DeclareLaunchArgument(
                "use_nav2",
                default_value="true",
                choices=["true", "false"],
                description="是否启动 Nav2 导航栈及二维地图服务器",
            ),
            DeclareLaunchArgument(
                "map_load_path",
                default_value="/home/yan/ros2_ws/src/scout_mini_nav/maps/S402-S413.yaml",
                description="导航地图 YAML 完整路径；其中 image 字段指向 PGM 图像",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(nav_share, "params", "nav2_params.yaml"),
                description="Nav2 参数 YAML；默认使用安装后的 scout_mini_nav/share 配置",
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="true",
                choices=["true", "false"],
                description="是否启动 Scout Mini 底盘节点；启动后底盘会进入受控模式",
            ),
            DeclareLaunchArgument(
                "publish_base_tf",
                default_value="false",
                choices=["true", "false"],
                description="底盘自身的轮式 TF 开关；EKF 发布融合 TF 时必须保持 false，/odom 仍发布",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                choices=["true", "false"],
                description="是否启动 MID360 雷达驱动及 base_link 到 lidar 的静态 TF",
            ),
            DeclareLaunchArgument(
                "use_relocalization",
                default_value="false",
                choices=["true", "false"],
                description="true 启动 small_gicp 重定位，RViz 给初值后维护 map → odom；false 不启动重定位并发布重合的静态 TF",
            ),
            DeclareLaunchArgument(
                "prior_pcd_file", default_value="",
                description="启用重定位时必填：先验 PCD 地图绝对路径，不自动选择地图",
            ),
            DeclareLaunchArgument(
                "mapping_mode",
                default_value="false",
                choices=["true", "false"],
                description="true 开启全局地图发布和 PCD 累积；false 只运行激光雷达惯性里程计",
            ),
            DeclareLaunchArgument(
                "map_save_path",
                default_value=os.path.join(
                    "/home/yan/ros2_ws/src/scout_mini_nav/maps", map_filename
                ),
                description="PCD 绝对路径；默认按启动时间命名，指定同名文件时会覆盖",
            ),
            DeclareLaunchArgument(
                "fast_lio_config",
                default_value=os.path.join(
                    fast_lio_share, "config", "mid360_jetson.yaml"
                ),
                description="FAST-LIO2 使用的 ROS 参数 YAML 完整路径",
            ),
            DeclareLaunchArgument(
                "lio_transform_config",
                default_value=os.path.join(
                    nav_share,
                    "params", "lidar_odom_transformer.yaml"),
                description="FAST-LIO IMU 位姿到车体位姿的转换配置",
            ),
            DeclareLaunchArgument(
                "ekf_config", default_value=os.path.join(nav_share, "params", "ekf.yaml"),
                description="轮速和 LIO 融合配置，含底盘速度测量方差",
            ),
            DeclareLaunchArgument(
                "relocalization_config", default_value="",
                description="重定位参数 YAML；空值使用 scout_mini_relocalization 的默认配置",
            ),
            DeclareLaunchArgument(
                "port_name",
                default_value="can0",
                description="Scout Mini 使用的 SocketCAN 接口名称",
            ),
        ],
    )


def _create_bringup_group(scout_base_share, fast_lio_share, nav_share):
    """集中执行准备函数及节点启动。"""
    use_sim_time = LaunchConfiguration("use_sim_time")
    mapping_mode = LaunchConfiguration("mapping_mode")
    fast_lio_config = LaunchConfiguration("fast_lio_config")

    # 由 FAST-LIO2 软件包提供的 MID360 专用驱动启动文件使用 Livox
    # CustomMsg 消息；安装 TF 在本启动文件中统一发布，关闭驱动内的重复发布器。
    mid360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, "launch", "mid360_driver.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_lidar")),
        launch_arguments={"publish_lidar_tf": "false"}.items(),
    )

    # 雷达相对 base_link 的安装外参：X/Y 保持 0，Z 为 0.145 m。
    # 从 +Z 方向俯视，顺时针旋转 90 度对应 yaw = -pi/2。
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_mount_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_lidar")),
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x", str(LIDAR_MOUNT["translation"][0]),
            "--y", str(LIDAR_MOUNT["translation"][1]),
            "--z", str(LIDAR_MOUNT["translation"][2]),
            "--roll", str(LIDAR_MOUNT["rpy"][0]),
            "--pitch", str(LIDAR_MOUNT["rpy"][1]),
            "--yaw", str(LIDAR_MOUNT["rpy"][2]),
            "--frame-id", "base_link",
            "--child-frame-id", "lidar",
        ],
    )

    # 未采用重定位时，map 与 odom 重合；启用重定位后由定位节点维护该 TF。
    map_odom_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_static_tf",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_relocalization")),
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )

    # 底盘发布以 odom 为父坐标系、base_link 为子坐标系的轮式里程计。
    # FAST-LIO2 使用其配置中的 camera_init/body 坐标系，两套里程计均保留。
    scout_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                scout_base_share, "launch", "scout_mini_base.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_base")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "port_name": LaunchConfiguration("port_name"),
            "odom_frame": "odom",
            "base_frame": "base_link",
            "odom_topic_name": "odom",
            "is_scout_mini": "true",
            "is_omni_wheel": "false",
            "simulated_robot": "false",
            "control_rate": "50",
            "publish_tf": LaunchConfiguration("publish_base_tf"),
            "linear_velocity_variance": LaunchConfiguration("linear_velocity_variance"),
            "angular_velocity_variance": LaunchConfiguration("angular_velocity_variance"),
        }.items(),
    )

    # 融合节点无 mapping_mode 条件，两种模式均输出融合里程计和动态 TF。
    ekf_node = Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node",
        output="screen",
        parameters=[LaunchConfiguration("ekf_config"), {
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

    # 将启动参数转换为 YAML 参数所需的布尔值。pcd_save.interval 固定为 -1，
    # 让 FAST-LIO2 把整次建图数据合并到单个 PCD 文件中。
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[
            fast_lio_config,
            {
                "use_sim_time": use_sim_time,
                # 外部转换节点使用固定 YAML 外参，算法端必须保持相同的固定值。
                "mapping.extrinsic_est_en": False,
                "pcd_save.pcd_save_en": ParameterValue(
                    mapping_mode, value_type=bool
                ),
                "pcd_save.interval": -1,
                "map_file_path": LaunchConfiguration("resolved_map_save_path"),
                "publish.map_en": ParameterValue(
                    mapping_mode, value_type=bool
                ),
                "publish.path_en": ParameterValue(
                    mapping_mode, value_type=bool
                ),
            },
        ],
    )

    # Nav2 的全部节点、地图检查与生命周期管理由独立导航入口负责。
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "navigation_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
        launch_arguments={
            "map": LaunchConfiguration("map_load_path"),
            "params_file": LaunchConfiguration("nav2_params_file"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 启动前先检查建图输出路径。所有实际设备和算法节点都集中在本组中，
    # 因而启动参数错误时不会留下已经启动的底盘或雷达进程。
    return GroupAction(
        [
            # 标定文件错误时先终止，避免相机/底盘/雷达已启动。
            OpaqueFunction(function=_prepare_camera, kwargs={"nav_share": nav_share}),
            OpaqueFunction(function=_create_relocalizer),
            OpaqueFunction(function=_validate_and_prepare_map_path),
            OpaqueFunction(function=_prepare_fusion),
            OpaqueFunction(function=_create_lidar_odom_transformer),
            navigation_launch,
            ekf_node,
            scout_base_launch,
            mid360_launch,
            lidar_static_tf,
            OpaqueFunction(function=_create_camera, kwargs={"nav_share": nav_share}),
            map_odom_static_tf,
            fast_lio_node,
        ]
    )


def generate_launch_description():
    """先声明参数，再执行启动组。"""
    scout_base_share = get_package_share_directory("scout_base")
    fast_lio_share = get_package_share_directory("fast_lio")
    nav_share = get_package_share_directory("scout_mini_nav")

    declare_group = _create_declare_group(fast_lio_share, nav_share)
    bringup_cmd_group = _create_bringup_group(scout_base_share, fast_lio_share, nav_share)
    return LaunchDescription([declare_group, bringup_cmd_group])
