"""Femto Bolt 启动与标定结果读取；不依赖 NumPy/SciPy，不重复发布相机内部 TF."""
import ast
import json
import math
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    LogInfo, SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_CALIBRATION = (
    "/home/yan/ros2_ws/src/scout_sensor_calibration/results/"
    "20261028_215109_696457/extrinsics.yaml"
)
# 前缀参数只影响本相机，不覆盖父启动文件中的同名参数。
STREAM_ARGUMENTS = {
    "depth_width": ("camera_depth_width", "640", "深度图宽度"),
    "depth_height": ("camera_depth_height", "576", "深度图高度"),
    "depth_fps": ("camera_depth_fps", "15", "深度图帧率"),
    "color_width": ("camera_color_width", "1280", "彩色图宽度"),
    "color_height": ("camera_color_height", "720", "彩色图高度"),
    "color_fps": ("camera_color_fps", "15", "彩色图帧率"),
    "ir_width": ("camera_ir_width", "640", "红外图宽度"),
    "ir_height": ("camera_ir_height", "576", "红外图高度"),
    "ir_fps": ("camera_ir_fps", "15", "红外图帧率"),
}


# ROS 2 driver 参数集中在 params/femto_bolt.yaml，以下是对外暴露的常用覆盖项。
CAMERA_ARGUMENTS = {
    "camera_name": ("camera_name", "驱动命名空间/坐标系前缀，须匹配标定结果", None),
    "camera_serial_number": ("serial_number", "相机序列号；空值自动选择", None),
    "camera_usb_port": ("usb_port", "USB 端口；空值不绑定端口", None),
    "camera_enable_point_cloud": ("enable_point_cloud", "发布深度点云；D2C 开启时为彩色光学坐标系", ["true", "false"]),
    "camera_enable_ir": ("enable_ir", "可选红外图像流，不影响深度测量", ["true", "false"]),
    "camera_ordered_pc": ("ordered_pc", "保留点云图像组织结构；false 省略无效点", ["true", "false"]),
    "camera_depth_registration": ("depth_registration", "自动把深度图对齐到彩色图", ["true", "false"]),
    "camera_align_mode": ("align_mode", "D2C 模式；当前 Femto Bolt 使用 SW，HW 需设备支持", ["SW", "HW"]),
    "camera_enable_frame_sync": ("enable_frame_sync", "SDK 按设备时间戳配对深度和彩色帧", ["true", "false"]),
    "camera_color_qos": ("color_qos", "彩色图 QoS，sensor_data 为 best effort", ["sensor_data", "default"]),
    "camera_depth_qos": ("depth_qos", "深度图 QoS，sensor_data 为 best effort", ["sensor_data", "default"]),
}


def _as_launch(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def _driver_settings():
    path = Path(get_package_share_directory("scout_mini_nav")) / "params/femto_bolt.yaml"
    try:
        settings = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"无法读取相机启动配置 {path}: {exc}") from exc
    if not isinstance(settings, dict) or any(
            not isinstance(settings.get(key), dict) for key in ("driver_arguments", "node_parameters")):
        raise RuntimeError("femto_bolt.yaml 必须包含 driver_arguments 和 node_parameters 映射")
    for block in settings.values():
        for name, value in block.items():
            if not isinstance(value, (str, bool, int, float)) or (
                    isinstance(value, float) and not math.isfinite(value)):
                raise RuntimeError(f"相机参数 {name} 必须是有限标量")
    return settings


def declare_arguments():
    """默认 D2C 对齐彩色、关闭 IR/点云，常用参数可在 launch 命令中覆盖."""
    defaults = _driver_settings()["driver_arguments"]
    args = [
        DeclareLaunchArgument(
            "start_camera", default_value="true", choices=["true", "false"],
            description="是否启动 Femto Bolt 及 base_link 到 camera_link 的标定安装 TF"),
        DeclareLaunchArgument(
            "camera_calibration_file", default_value=DEFAULT_CALIBRATION,
            description="自动标定保存的 extrinsics.yaml；显式指定文件，不自动搜索最新结果"),
    ]
    args.extend(DeclareLaunchArgument(
        name, default_value=_as_launch(defaults[key]), choices=choices, description=description)
        for name, (key, description, choices) in CAMERA_ARGUMENTS.items())
    args.extend(DeclareLaunchArgument(
        name, default_value=_as_launch(defaults[key]), description=description)
        for key, (name, _, description) in STREAM_ARGUMENTS.items())
    return args


def _multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _inverse(a):
    r = [[a[j][i] for j in range(3)] for i in range(3)]
    return [r[i] + [-sum(r[i][j] * a[j][3] for j in range(3))] for i in range(3)] + [
        [0.0, 0.0, 0.0, 1.0]]


def _rigid(block, name):
    """检查列向量、米制 4×4 刚体矩阵；不猜测行向量或转置约定."""
    data = block.get("matrix") if isinstance(block, dict) else None
    if (not isinstance(data, list) or len(data) != 4
            or any(not isinstance(row, list) or len(row) != 4 for row in data)):
        raise RuntimeError(f"{name}.matrix 必须是 4×4 数组；请选择结果 extrinsics.yaml")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for row in data for v in row):
        raise RuntimeError(f"{name}.matrix 含非有限数值")
    a = [[float(v) for v in row] for row in data]
    if max(abs(a[3][j] - float(j == 3)) for j in range(4)) > 1e-8:
        raise RuntimeError(f"{name}.matrix 最后一行必须为 [0, 0, 0, 1]")
    for i in range(3):
        for j in range(3):
            if abs(sum(a[k][i] * a[k][j] for k in range(3)) - float(i == j)) > 1e-5:
                raise RuntimeError(f"{name} 的旋转矩阵不正交")
    det = (a[0][0] * (a[1][1]*a[2][2] - a[1][2]*a[2][1])
           - a[0][1] * (a[1][0]*a[2][2] - a[1][2]*a[2][0])
           + a[0][2] * (a[1][0]*a[2][1] - a[1][1]*a[2][0]))
    if abs(det - 1.0) > 1e-5:
        raise RuntimeError(f"{name} 的旋转矩阵必须满足 det(R)=+1")
    return a


def _same(a, b, name):
    if max(abs(a[i][j] - b[i][j]) for i in range(4) for j in range(4)) > 1e-5:
        raise RuntimeError(f"{name} 与规范外参/相机内部 TF 不一致，请重新导出结果")


def _base_lidar(mount):
    roll, pitch, yaw = mount["rpy"]
    sr, cr, sp, cp, sy, cy = (
        math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch),
        math.sin(yaw), math.cos(yaw))
    rows = [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ]
    return [row + [mount["translation"][i]] for i, row in enumerate(rows)] + [
        [0.0, 0.0, 0.0, 1.0]]


def _quaternion(a):
    """从正交旋转矩阵生成 ROS xyzw 四元数，包括接近 180 度的旋转."""
    trace = sum(a[i][i] for i in range(3))
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = [(a[2][1]-a[1][2])/s, (a[0][2]-a[2][0])/s, (a[1][0]-a[0][1])/s, s/4]
    else:
        i = max(range(3), key=lambda k: a[k][k])
        j, k = (i+1) % 3, (i+2) % 3
        s = 2 * math.sqrt(1 + a[i][i] - a[j][j] - a[k][k])
        q = [0.0] * 4
        q[i], q[j], q[k], q[3] = s/4, (a[j][i]+a[i][j])/s, (a[k][i]+a[i][k])/s, (a[k][j]-a[j][k])/s
    length = math.sqrt(sum(v*v for v in q))
    return [v / length * (-1 if q[3] < 0 else 1) for v in q]


def load_camera_mount(filename, camera_name, lidar_mount):
    """T_base_link = T_base_lidar · inv(T_optical_lidar) · T_optical_camera_link.

    YAML 的 result_extrinsic 可能指向 RGB 光学坐标系，甚至反向输入；
    使用规范的 T_camera_lidar 与导出的真实驱动内部 TF，不把光学系当作根。
    """
    if not filename.strip():
        raise RuntimeError("start_camera=true 时必须设置 camera_calibration_file")
    path = Path(filename).expanduser()
    if not path.is_absolute():
        raise RuntimeError("camera_calibration_file 必须是绝对路径")
    if not path.is_file():
        raise RuntimeError(f"相机标定结果不存在：{path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"无法读取相机标定结果 {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise RuntimeError("相机标定结果必须为 YAML 映射")
    if document.get("convention") != "T_A_B maps p_B to p_A; all matrices use column vectors, meters":
        raise RuntimeError("相机外参必须使用标定结果的列向量、米制约定；请选择 extrinsics.yaml")
    frames = document.get("frames", {})
    if not isinstance(frames, dict) or frames.get("base") != "base_link" or frames.get("lidar") != "lidar":
        raise RuntimeError("标定结果必须使用 frames.base=base_link、frames.lidar=lidar")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", camera_name):
        raise RuntimeError("camera_name 必须是单段有效 ROS 名称")
    root = frames.get("camera_link")
    if root != camera_name + "_link":
        raise RuntimeError(f"camera_name 与标定根坐标系不一致：{root}")
    optical = frames.get("camera")
    if not isinstance(optical, str) or not optical:
        raise RuntimeError("标定结果缺少 frames.camera")
    metrics = document.get("metrics")
    if metrics is not None:
        if not isinstance(metrics, dict) or metrics.get("accepted") is not True:
            raise RuntimeError("相机标定结果记录的质量检查未通过")
        validation = metrics.get("independent_validation")
        if validation is not None and (
                not isinstance(validation, dict) or validation.get("passed") is not True):
            raise RuntimeError("相机标定结果记录的独立验证未通过")
    canonical = _rigid(document.get("T_camera_lidar"), "T_camera_lidar")
    internal = document.get("camera_internal_T_optical_frame")
    if not isinstance(internal, dict) or root not in internal:
        raise RuntimeError(f"标定结果缺少相机内部 T_{optical}_{root}；不能猜测光学系到根坐标系的旋转")
    camera_root = _rigid(internal[root], "camera_internal_T_optical_frame." + root)
    root_lidar = _multiply(_inverse(camera_root), canonical)
    if "T_camera_link_lidar" in document:
        _same(_rigid(document["T_camera_link_lidar"], "T_camera_link_lidar"),
              root_lidar, "T_camera_link_lidar")
    if "T_lidar_camera" in document:
        _same(_rigid(document["T_lidar_camera"], "T_lidar_camera"),
              _inverse(canonical), "T_lidar_camera")
    # 与导航/LIO 转换器共用当前 LIDAR_MOUNT，不使用过期的底盘安装参数。
    base_root = _multiply(_base_lidar(lidar_mount), _inverse(root_lidar))
    return {
        "file": str(path.resolve()), "parent": "base_link", "child": root,
        "matrix": base_root, "translation": [base_root[i][3] for i in range(3)],
        "quaternion_xyzw": _quaternion(base_root),
    }


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def prepare_camera(context, lidar_mount):
    """在任何硬件启动前验证外参与全部相机参数，并缓存最终覆盖结果."""
    if _arg(context, "start_camera").lower() == "false":
        return []
    settings = _driver_settings()
    driver_args = {key: _as_launch(value) for key, value in settings["driver_arguments"].items()}
    driver_args.update({key: _arg(context, name) for name, (key, _, _) in CAMERA_ARGUMENTS.items()})
    driver_args.update({key: _arg(context, spec[0]) for key, spec in STREAM_ARGUMENTS.items()})
    for key, (name, _, _) in STREAM_ARGUMENTS.items():
        value = driver_args[key]
        if not value.isdecimal() or int(value) <= 0:
            raise RuntimeError(f"{name} 必须为正整数")
    if driver_args["enable_depth"] != "true" or driver_args["enable_color"] != "true":
        raise RuntimeError("为保持标定中的 camera_link 定义，必须启用深度和彩色流")
    if driver_args["publish_tf"] != "true" or driver_args["cloud_frame_id"]:
        raise RuntimeError("必须由驱动发布内部 TF，cloud_frame_id 必须留空以保留真实坐标")
    if driver_args["align_target_stream"] != "COLOR":
        raise RuntimeError("导航 D2C 的 align_target_stream 必须为 COLOR")
    if driver_args["enable_frame_sync"] == "true" and driver_args["depth_fps"] != driver_args["color_fps"]:
        raise RuntimeError("SDK 帧同步要求 camera_depth_fps 与 camera_color_fps 一致")
    if driver_args["depth_registration"] == "true" and settings["node_parameters"].get(
            "frame_aggregate_mode") != "full_frame":
        raise RuntimeError("D2C 要求 frame_aggregate_mode=full_frame，避免处理缺失彩色/深度的帧组")
    mount = load_camera_mount(
        _arg(context, "camera_calibration_file"), _arg(context, "camera_name"), lidar_mount)
    driver = Path(get_package_share_directory("orbbec_camera")) / "launch/femto_bolt.launch.py"
    if not driver.is_file():
        raise RuntimeError(f"Femto Bolt 驱动启动文件不存在：{driver}")
    declared = {
        ast.literal_eval(node.args[0]) for node in ast.walk(ast.parse(driver.read_text()))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
    }
    if set(driver_args) != declared:
        raise RuntimeError(f"相机配置与当前驱动参数不一致：未知 {sorted(set(driver_args)-declared)}；"
                           f"缺少 {sorted(declared-set(driver_args))}")
    return [
        SetLaunchConfiguration("resolved_camera_mount", json.dumps(mount)),
        SetLaunchConfiguration("resolved_camera_launch", str(driver)),
        SetLaunchConfiguration("resolved_camera_driver_args", json.dumps(driver_args)),
        SetLaunchConfiguration("resolved_camera_node_params", json.dumps(settings["node_parameters"])),
    ]


def create_camera(context):
    """发布一条 base_link→camera_link；所有相机内部 TF 由驱动负责."""
    if _arg(context, "start_camera").lower() == "false":
        return []
    mount = json.loads(_arg(context, "resolved_camera_mount"))
    driver_args = json.loads(_arg(context, "resolved_camera_driver_args"))
    node_params = json.loads(_arg(context, "resolved_camera_node_params"))
    tf_args = []
    for key, value in zip(("x", "y", "z"), mount["translation"]):
        tf_args.extend(["--" + key, str(value)])
    for key, value in zip(("qx", "qy", "qz", "qw"), mount["quaternion_xyzw"]):
        tf_args.extend(["--" + key, str(value)])
    tf_args.extend(["--frame-id", mount["parent"], "--child-frame-id", mount["child"]])
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [
        LogInfo(msg=f"Femto Bolt D2C={driver_args['depth_registration']} "
                    f"({driver_args['align_mode']} -> {driver_args['align_target_stream']})；"
                    f"IR={driver_args['enable_ir']}；点云={driver_args['enable_point_cloud']}"),
        LogInfo(msg=f"Femto Bolt 外参：{mount['file']}；静态 TF {mount['parent']} -> {mount['child']}；"
                    f"xyz(m)={mount['translation']}，q(xyzw)={mount['quaternion_xyzw']}"),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="camera_mount_tf", output="screen", arguments=tf_args,
            parameters=[{"use_sim_time": use_sim_time}]),
        # Include 的默认参数和 use_sim_time 只在相机组内生效。
        GroupAction(scoped=True, actions=[
            SetParameter(name="use_sim_time", value=use_sim_time),
            *(SetParameter(name=name, value=value) for name, value in node_params.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_arg(context, "resolved_camera_launch")),
                launch_arguments=driver_args.items()),
        ]),
    ]
