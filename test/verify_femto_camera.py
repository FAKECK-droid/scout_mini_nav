#!/usr/bin/env python3
"""手动硬件检查：只启动 Femto Bolt 和 RGB-D 队列，不启动底盘或导航。"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def camera_child(config_path, backend, color_format):
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription, LaunchService
    from launch.actions import GroupAction, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import SetParameter

    config = yaml.safe_load(Path(config_path).read_text())
    args = config['driver_arguments'].copy()
    args.update(uvc_backend=backend, color_format=color_format, log_level='info')
    args['use_sim_time'] = False
    args = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in args.items()}
    driver = Path(get_package_share_directory('orbbec_camera')) / 'launch/femto_bolt.launch.py'
    launch = LaunchService(argv=[])
    launch.include_launch_description(LaunchDescription([GroupAction(actions=[
        SetParameter(name='use_sim_time', value=False),
        *(SetParameter(name=k, value=v) for k, v in config['node_parameters'].items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver)),
                                 launch_arguments=args.items()),
    ])]))
    return launch.run()


def stop_owned(process):
    if process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
        if process.poll() is not None:
            return
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--camera-child':
        return camera_child(*sys.argv[2:5])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--backend', choices=['v4l2', 'libuvc'], required=True)
    parser.add_argument('--color-format', choices=['MJPG', 'RGB', 'YUYV'], required=True)
    parser.add_argument('--domain', type=int, default=229)
    options = parser.parse_args()
    if not 0 <= options.domain <= 232:
        parser.error('DDS domain must be between 0 and 232')
    workspace = options.workspace.expanduser().resolve()
    config = workspace / 'src/scout_mini_nav/params/femto_bolt.yaml'
    queue_config = workspace / 'src/scout_mini_vln/rgbd_image_queue/config/queue.yaml'
    assert config.is_file() and queue_config.is_file()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal():
            continue
        try:
            args = proc.joinpath('cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if args and b'component_container' in Path(os.fsdecode(args[0])).name.encode():
            if any(b'camera_container' in arg for arg in args):
                raise RuntimeError('A camera container is already running; preserving it')
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = workspace / 'log/scout_mini_nav' / (
        'femto_' + options.backend + '_' + options.color_format + '_' + stamp)
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(options.domain), ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(output / 'ros'))
    import cv2
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CompressedImage
    from rgbd_image_queue.srv import GetImage

    rclpy.init()
    node = rclpy.create_node('femto_camera_repair_probe')
    counts = {key: 0 for key in ['color', 'depth', 'color_compressed', 'depth_compressed']}
    samples, subscriptions, processes, files = {}, [], [], []
    result = {'backend': options.backend, 'requested_color_format': options.color_format,
              'artifacts': str(output), 'counts': counts, 'passed': False}
    def callback(key, msg):
        counts[key] += 1
        if key not in samples:
            samples[key] = msg
    for key, topic, kind in [
        ('color', '/camera/color/image_raw', Image),
        ('depth', '/camera/depth/image_raw', Image),
        ('color_compressed', '/camera/color/image_raw/compressed', CompressedImage),
        ('depth_compressed', '/camera/depth/image_raw/compressedDepth', CompressedImage),
    ]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, key=key: callback(key, msg), qos_profile_sensor_data))
    client = node.create_client(GetImage, '/rgbd_image_queue/get_image')
    def start(command, filename):
        log = open(output / filename, 'w')
        files.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process
    try:
        queue_binary = Path(get_package_prefix('rgbd_image_queue')) / (
            'lib/rgbd_image_queue/rgbd_image_queue_node')
        start([str(queue_binary), '--ros-args', '--params-file', str(queue_config)], 'queue.log')
        camera = start([sys.executable, str(Path(__file__).resolve()), '--camera-child',
                        str(config), options.backend, options.color_format], 'camera.log')
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if camera.poll() is not None:
                raise RuntimeError('Camera launch exited with code ' + str(camera.returncode))
            if min(counts.values()) >= 35 and client.service_is_ready():
                break
        assert min(counts.values()) >= 35, 'Missing image streams: ' + repr(counts)
        request = GetImage.Request()
        request.operation = GetImage.Request.LATEST_PAIR
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=4)
        assert future.done() and future.result() is not None, 'Queue service did not respond'
        pair = future.result()
        assert pair.success and pair.has_color and pair.has_depth, pair.message
        decoded = cv2.imdecode(np.frombuffer(pair.color.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None and decoded.shape[2] == 3, 'Agent color decode failed'
        assert 'compressedDepth' in pair.depth.format, 'Wrong depth transport'
        depth = cv2.imdecode(np.frombuffer(pair.depth.data[12:], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        assert depth is not None and depth.dtype == np.uint16, 'Agent depth decode failed'
        assert list(depth.shape) == list(decoded.shape[:2]), 'D2C dimensions differ'
        result.update(passed=True, color_encoding=samples['color'].encoding,
                      depth_encoding=samples['depth'].encoding,
                      compressed_color_format=pair.color.format,
                      compressed_depth_format=pair.depth.format,
                      color_shape=list(decoded.shape), depth_shape=list(depth.shape),
                      queue_pair_id=pair.pair_id, queue_paired_count=pair.paired_count,
                      queue_stamp_delta_sec=pair.stamp_delta_sec)
    except Exception as error:
        result['error'] = str(error)
    finally:
        for process in reversed(processes):
            stop_owned(process)
        for log in files:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['passed']:
        print((output / 'camera.log').read_text(errors='replace')[-4500:])
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
