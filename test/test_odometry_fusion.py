#!/usr/bin/env python3
"""隔离 ROS 域中验证完整融合链路；没有底盘节点或运动指令发布者。"""
import argparse
import importlib.util
import math
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "232"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_ros.utilities import evaluate_parameters
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

parser = argparse.ArgumentParser()
parser.add_argument("--binary", required=True)
args, remaining = parser.parse_known_args()
project = Path(__file__).resolve().parents[1]


def load_bringup():
    spec = importlib.util.spec_from_file_location(
        "fusion_launch_under_test", project / "launch/bringup_launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LaunchContractTest(unittest.TestCase):
    def test_both_mapping_modes_always_start_fusion(self):
        for mapping in ("false", "true"):
            module = load_bringup()
            context = LaunchContext()
            context.launch_configurations.update(
                mapping_mode=mapping, publish_base_tf="false")
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            group = next(a for a in description.entities if isinstance(a, GroupAction))
            # 只求值声明和参数，不执行任何 Node/Include，不连接真实硬件。
            for action in module._prepare_fusion(context):
                action.execute(context)
            nodes = [a for a in group.get_sub_entities() if isinstance(a, LaunchNode)]
            fusion = next(a for a in nodes if perform_substitutions(
                context, normalize_to_list_of_substitutions(a.node_executable)) == "ekf_node")
            self.assertIsNone(fusion.condition)
            params = evaluate_parameters(context, fusion._Node__parameters)
            self.assertIn("ekf.yaml", str(params[0]))
            wheel = next(a for a in group.get_sub_entities()
                         if isinstance(a, IncludeLaunchDescription)
                         and "publish_tf" in dict(a.launch_arguments))
            child = None
            from launch import LaunchDescription
            for action in wheel.execute(context):
                if isinstance(action, LaunchDescription):
                    child = action
                else:
                    action.execute(context)
            for action in child.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            node = next(a for a in child.entities if isinstance(a, LaunchNode))
            values = {}
            for block in evaluate_parameters(context, node._Node__parameters):
                if isinstance(block, dict):
                    values.update(block)
            self.assertIs(values["publish_tf"], False)
            self.assertEqual(values["linear_velocity_variance"], 0.01)
            self.assertEqual(values["angular_velocity_variance"], 0.01)


class FusionTrajectoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("fusion_synthetic_inputs")
        cls.filtered = []
        cls.transforms = []
        cls.lio = []
        cls.raw_pub = cls.node.create_publisher(Odometry, "/Odometry", 20)
        cls.wheel_pub = cls.node.create_publisher(Odometry, "/odom", 20)
        cls.sub = cls.node.create_subscription(
            Odometry, "/odometry/filtered", cls.filtered.append, 20)
        cls.lio_sub = cls.node.create_subscription(
            Odometry, "/lio/odometry", cls.lio.append, 20)
        cls.tf_sub = cls.node.create_subscription(
            TFMessage, "/tf", lambda msg: cls.transforms.extend(msg.transforms), 20)
        cls.temp = tempfile.TemporaryDirectory()
        cls.log = open(Path(cls.temp.name) / "software_nodes.log", "w+")
        cls.processes = []
        cls.addClassCleanup(cls._cleanup)
        transform_config = yaml.safe_load(
            (project / "params/lidar_odom_transformer.yaml").read_text())
        params = transform_config["lidar_odom_transformer"]["ros__parameters"]
        params.update(
            t_imu_lidar=[-0.011, -0.02329, 0.04412],
            R_imu_lidar=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t_base_lidar=[0.0, 0.0, 0.145],
            rpy_base_lidar=[0.0, 0.0, -math.pi/2])
        config = Path(cls.temp.name) / "converter.yaml"
        config.write_text(yaml.safe_dump(transform_config))
        ekf = Path(get_package_prefix("robot_localization")) / "lib/robot_localization/ekf_node"
        for command in (
            [args.binary, "--ros-args", "--params-file", str(config)],
            [str(ekf), "--ros-args", "-r", "__node:=ekf_filter_node",
             "--params-file", str(project / "params/ekf.yaml")],
        ):
            cls.processes.append(subprocess.Popen(command, stdout=cls.log, stderr=cls.log))
        deadline = time.monotonic()+8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if any(p.poll() is not None for p in cls.processes):
                raise AssertionError("Software node exited")
            if (cls.raw_pub.get_subscription_count() and cls.wheel_pub.get_subscription_count()
                    and any(p.node_name == "ekf_filter_node"
                            for p in cls.node.get_subscriptions_info_by_topic("/lio/odometry"))):
                return
        raise AssertionError("DDS discovery timeout")

    @classmethod
    def _cleanup(cls):
        for process in cls.processes:
            process.terminate()
            process.wait(timeout=5)
        cls.log.flush()
        cls.log.seek(0)
        print(cls.log.read()[-2000:])
        cls.log.close()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.temp.cleanup()

    def wheel_message(self, stamp, vx, vyaw):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        # 故意给轮式积分位姿极大错误，验证 x/y/yaw 位姿未参与融合。
        msg.pose.pose.position.x = 1000.0
        msg.pose.pose.position.y = -900.0
        msg.pose.pose.orientation.z = math.sin(-2.5/2)
        msg.pose.pose.orientation.w = math.cos(-2.5/2)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = vyaw
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.01
        return msg

    def imu_message(self, stamp, position, yaw):
        # 已知车体物理轨迹 → 原始 IMU 观测；覆盖真实 MID360 杠杆臂与 90° 安装角。
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "camera_init"
        msg.child_frame_id = "body"
        initial_yaw = 1.1
        c, s = math.cos(initial_yaw), math.sin(initial_yaw)
        p_base = np.array([c*position[0]-s*position[1]+4.0,
                           s*position[0]+c*position[1]-3.0, 0.25])
        imu_yaw = initial_yaw+yaw-math.pi/2
        ci, si = math.cos(imu_yaw), math.sin(imu_yaw)
        t_ib = np.array([-0.011, -0.02329, -0.10088])
        rotated = np.array([ci*t_ib[0]-si*t_ib[1],
                            si*t_ib[0]+ci*t_ib[1], t_ib[2]])
        p = p_base-rotated
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, p)
        msg.pose.pose.orientation.z = math.sin(imu_yaw/2)
        msg.pose.pose.orientation.w = math.cos(imu_yaw/2)
        for i, variance in enumerate([0.0001]*3+[0.001]*3):
            msg.pose.covariance[7*i] = variance
        return msg

    def test_delayed_lio_and_wheel_twist_follow_known_curve(self):
        from rclpy.duration import Duration
        # 先静止建立 odom，避免把启动发现时间当作运动时间。
        seed_end = time.monotonic()+0.6
        while time.monotonic() < seed_end:
            now = self.node.get_clock().now()
            self.raw_pub.publish(self.imu_message(now, [0.0, 0.0], 0.0))
            self.wheel_pub.publish(self.wheel_message(now, 0.0, 0.0))
            rclpy.spin_once(self.node, timeout_sec=0.02)
        t0 = self.node.get_clock().now().nanoseconds
        vx, vyaw, delay = 0.3, 0.25, 0.06
        end = time.monotonic()+4.0
        step = 0
        while time.monotonic() < end:
            now = self.node.get_clock().now()
            self.wheel_pub.publish(self.wheel_message(now, vx, vyaw))
            if step % 5 == 0:
                observed = now-Duration(seconds=delay)
                t = max(0.0, (observed.nanoseconds-t0)*1e-9)
                angle = vyaw*t
                position = [vx/vyaw*math.sin(angle), vx/vyaw*(1.0-math.cos(angle))]
                self.raw_pub.publish(self.imu_message(observed, position, angle))
            step += 1
            stop = time.monotonic()+0.02
            while time.monotonic() < stop:
                rclpy.spin_once(self.node, timeout_sec=0.002)
        self.assertGreater(len(self.filtered), 30)
        self.assertGreater(len(self.lio), 5)
        output = self.filtered[-1]
        elapsed = (output.header.stamp.sec*10**9+output.header.stamp.nanosec-t0)*1e-9
        angle = vyaw*elapsed
        expected = [vx/vyaw*math.sin(angle), vx/vyaw*(1.0-math.cos(angle))]
        p, q = output.pose.pose.position, output.pose.pose.orientation
        np.testing.assert_allclose([p.x, p.y], expected, atol=0.09)
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.assertLess(abs(yaw-angle), 0.08)
        self.assertAlmostEqual(output.twist.twist.linear.x, vx, delta=0.05)
        self.assertAlmostEqual(output.twist.twist.angular.z, vyaw, delta=0.05)
        self.assertEqual((output.header.frame_id, output.child_frame_id), ("odom", "base_link"))
        self.assertEqual((self.lio[-1].header.frame_id, self.lio[-1].child_frame_id),
                         ("odom", "base_link"))
        self.assertTrue(np.isfinite(output.pose.covariance).all())
        self.assertTrue(any(t.header.frame_id == "odom" and t.child_frame_id == "base_link"
                            for t in self.transforms))
        publishers = self.node.get_publishers_info_by_topic("/tf")
        self.assertEqual({p.node_name for p in publishers}, {"ekf_filter_node"})


if __name__ == "__main__":
    unittest.main(argv=[__file__]+remaining)
