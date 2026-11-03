#!/usr/bin/env python3
"""无硬件启动测试：检查外参方向、工厂 TF、驱动参数及作用域."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

PROJECT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rigid(r, t):
    a = np.eye(4)
    a[:3, :3], a[:3, 3] = r, t
    return a


def qmatrix(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class FemtoBoltTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bringup = load(PROJECT / "launch/bringup_launch.py", "bringup_camera_test")
        cls.support = load(PROJECT / "launch/femto_bolt_support.py", "camera_support_test")
        cls.share = get_package_share_directory("scout_mini_nav")
        qa = PROJECT.parents[1] / "build/scout_mini_nav/qa"
        qa.mkdir(parents=True, exist_ok=True)
        cls.directory = tempfile.TemporaryDirectory(prefix="camera-contract-", dir=qa)
        cls.addClassCleanup(cls.directory.cleanup)

    def setUp(self):
        c, s = np.cos(.21), np.sin(.21)
        self.expected = rigid([[c, -s, 0], [s, c, 0], [0, 0, 1]], [.11, -.02, .08])
        internal = rigid([[0, -1, 0], [0, 0, -1], [1, 0, 0]], [.031, .002, -.003])
        base_lidar = rigid([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], [0, 0, .145])
        canonical = internal @ np.linalg.inv(self.expected) @ base_lidar
        self.document = {
            "convention": "T_A_B maps p_B to p_A; all matrices use column vectors, meters",
            "frames": {"base": "base_link", "lidar": "lidar", "body": "body",
                       "camera": "camera_depth_optical_frame", "camera_link": "camera_link"},
            "T_camera_lidar": {"matrix": canonical.tolist()},
            "T_camera_link_lidar": {
                "matrix": (np.linalg.inv(internal) @ canonical).tolist()},
            "T_lidar_camera": {"matrix": np.linalg.inv(canonical).tolist()},
            "camera_internal_T_optical_frame": {
                "camera_link": {"matrix": internal.tolist()}},
            "metrics": {"accepted": True, "independent_validation": {"passed": True}},
            # 这些粗初值/显示方向/旧安装参数不能覆盖规范标定矩阵。
            "initial_lidar_to_camera": {"translation": [9, 9, 9]},
            "result_extrinsic": {"source_frame": "camera_color_optical_frame",
                                 "target_frame": "body", "translation": [9, 9, 9]},
            "T_base_camera_link": {"matrix": np.eye(4).tolist()},
        }
        self.file = Path(self.directory.name) / (self._testMethodName + ".yaml")
        self.write()
        self.context = LaunchContext()
        self.description = self.bringup.generate_launch_description()
        for action in self.description.entities[0].get_sub_entities():
            action.execute(self.context)
        self.context.launch_configurations["camera_calibration_file"] = str(self.file)

    def write(self):
        self.file.write_text(yaml.safe_dump(self.document), encoding="utf-8")

    def mount(self):
        return self.support.load_camera_mount(str(self.file), "camera", self.bringup.LIDAR_MOUNT)

    def prepare(self):
        for action in self.bringup._prepare_camera(self.context, self.share):
            action.execute(self.context)

    def value(self, substitutions):
        return perform_substitutions(
            self.context, normalize_to_list_of_substitutions(substitutions))

    def test_direction_optical_root_and_current_lidar_mount(self):
        result = self.mount()
        self.assertEqual((result["parent"], result["child"]), ("base_link", "camera_link"))
        np.testing.assert_allclose(result["matrix"], self.expected, atol=1e-12)
        np.testing.assert_allclose(qmatrix(result["quaternion_xyzw"]), self.expected[:3, :3],
                                   atol=1e-12)
        changed = copy.deepcopy(self.bringup.LIDAR_MOUNT)
        changed["translation"][2] += .1
        updated = self.support.load_camera_mount(str(self.file), "camera", changed)
        np.testing.assert_allclose(np.array(updated["translation"]) - result["translation"],
                                   [0, 0, .1], atol=1e-12)

    def test_quaternion_180_degree_rotations(self):
        for r in [np.diag([1, -1, -1]), np.diag([-1, 1, -1]), np.diag([-1, -1, 1])]:
            np.testing.assert_allclose(
                qmatrix(self.support._quaternion(rigid(r, [0, 0, 0]))), r, atol=1e-12)

    def test_invalid_matrices_and_inconsistent_redundant_data(self):
        original = copy.deepcopy(self.document)
        for kind in ("nan", "reflection", "bottom", "shape", "inconsistent", "missing_internal"):
            with self.subTest(kind=kind):
                self.document = copy.deepcopy(original)
                a = self.document["T_camera_lidar"]["matrix"]
                if kind == "nan":
                    a[0][0] = float("nan")
                elif kind == "reflection":
                    for row in a[:3]:
                        row[0] *= -1
                elif kind == "bottom":
                    a[3][0] = 1
                elif kind == "shape":
                    a.pop()
                elif kind == "inconsistent":
                    self.document["T_camera_link_lidar"]["matrix"][0][3] += .2
                else:
                    self.document.pop("camera_internal_T_optical_frame")
                self.write()
                with self.assertRaises(RuntimeError):
                    self.mount()

    def test_wrong_frames_convention_and_failed_quality_rejected(self):
        original = copy.deepcopy(self.document)
        for kind in ("base", "lidar", "camera_link", "convention", "accepted", "validation"):
            with self.subTest(kind=kind):
                self.document = copy.deepcopy(original)
                if kind in ("base", "lidar", "camera_link"):
                    self.document["frames"][kind] = "other"
                elif kind == "convention":
                    self.document["convention"] = "row vectors"
                elif kind == "accepted":
                    self.document["metrics"]["accepted"] = False
                else:
                    self.document["metrics"]["independent_validation"]["passed"] = False
                self.write()
                with self.assertRaises(RuntimeError):
                    self.mount()

    def test_disabled_camera_bypasses_missing_result(self):
        self.context.launch_configurations.update(
            start_camera="false", camera_calibration_file="/missing/result.yaml")
        self.assertEqual(self.bringup._prepare_camera(self.context, self.share), [])
        self.assertEqual(self.bringup._create_camera(self.context, self.share), [])

    def test_preflight_errors_precede_hardware_actions(self):
        actions = self.description.entities[1].get_sub_entities()
        first = next(action for action in actions if isinstance(action, OpaqueFunction))
        self.assertFalse(any(isinstance(action, (Node, IncludeLaunchDescription))
                             for action in actions[:actions.index(first)]))
        self.context.launch_configurations["camera_calibration_file"] = "/missing/result.yaml"
        with self.assertRaisesRegex(RuntimeError, "不存在"):
            first.execute(self.context)
        self.assertNotIn("resolved_camera_mount", self.context.launch_configurations)
        self.context.launch_configurations.update(
            camera_calibration_file=str(self.file), camera_depth_fps="0")
        with self.assertRaisesRegex(RuntimeError, "camera_depth_fps"):
            self.prepare()

    def test_defaults_d2c_ir_off_and_optional_native_mode(self):
        self.prepare()
        import json
        flags = json.loads(self.context.launch_configurations["resolved_camera_driver_args"])
        expected = {
            "enable_ir": "false", "enable_depth": "true", "enable_color": "true",
            "depth_registration": "true", "align_mode": "SW", "align_target_stream": "COLOR",
            "enable_frame_sync": "true", "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false", "cloud_frame_id": "",
            "enable_accel": "false", "enable_gyro": "false",
        }
        for key, value in expected.items():
            self.assertEqual(flags[key], value, key)
        self.context.launch_configurations.update(
            camera_depth_registration="false", camera_enable_frame_sync="false",
            camera_enable_ir="true", camera_enable_point_cloud="true")
        self.prepare()
        flags = json.loads(self.context.launch_configurations["resolved_camera_driver_args"])
        self.assertEqual(flags["depth_registration"], "false")
        self.assertEqual(flags["enable_ir"], "true")
        self.assertEqual(flags["enable_point_cloud"], "true")
        self.assertEqual(flags["cloud_frame_id"], "")

    def test_synchronized_mismatched_fps_rejected_before_hardware(self):
        self.context.launch_configurations["camera_depth_fps"] = "30"
        with self.assertRaisesRegex(RuntimeError, "帧同步"):
            self.prepare()

    def test_driver_flags_static_tf_clock_and_scope(self):
        self.context.launch_configurations.update(
            use_sim_time="true", camera_usb_port="2-1.1", camera_serial_number="test-device",
            camera_depth_fps="30", camera_color_fps="30",
            camera_enable_ir="false", camera_enable_point_cloud="false")
        self.prepare()
        actions = self.bringup._create_camera(self.context, self.share)
        mount = next(action for action in actions if isinstance(action, Node))
        command = [argument[0].text for argument in mount.cmd
                   if len(argument) == 1 and isinstance(argument[0], TextSubstitution)]
        args = {command[i]: command[i+1] for i in range(len(command)-1)
                if command[i].startswith("--")}
        self.assertEqual(args["--frame-id"], "base_link")
        self.assertEqual(args["--child-frame-id"], "camera_link")
        parameters = {}
        for block in evaluate_parameters(self.context, mount._Node__parameters):
            parameters.update(block)
        self.assertIs(parameters["use_sim_time"], True)
        children = actions[-1].get_sub_entities()
        include = next(a for a in children if isinstance(a, IncludeLaunchDescription))
        flags = {k: self.value(v) for k, v in include.launch_arguments}
        for key, expected in {
            "cloud_frame_id": "", "publish_tf": "true", "tf_publish_rate": "0.0",
             "depth_registration": "true", "align_mode": "SW", "align_target_stream": "COLOR",
            "enable_frame_sync": "true", "enable_depth": "true", "enable_color": "true",
            "enable_ir": "false", "enable_point_cloud": "false", "depth_fps": "30",
            "serial_number": "test-device", "usb_port": "2-1.1",
            "point_cloud_qos": "sensor_data", "color_qos": "sensor_data", "depth_qos": "sensor_data",
        }.items():
            self.assertEqual(flags[key], expected, key)
        # 只求值作用域动作和 SetParameter，跳过 Include，完全不打开设备。
        for action in children:
            if action is include:
                self.assertIn(("use_sim_time", True),
                              self.context.launch_configurations["global_params"])
                self.assertIn(("frame_aggregate_mode", "full_frame"),
                              self.context.launch_configurations["global_params"])
                self.assertIn(("diagnostic_period", 5.0),
                              self.context.launch_configurations["global_params"])
                self.assertIn(("align_target_distortion", True),
                              self.context.launch_configurations["global_params"])
            else:
                action.execute(self.context)
        self.assertNotIn("global_params", self.context.launch_configurations)
        self.assertNotIn("publish_tf", self.context.launch_configurations)
        camera = load(Path(get_package_share_directory("orbbec_camera")) /
                      "launch/femto_bolt.launch.py", "vendor_camera_test")
        declared = {a.name for a in camera.generate_launch_description().entities
                    if type(a).__name__ == "DeclareLaunchArgument"}
        self.assertTrue(set(flags).issubset(declared))


if __name__ == "__main__":
    unittest.main(verbosity=2)
