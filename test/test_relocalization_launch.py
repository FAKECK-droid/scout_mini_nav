"""Evaluate launch actions without executing hardware nodes."""
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

project = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RelocalizationLaunchTest(unittest.TestCase):
    def setUp(self):
        self.module = load(project / "launch/bringup_launch.py", "bringup_relocalization_test")
        self.context = LaunchContext()
        self.description = self.module.generate_launch_description()
        for action in self.description.entities[0].get_sub_entities():
            action.execute(self.context)
        self.map_odom_static_tf = next(
            action for action in self.description.entities[1].get_sub_entities()
            if isinstance(action, Node) and any(
                isinstance(substitution, TextSubstitution) and substitution.text == "odom"
                for argument in action.cmd for substitution in argument))
        self.directory = Path(tempfile.mkdtemp(prefix="launch-contract-", dir=Path.cwd()))
        self.map = self.directory / "fixture.pcd"
        self.map.write_text("launch validation fixture\n")

    def node_parameters(self, node):
        result = {}
        for block in evaluate_parameters(self.context, node._Node__parameters):
            if isinstance(block, dict):
                result.update(block)
        return result

    def test_disabled_needs_no_map(self):
        self.assertEqual(self.module._create_relocalizer(self.context), [])
        self.assertTrue(self.map_odom_static_tf.condition.evaluate(self.context))

    def test_enabled_missing_map_is_rejected(self):
        self.context.launch_configurations["use_relocalization"] = "true"
        with self.assertRaisesRegex(RuntimeError, "prior_pcd_file"):
            self.module._create_relocalizer(self.context)

    def test_extrinsics_match_existing_converter_and_standalone(self):
        self.context.launch_configurations.update(
            use_relocalization="true", prior_pcd_file=str(self.map))
        relocalizer = self.module._create_relocalizer(self.context)[0]
        converter = self.module._create_lidar_odom_transformer(self.context)[0]
        actual = self.node_parameters(relocalizer)
        original = self.node_parameters(converter)
        self.assertEqual(actual["t_imu_lidar"], original["t_imu_lidar"])
        self.assertEqual(actual["R_imu_lidar"], original["R_imu_lidar"])
        self.assertEqual(list(original["t_base_lidar"]), [0.0, 0.0, 0.145])
        self.assertEqual(list(original["rpy_base_lidar"]), [0.0, 0.0, -math.pi/2])
        self.assertNotIn("t_base_lidar", actual)
        self.assertNotIn("rpy_base_lidar", actual)
        share = Path(get_package_share_directory("scout_mini_relocalization"))
        standalone = load(share / "launch/relocalization.launch.py", "standalone_under_test")
        self.context.launch_configurations["relocalization_config"] = str(
            share / "config/relocalization.yaml")
        self.assertEqual(actual, self.node_parameters(standalone._create(self.context)[0]))
        self.assertFalse(self.map_odom_static_tf.condition.evaluate(self.context))

    def test_invalid_rotation_is_rejected(self):
        config = self.directory / "invalid.yaml"
        config.write_text(yaml.safe_dump({
            "/**": {"ros__parameters": {
                "mapping": {"extrinsic_T": [0, 0, 0], "extrinsic_R": [0]*9}}}}))
        self.context.launch_configurations.update(
            use_relocalization="true", prior_pcd_file=str(self.map), fast_lio_config=str(config))
        with self.assertRaisesRegex(RuntimeError, "orthonormal"):
            self.module._create_relocalizer(self.context)


if __name__ == "__main__":
    unittest.main()
