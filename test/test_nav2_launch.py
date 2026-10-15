#!/usr/bin/env python3
"""Nav2 参数传递与独立导航入口验证，不执行硬件节点。"""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest

os.environ["ROS_DOMAIN_ID"] = "224"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.context import Context
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

project = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Nav2LaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        log = project.parents[1] / "log"
        log.mkdir(exist_ok=True)
        cls.directory = Path(tempfile.mkdtemp(prefix="nav2-launch-contract-", dir=log))
        tempfile.tempdir = str(cls.directory)
        cls.params = cls.directory / "nav2.yaml"
        cls.params.write_text((project / "params/nav2_params.yaml").read_text())
        cls.image = cls.directory / "fixture.pgm"
        cls.image.write_text("P2\n2 2\n255\n255 255\n255 255\n")
        cls.map = cls.directory / "fixture.yaml"
        cls.map.write_text(yaml.safe_dump({
            "image": "fixture.pgm", "resolution": 0.05,
            "origin": [0.0, 0.0, 0.0], "negate": 0,
            "occupied_thresh": 0.65, "free_thresh": 0.25}))
        cls.bringup = load(project / "launch/bringup_launch.py", "bringup_under_test")
        cls.navigation = load(project / "launch/navigation_launch.py", "navigation_under_test")
        cls.share = get_package_share_directory("scout_mini_nav")

    def setUp(self):
        self.context = LaunchContext()
        self.description = self.bringup.generate_launch_description()
        for action in self.description.entities[0].get_sub_entities():
            action.execute(self.context)
        self.bringup_actions = self.description.entities[1].get_sub_entities()
        self.include = next(action for action in self.bringup_actions
                            if isinstance(action, IncludeLaunchDescription)
                            and "params_file" in dict(action.launch_arguments))

    def value(self, substitutions):
        return perform_substitutions(
            self.context, normalize_to_list_of_substitutions(substitutions))

    def custom_inputs(self, **values):
        self.context.launch_configurations.update(
            nav2_params_file=str(self.params), map_load_path=str(self.map), **values)

    def prepare_navigation(self):
        for name, value in self.include.launch_arguments:
            self.context.launch_configurations[name] = self.value(value)
        child = self.navigation.generate_launch_description()
        for action in child.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(self.context)
        return child

    def parameters(self, node):
        result = {}
        for block in evaluate_parameters(self.context, node._Node__parameters):
            if isinstance(block, dict):
                result.update(block)
        return result

    def navigation_nodes(self, child):
        return [action for action in child.entities[-2].get_sub_entities()
                if isinstance(action, Node)]

    def test_defaults_use_installed_share(self):
        values = self.context.launch_configurations
        self.assertEqual(values["use_nav2"], "true")
        self.assertEqual(values["map_load_path"],
                         "/home/yan/ros2_ws/src/scout_mini_nav/maps/S402.yaml")
        self.assertEqual(values["nav2_params_file"],
                         str(Path(self.share) / "params/nav2_params.yaml"))

    def test_disabled_skips_navigation_include_with_missing_inputs(self):
        self.context.launch_configurations.update(
            use_nav2="false", nav2_params_file="/missing/nav2.yaml",
            map_load_path="/missing/map.yaml")
        self.assertFalse(self.include.condition.evaluate(self.context))
        # bringup 没有任何 Nav2 Node，关闭 Include 即关闭全部导航节点。
        packages = [self.value(action.node_package) for action in self.bringup_actions
                    if isinstance(action, Node)]
        self.assertFalse(any(name.startswith("nav2_") for name in packages))

    def test_missing_map_rejected_before_hardware(self):
        self.custom_inputs()
        self.context.launch_configurations["map_load_path"] = "/missing/map.yaml"
        child = self.prepare_navigation()
        validation = next(action for action in child.entities if isinstance(action, OpaqueFunction))
        with self.assertRaisesRegex(RuntimeError, "map_load_path"):
            validation.execute(self.context)
        base = next(action for action in self.bringup_actions
                    if isinstance(action, IncludeLaunchDescription)
                    and "port_name" in dict(action.launch_arguments))
        self.assertLess(self.bringup_actions.index(self.include), self.bringup_actions.index(base))

    def test_pgm_cannot_replace_map_yaml(self):
        self.custom_inputs()
        self.context.launch_configurations["map_load_path"] = str(self.image)
        self.prepare_navigation()
        with self.assertRaisesRegex(RuntimeError, "PGM"):
            self.navigation._validate_navigation_inputs(self.context)

    def test_missing_map_image_rejected(self):
        missing = self.directory / "missing-image.yaml"
        missing.write_text("image: does-not-exist.pgm\n")
        self.custom_inputs()
        self.context.launch_configurations["map_load_path"] = str(missing)
        self.prepare_navigation()
        with self.assertRaisesRegex(RuntimeError, "image"):
            self.navigation._validate_navigation_inputs(self.context)

    def test_nested_footprint_rejected(self):
        invalid = self.directory / "nested-footprint.yaml"
        invalid.write_text(yaml.safe_dump({
            "local_costmap": {"local_costmap": {"ros__parameters": {
                "footprint": [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2]]}}}}))
        self.custom_inputs()
        self.context.launch_configurations["nav2_params_file"] = str(invalid)
        self.prepare_navigation()
        with self.assertRaisesRegex(RuntimeError, "footprint"):
            self.navigation._validate_navigation_inputs(self.context)

    def test_custom_paths_clock_and_map_lifecycle_live_in_child(self):
        self.custom_inputs(use_sim_time="true")
        self.assertTrue(self.include.condition.evaluate(self.context))
        child = self.prepare_navigation()
        for action in self.navigation._validate_navigation_inputs(self.context):
            action.execute(self.context)
        arguments = dict(self.include.launch_arguments)
        self.assertEqual(self.value(arguments["params_file"]), str(self.params))
        self.assertEqual(self.value(arguments["map"]), str(self.map))
        self.assertEqual(self.value(arguments["use_sim_time"]), "true")
        nodes = self.navigation_nodes(child)
        by_package = {self.value(node.node_package): node for node in nodes}
        map_node = by_package["nav2_map_server"]
        rewritten = evaluate_parameters(self.context, map_node._Node__parameters)[0]
        document = yaml.safe_load(Path(rewritten).read_text())
        map_parameters = document["map_server"]["ros__parameters"]
        self.assertEqual(map_parameters["yaml_filename"], str(self.map))
        self.assertIs(map_parameters["use_sim_time"], True)
        manager = by_package["nav2_lifecycle_manager"]
        managed = list(self.parameters(manager)["node_names"])
        self.assertIn("map_server", managed)
        self.assertEqual(len(managed), 8)
        self.assertIs(self.parameters(manager)["autostart"], True)
        self.assertEqual(len(nodes), 9)

    def test_child_clock_rewrites_and_velocity_routes(self):
        self.custom_inputs(use_sim_time="false")
        child = self.prepare_navigation()
        self.assertTrue(child.entities[-2].condition.evaluate(self.context))
        nodes = self.navigation_nodes(child)
        by_package = {self.value(node.node_package): node for node in nodes}
        for package in ("nav2_controller", "nav2_behaviors"):
            self.assertIn(("cmd_vel", "cmd_vel_nav"), [
                (self.value(source), self.value(target))
                for source, target in by_package[package]._Node__remappings])
        smoother_routes = [
            (self.value(source), self.value(target))
            for source, target in by_package["nav2_velocity_smoother"]._Node__remappings]
        self.assertIn(("cmd_vel", "cmd_vel_nav"), smoother_routes)
        self.assertIn(("cmd_vel_smoothed", "cmd_vel"), smoother_routes)
        rewritten = evaluate_parameters(
            self.context, by_package["nav2_controller"]._Node__parameters)[0]
        document = yaml.safe_load(Path(rewritten).read_text())
        for name in ("local_costmap", "global_costmap"):
            self.assertIs(document[name][name]["ros__parameters"]["use_sim_time"], False)

    def test_nav2_yaml_parses_in_real_ros(self):
        context = Context()
        try:
            rclpy.init(args=["--ros-args", "--params-file", str(self.params)], context=context)
            self.assertTrue(context.ok())
        finally:
            if context.ok():
                context.shutdown()
        document = yaml.safe_load(self.params.read_text())
        for name in ("local_costmap", "global_costmap"):
            parameters = document[name][name]["ros__parameters"]
            self.assertIsInstance(parameters["footprint"], str)
            self.assertIn("inflation_layer", parameters["plugins"])
        controller = document["controller_server"]["ros__parameters"]
        self.assertIn("ObstacleFootprint", controller["FollowPath"]["critics"])
        for name in ("bt_navigator", "controller_server", "velocity_smoother"):
            self.assertEqual(document[name]["ros__parameters"]["odom_topic"],
                             "/odometry/filtered")
        self.assertEqual(document["velocity_smoother"]["ros__parameters"]
                         ["smoothing_frequency"], 50.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
