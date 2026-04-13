#!/usr/bin/env python3
import os
from typing import Dict, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from interfaces.srv import SetConfig


class LoadConfigManager:
    def __init__(self):
        self.route_id = "route_a"
        self.cycles = 2
        self.points = ["p0", "p1"]
        self.trigger_mode = "button"
        self.trigger_time = "08:00"
        self.trigger_duration_sec = 5

    def load_from_file(self, path: str) -> Tuple[bool, str]:
        if not os.path.exists(path):
            return False, f"config not found: {path}"
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            return False, f"failed to load yaml: {exc}"

        route_id = data.get("patrol_route_id", self.route_id)
        cycles = int(data.get("patrol_cycles", self.cycles))
        points = data.get("patrol_points", list(self.points))
        if not isinstance(points, list) or not points:
            return False, "patrol_points must be a non-empty list"

        trigger_mode = str(data.get("patrol_trigger_mode", self.trigger_mode)).strip().lower()
        if trigger_mode not in ("button", "time"):
            return False, "patrol_trigger_mode must be 'button' or 'time'"
        trigger_time = str(data.get("patrol_trigger_time", self.trigger_time)).strip()
        trigger_duration_sec = int(data.get("patrol_trigger_duration_sec", self.trigger_duration_sec))
        if trigger_duration_sec <= 0:
            return False, "patrol_trigger_duration_sec must be > 0"

        self.route_id = str(route_id)
        self.cycles = int(cycles)
        self.points = [str(p) for p in points]
        self.trigger_mode = trigger_mode
        self.trigger_time = trigger_time
        self.trigger_duration_sec = int(trigger_duration_sec)
        return True, "ok"

    def as_dict(self) -> Dict[str, object]:
        return {
            "patrol_route_id": self.route_id,
            "patrol_cycles": int(self.cycles),
            "patrol_points": list(self.points),
            "patrol_trigger_mode": self.trigger_mode,
            "patrol_trigger_time": self.trigger_time,
            "patrol_trigger_duration_sec": int(self.trigger_duration_sec),
        }


class LoadConfigServer(Node):
    def __init__(self):
        super().__init__("loadconfig_server")
        self.manager = LoadConfigManager()
        self.config_dir = os.path.join(get_package_share_directory("loadconfig"), "config")

        self._declare_all_params()
        self._service = self.create_service(SetConfig, "loadconfig/set_config", self.handle_set_config)
        self._timer = self.create_timer(1.0, self._publish_params)

        self._load_default()
        self.get_logger().info("loadconfig_server started")

    def _declare_all_params(self):
        self.declare_parameter("patrol_route_id", self.manager.route_id)
        self.declare_parameter("patrol_cycles", self.manager.cycles)
        self.declare_parameter("patrol_points", self.manager.points)
        self.declare_parameter("patrol_trigger_mode", self.manager.trigger_mode)
        self.declare_parameter("patrol_trigger_time", self.manager.trigger_time)
        self.declare_parameter("patrol_trigger_duration_sec", self.manager.trigger_duration_sec)

    def _publish_params(self):
        data = self.manager.as_dict()
        for key, value in data.items():
            self.set_parameters([rclpy.parameter.Parameter(key, value=value)])

    def _load_default(self):
        default_path = os.path.join(self.config_dir, "default.yaml")
        ok, msg = self.manager.load_from_file(default_path)
        if ok:
            self._publish_params()
        else:
            self.get_logger().warning(f"loadconfig: default load failed: {msg}")

    def handle_set_config(self, request, response):
        config_id = (request.config_id or "default").strip()
        config_path = os.path.join(self.config_dir, f"{config_id}.yaml")
        ok, msg = self.manager.load_from_file(config_path)
        if ok:
            self._publish_params()
            self.get_logger().info(f"loadconfig: loaded {config_id}")
            response.ok = True
            response.message = "ok"
            return response

        self.get_logger().warning(f"loadconfig: failed to load {config_id}: {msg}")
        response.ok = False
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LoadConfigServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
