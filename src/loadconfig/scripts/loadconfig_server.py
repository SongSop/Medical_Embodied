#!/usr/bin/env python3
import os
import rospy
import rospkg
import yaml

from interfaces.srv import SetConfig, SetConfigResponse


class LoadConfigManager:
    def __init__(self):
        self.route_id = "route_a"
        self.cycles = 2
        self.points = ["p0", "p1"]
        self.trigger_mode = "button"
        self.trigger_time = "08:00"
        self.trigger_duration_sec = 5

    def load_from_file(self, path):
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

    def publish_params(self):
        rospy.set_param("/loadconfig/patrol_route_id", self.route_id)
        rospy.set_param("/loadconfig/patrol_cycles", int(self.cycles))
        rospy.set_param("/loadconfig/patrol_points", list(self.points))
        rospy.set_param("/loadconfig/patrol_trigger_mode", self.trigger_mode)
        rospy.set_param("/loadconfig/patrol_trigger_time", self.trigger_time)
        rospy.set_param("/loadconfig/patrol_trigger_duration_sec", int(self.trigger_duration_sec))


class LoadConfigServer:
    def __init__(self):
        self.manager = LoadConfigManager()
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path("loadconfig")
        self.config_dir = os.path.join(self.pkg_path, "config")
        self.service = rospy.Service("loadconfig/set_config", SetConfig, self.handle_set_config)
        self._load_default()

    def _load_default(self):
        default_path = os.path.join(self.config_dir, "default.yaml")
        ok, msg = self.manager.load_from_file(default_path)
        if ok:
            self.manager.publish_params()
        else:
            rospy.logwarn("loadconfig: default load failed: %s", msg)

    def handle_set_config(self, req):
        config_id = (req.config_id or "default").strip()
        config_path = os.path.join(self.config_dir, f"{config_id}.yaml")
        ok, msg = self.manager.load_from_file(config_path)
        if ok:
            self.manager.publish_params()
            rospy.loginfo("loadconfig: loaded %s", config_id)
            return SetConfigResponse(ok=True, message="ok")
        rospy.logwarn("loadconfig: failed to load %s: %s", config_id, msg)
        return SetConfigResponse(ok=False, message=msg)


def main():
    rospy.init_node("loadconfig_server")
    LoadConfigServer()
    rospy.loginfo("loadconfig_server started")
    rospy.spin()


if __name__ == "__main__":
    main()
