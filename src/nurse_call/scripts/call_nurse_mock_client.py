#!/usr/bin/env python3
import argparse
import rospy
import actionlib

from interfaces.msg import CallNurseAction, CallNurseGoal


STATUS_NAMES = {
    0: "OK",
    1: "TIMEOUT",
    2: "PREEMPTED",
    3: "NO_PATH",
    4: "ABORTED",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bed-ids", default="0")
    parser.add_argument("--summarys", default="")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    rospy.init_node("call_nurse_mock_client")
    client = actionlib.SimpleActionClient("call_nurse", CallNurseAction)
    if not client.wait_for_server(rospy.Duration(3.0)):
        rospy.logerr("call_nurse action server not available")
        return 1

    bed_ids = []
    for item in args.bed_ids.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            bed_ids.append(int(item))
        except ValueError:
            bed_ids.append(0)

    summarys = []
    if args.summarys:
        summarys = [s.strip() for s in args.summarys.split(",")]

    goal = CallNurseGoal(bed_ids=bed_ids, summarys=summarys)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(args.timeout))
    result = client.get_result()
    if result is None:
        rospy.logerr("no result")
        return 2

    status = result.status.status
    rospy.loginfo("call_nurse result=%s message=%s", STATUS_NAMES.get(status, str(status)), result.message)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
