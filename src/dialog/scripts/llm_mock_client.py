#!/usr/bin/env python3
import argparse
import rospy
import actionlib

from interfaces.msg import LLMInteractionAction, LLMInteractionGoal


STATUS_NAMES = {
    0: "OK",
    1: "TIMEOUT",
    2: "PREEMPTED",
    3: "NO_PATH",
    4: "ABORTED",
}

INTERACTION_ALERT = 0
INTERACTION_PASSIVE = 1
INTERACTION_INTERRUPT = 2

MODE_ALIASES = {
    "alert": INTERACTION_ALERT,
    "passive": INTERACTION_PASSIVE,
    "interrupt": INTERACTION_INTERRUPT,
    "abnormal_alert": INTERACTION_ALERT,
    "passive_wakeup": INTERACTION_PASSIVE,
    "call_response": INTERACTION_INTERRUPT,
}


def parse_mode(value):
    if isinstance(value, int):
        return value
    if value.isdigit() or (value.startswith("-") and value[1:].isdigit()):
        return int(value)
    if value in MODE_ALIASES:
        return MODE_ALIASES[value]
    raise ValueError(f"unknown mode: {value}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="alert")
    parser.add_argument("--person-id", type=int, default=1)
    parser.add_argument("--context", default="")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    rospy.init_node("llm_mock_client")
    client = actionlib.SimpleActionClient("llm_interaction", LLMInteractionAction)
    if not client.wait_for_server(rospy.Duration(3.0)):
        rospy.logerr("llm action server not available")
        return 1

    try:
        mode_value = parse_mode(args.mode)
    except ValueError as exc:
        rospy.logerr(str(exc))
        return 2

    goal = LLMInteractionGoal(mode=mode_value, person_id=args.person_id, context=args.context)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(args.timeout))
    result = client.get_result()
    if result is None:
        rospy.logerr("no result")
        return 2

    status = result.status.status
    rospy.loginfo(
        "llm result=%s need_call_nurse=%s summary=%s",
        STATUS_NAMES.get(status, str(status)),
        str(result.need_call_nurse).lower(),
        result.summary,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
