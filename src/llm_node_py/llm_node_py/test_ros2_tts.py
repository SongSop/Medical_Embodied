#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_args():
    parser = argparse.ArgumentParser(description="High-speed ROS2 realtime TTS publisher")
    parser.add_argument("--interval", type=float, default=0.01, help="interval between text messages (sec)")
    parser.add_argument("--repeat", type=int, default=1, help="number of repeated batches")
    parser.add_argument("--batch-gap", type=float, default=0.2, help="gap between batches (sec)")
    parser.add_argument("--startup-wait", type=float, default=1.0, help="wait after publisher creation (sec)")
    parser.add_argument("--after-start-wait", type=float, default=0.3, help="wait after [START] before text burst (sec)")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    node = Node("test_pub")
    pub = node.create_publisher(String, "/tts_realtime_data", 200)

    def send(text: str):
        msg = String()
        msg.data = text
        pub.publish(msg)
        node.get_logger().info(f"send: {text}")
        rclpy.spin_once(node, timeout_sec=0.0)

    texts = [
        "你好，我是小医。",
        "很高兴为你提供帮助。",
        "请问你今天有什么不舒服的地方吗？",
        "比如说头痛，发烧，或者咳嗽等症状。",
        "如果有的话，可以尽量详细描述一下。",
        "我会根据你的描述，给出一些初步的建议。",
        "当然，这些建议不能替代专业医生的诊断。",
        "如果症状比较严重，建议尽快去医院就诊。",
        "现在你可以告诉我你的具体情况。",
    ]

    time.sleep(args.startup_wait)
    send("[START]")
    time.sleep(max(args.after_start_wait, 0.0))

    for batch in range(args.repeat):
        for t in texts:
            send(t)
            time.sleep(max(args.interval, 0.0))
        node.get_logger().info(f"batch finished: {batch + 1}/{args.repeat}")
        if batch < args.repeat - 1:
            time.sleep(max(args.batch_gap, 0.0))

    send("[DONE]")
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
