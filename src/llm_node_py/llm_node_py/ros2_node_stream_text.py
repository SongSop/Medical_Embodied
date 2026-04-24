
"""
2026-4-3 测试没啥问题
"""


"""
发布消息给 stream text

ros2 topic pub -1 /ali_llm_output std_msgs/String "data: '你'"
ros2 topic pub -1 /ali_llm_output std_msgs/String "data: '好，'"

ros2 topic pub -1 /ali_llm_output std_msgs/String "data: '这个是一条'"
ros2 topic pub -1 /ali_llm_output std_msgs/String "data: '测试消息。'"

监听：

ros2 topic echo /tts_realtime_data
"""

import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SentenceSplitterNode(Node):
    """
    ROS 节点: 接收大模型的零碎文本，并按中文逗号、句号、分号拆分成完整句子
    支持 [START] / [DONE] 控制
    """

    def __init__(self):
        super().__init__("sentence_splitter_node")

        self.buffer = ""

        self.sub = self.create_subscription(
            String,
            "/ali_llm_output",
            self.on_text,
            500,
        )

        self.pub = self.create_publisher(
            String,
            "/tts_realtime_data",
            100,
        )

        self.get_logger().info("Sentence Splitter Node ready")

    def on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        if text == "[START]":
            self.buffer = ""
            start_msg = String()
            start_msg.data = "[START]"
            self.pub.publish(start_msg)
            self.get_logger().info("[CONTROL] [START] received, buffer cleared")
            return

        if text == "[DONE]":
            if self.buffer:
                remaining_msg = String()
                remaining_msg.data = self.buffer
                self.pub.publish(remaining_msg)
                self.get_logger().info(
                    f"[BUFFER] Remaining buffer sent: {self.buffer}"
                )
                self.buffer = ""

            done_msg = String()
            done_msg.data = "[DONE]"
            self.pub.publish(done_msg)
            self.get_logger().info("[CONTROL] [DONE] received, buffer flushed")
            return

        self.buffer += text

        pattern = r"[^，。；]*[，。；]?"
        parts = re.findall(pattern, self.buffer)

        new_buffer = ""

        for part in parts:
            part = part.strip()
            if not part:
                continue

            if part[-1] in "。；":
                sentence_msg = String()
                sentence_msg.data = part
                self.pub.publish(sentence_msg)
                self.get_logger().info(f"[SENTENCE] {part}")
            elif part[-1] == "，":
                fragment_msg = String()
                fragment_msg.data = part
                self.pub.publish(fragment_msg)
                self.get_logger().info(f"[FRAGMENT] {part}")
            else:
                new_buffer += part

        self.buffer = new_buffer


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None
    try:
        node = SentenceSplitterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
