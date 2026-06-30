#!/usr/bin/env python3
"""ROS2 node: std_msgs/String -> raspi-eye serial emotion frames."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError as exc:
    raise ImportError(
        'pyserial is required: sudo apt install python3-serial'
    ) from exc

from emotion.protocol import encode_emotion_name


class EmotionBridgeNode(Node):
    def __init__(self):
        super().__init__('emotion_bridge')
        self.declare_parameter('serial_port', '/dev/emotion')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('topic', 'emotion')
        self.declare_parameter('default_emotion', 'neutral')
        self.declare_parameter('emotions', ['neutral', 'happy', 'sad',
                                            'angry', 'surprised', 'curious'])
        self.declare_parameter('emotion_aliases', ['平静=neutral'])

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        self._default_emotion = self._normalize(
            self.get_parameter('default_emotion').get_parameter_value().string_value)
        self._emotions = self._load_emotion_list()
        self._aliases = self._load_aliases()

        if self._default_emotion not in self._emotions:
            self.get_logger().warn(
                f"default_emotion '{self._default_emotion}' not in emotions list, "
                f"using '{self._emotions[0]}'"
            )
            self._default_emotion = self._emotions[0]

        try:
            self._ser = serial.Serial(port, baudrate=baud, timeout=0)
        except serial.SerialException as exc:
            self.get_logger().error(
                f'Cannot open serial port {port}: {exc}. '
                f'Check serial_port in config/emotion_bridge.yaml.'
            )
            raise

        self.create_subscription(String, topic, self._on_emotion, 10)
        self.get_logger().info(
            f"listening on '{topic}' (serial {port} @ {baud}), "
            f"emotions: {', '.join(self._emotions)}, "
            f"default: {self._default_emotion}"
        )

    @staticmethod
    def _normalize(name: str) -> str:
        return name.strip().lower()

    def _load_emotion_list(self) -> list[str]:
        raw = self.get_parameter('emotions').value
        if not raw:
            raise ValueError('emotions list must not be empty')
        return [self._normalize(str(item)) for item in raw]

    def _load_aliases(self) -> dict[str, str]:
        raw = self.get_parameter('emotion_aliases').value
        if not raw:
            return {}
        aliases: dict[str, str] = {}
        for item in raw:
            text = str(item).strip()
            if '=' not in text:
                self.get_logger().warn(f"skip invalid emotion alias: {text!r}")
                continue
            key, value = text.split('=', 1)
            aliases[self._normalize(key)] = self._normalize(value)
        return aliases

    def _resolve_emotion(self, raw: str) -> tuple[str, bool]:
        key = self._normalize(raw)
        if not key:
            return self._default_emotion, False

        if key in self._aliases:
            key = self._aliases[key]

        if key in self._emotions:
            return key, True

        return self._default_emotion, False

    def _send_emotion(self, emotion: str) -> None:
        frame = encode_emotion_name(emotion)
        self._ser.write(frame)
        self._ser.flush()
        self.get_logger().info(f"sent emotion '{emotion}' ({frame.hex(' ')})")

    def _on_emotion(self, msg: String):
        emotion, matched = self._resolve_emotion(msg.data)
        if matched:
            self.get_logger().info(f"emotion '{msg.data.strip()}' -> '{emotion}'")
        else:
            self.get_logger().warn(
                f"emotion '{msg.data.strip()}' not in list, "
                f"fallback to '{emotion}' (平静)"
            )
        self._send_emotion(emotion)

    def destroy_node(self):
        if self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmotionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
