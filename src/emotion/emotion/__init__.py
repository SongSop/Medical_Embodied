"""Emotion bridge: ROS2 std_msgs/String -> raspi-eye serial frames."""

from emotion.protocol import EMOTIONS, encode_emotion_name, encode_set_emotion

__all__ = ['EMOTIONS', 'encode_emotion_name', 'encode_set_emotion']
