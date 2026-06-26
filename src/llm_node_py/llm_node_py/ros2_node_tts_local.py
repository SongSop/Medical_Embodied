#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
最小本地 TTS ROS2 服务节点。

服务名称和接口格式对齐 ros2_node_tts_oneshot.py：

# 一次性 tts 服务的调用
ros2 service call /tts_one_shot llm_node_comm/srv/TtsOneshot "{tts_text: '你好，我在呢。', block: true}"

ros2 topic echo /tts_session_finished

ros2 topic pub --once /tts_realtime_data std_msgs/msg/String "{data: '你好，我在呢。'}"
ros2 topic pub --once /tts_realtime_data std_msgs/msg/String "{data: '[DONE]'}"


"""

import os
import sys
import threading
from collections import deque

import numpy as np
import pyaudio
import rclpy
import torch
from qwen_tts import Qwen3TTSModel
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


ROOT_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "../../../install/llm_node_comm/lib/python3.12/site-packages",
    )
)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)


from llm_node_comm.srv import TtsOneshot


MODEL_PATH = (
    "/home/medical/Medical_Embodied/src/llm_node_py/"
    "Qwen3-TTS-12Hz-0.6B-CustomVoice"
)


class LocalTtsServiceNode(Node):
    def __init__(self):
        super().__init__("ros_node_tts_local")

        self.model_lock = threading.Lock()
        self.queueLock = threading.Lock()
        self.ttsRealtimeQueue = deque()
        self.ttsOneShotServiceCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.ttsRealtimeDataTopicCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.ttsRealtimeQueueCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.model = Qwen3TTSModel.from_pretrained(
            MODEL_PATH,
            device_map="cuda:0",
            dtype=torch.bfloat16,

            # 使用 flash attention2 加速
            attn_implementation="flash_attention_2",
        )

        self.service = self.create_service(
            TtsOneshot,
            "tts_one_shot",
            self.handleTtsOneShotService,
            callback_group=self.ttsOneShotServiceCallbackGroup,
        )
        self.ttsSessionFinishedPublisher = self.create_publisher(
            String,
            "/tts_session_finished",
            10,
        )
        self.ttsRealtimeDataSubscriber = self.create_subscription(
            String,
            "/tts_realtime_data",
            self.handleTtsRealtimeDataTopic,
            200,
            callback_group=self.ttsRealtimeDataTopicCallbackGroup,
        )
        self.ttsRealtimeTimer = self.create_timer(
            0.1,
            self.handleTtsRealtimeQueue,
            callback_group=self.ttsRealtimeQueueCallbackGroup,
        )

        self.get_logger().info("本地 TTS 服务节点启动完成，等待调用...")

    def runTtsOneShot(self, text: str):
        with self.model_lock:
            wavs, sample_rate = self.model.generate_custom_voice(
                text=text,
                language="Chinese",
                speaker="Serena",
                # instruct="用温柔、自然的语气说话，语速稍慢。",
            )

        return wavs[0], sample_rate

    def playTtsAudio(self, audio: np.ndarray, sample_rate: int) -> None:
        player = pyaudio.PyAudio()
        stream = player.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=sample_rate,
            output=True,
        )

        try:
            stream.write(audio.astype(np.float32).tobytes())
        finally:
            stream.stop_stream()
            stream.close()
            player.terminate()

    def publishTtsSessionFinished(self) -> None:
        msg = String()
        msg.data = "finished"
        self.ttsSessionFinishedPublisher.publish(msg)

    # 一次性的 tts 调用，有 阻塞和非阻塞的 区别
    def handleTtsOneShotService(
        self,
        req: TtsOneshot.Request,
        res: TtsOneshot.Response,
    ):
        self.get_logger().info(f"收到本地 TTS 请求: {req.tts_text}")

        try:
            if req.block:
                audio, sample_rate = self.runTtsOneShot(req.tts_text)
                self.playTtsAudio(audio, sample_rate)
            else:
                threading.Thread(
                    target=self.runTtsOneShotInBackground,
                    args=(req.tts_text,),
                    daemon=True,
                ).start()

            res.result = True
        except Exception as exc:
            self.get_logger().error(f"本地 TTS 执行失败: {exc}")
            res.result = False

        return res

    def runTtsOneShotInBackground(self, text: str) -> None:
        audio, sample_rate = self.runTtsOneShot(text)
        self.playTtsAudio(audio, sample_rate)

    def handleTtsRealtimeDataTopic(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            self.get_logger().info("收到空的 /tts_realtime_data 文本，忽略")
            return

        if text == "[START]":
            return

        if text == "[DONE]":
            with self.queueLock:
                self.ttsRealtimeQueue.append(("[DONE]", None, None))
            return

        try:
            audio, sample_rate = self.runTtsOneShot(text)
        except Exception as exc:
            self.get_logger().error(f"/tts_realtime_data 本地 TTS 合成失败: {exc}")
            return

        with self.queueLock:
            self.ttsRealtimeQueue.append((text, audio, sample_rate))

    def handleTtsRealtimeQueue(self) -> None:
        with self.queueLock:
            if not self.ttsRealtimeQueue:
                return
            text, audio, sample_rate = self.ttsRealtimeQueue.popleft()

        print(f"正在播放：{text}")

        # 遇到 "[DONE]" 的时候说明，之前所有的语句都全部合成完了
        if text == "[DONE]":
            self.publishTtsSessionFinished()
            return

        try:
            self.playTtsAudio(audio, sample_rate)
        except Exception as exc:
            self.get_logger().error(f"/tts_realtime_data 本地 TTS 播放失败: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = LocalTtsServiceNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
