#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
最小本地 TTS ROS2 服务节点。

服务名称和接口格式对齐 ros2_node_tts_oneshot.py：

ros2 service call /tts_one_shot llm_node_comm/srv/TtsOneshot "{tts_text: '你好，我在呢。', block: true}"

"""

import os
import sys
import threading

import numpy as np
import pyaudio
import rclpy
import torch
from qwen_tts import Qwen3TTSModel
from rclpy.node import Node


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
        self.model = Qwen3TTSModel.from_pretrained(
            MODEL_PATH,
            device_map="cuda:0",
            dtype=torch.bfloat16,
        )

        self.service = self.create_service(
            TtsOneshot,
            "tts_one_shot",
            self.handleTtsOneShotService,
        )

        self.get_logger().info("本地 TTS 服务节点启动完成，等待调用...")

    def runTtsOneShot(self, text: str) -> None:
        with self.model_lock:
            wavs, sample_rate = self.model.generate_custom_voice(
                text=text,
                language="Chinese",
                speaker="Vivian",
                instruct="",
            )

        player = pyaudio.PyAudio()
        stream = player.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=sample_rate,
            output=True,
        )

        try:
            stream.write(wavs[0].astype(np.float32).tobytes())
        finally:
            stream.stop_stream()
            stream.close()
            player.terminate()

    def handleTtsOneShotService(
        self,
        req: TtsOneshot.Request,
        res: TtsOneshot.Response,
    ):
        self.get_logger().info(f"收到本地 TTS 请求: {req.tts_text}")

        try:
            if req.block:
                self.runTtsOneShot(req.tts_text)
            else:
                threading.Thread(
                    target=self.runTtsOneShot,
                    args=(req.tts_text,),
                    daemon=True,
                ).start()

            res.result = True
        except Exception as exc:
            self.get_logger().error(f"本地 TTS 执行失败: {exc}")
            res.result = False

        return res


def main(args=None):
    rclpy.init(args=args)
    node = LocalTtsServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
