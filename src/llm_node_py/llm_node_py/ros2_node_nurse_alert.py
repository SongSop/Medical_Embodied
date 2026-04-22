# 用来判断是否需要呼叫护士

"""
2026-4-3 测试没啥问题
"""

"""
测试命令

ros2 service call /check_need_call_nurse llm_node_comm/srv/NurseAlert "{question: '我头痛、胸闷，呼吸急促'}"
ros2 service call /check_need_call_nurse llm_node_comm/srv/NurseAlert "{question: '帮我叫护士.'}"
ros2 service call /check_need_call_nurse llm_node_comm/srv/NurseAlert "{question: '我有点喘不过上来气.'}"
ros2 service call /check_need_call_nurse llm_node_comm/srv/NurseAlert "{question: '我头痛、胸闷，呼吸急促'}"
"""



import json
import os
import sys
from pathlib import Path
from typing import Optional

import rclpy
from openai import OpenAI
from rclpy.node import Node


def _iter_workspace_candidates() -> list[Path]:
    current_file = Path(__file__).resolve()
    candidates: list[Path] = []

    for parent in current_file.parents:
        if (parent / "src").exists() and (parent / "install").exists():
            candidates.append(parent)

    env_candidates = []
    for env_name in ("COLCON_CURRENT_PREFIX", "AMENT_PREFIX_PATH"):
        env_value = os.environ.get(env_name, "")
        if not env_value:
            continue
        for raw_path in env_value.split(os.pathsep):
            if not raw_path:
                continue
            path = Path(raw_path).resolve()
            env_candidates.extend([path, path.parent])

    for candidate in env_candidates:
        if candidate not in candidates and (candidate / "src").exists():
            candidates.append(candidate)

    return candidates


def resolve_ros2_interface_root(package_name: str = "llm_node_comm") -> Optional[str]:
    for workspace_root in _iter_workspace_candidates():
        install_dir = workspace_root / "install" / package_name / "lib"
        if install_dir.exists():
            for python_dir in sorted(install_dir.glob("python*/site-packages")):
                package_dir = python_dir / package_name
                if package_dir.exists():
                    return str(python_dir)

        build_dir = workspace_root / "build" / package_name / "rosidl_generator_py"
        if build_dir.exists():
            for python_dir in sorted(build_dir.glob("*/")):
                package_dir = python_dir / package_name
                if package_dir.exists():
                    return str(python_dir)

    return None


ROOT_DIR = resolve_ros2_interface_root()
if ROOT_DIR and ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from llm_node_comm.srv import NurseAlert





def _resolve_key_file() -> Path:
    current_file = Path(__file__).resolve()
    candidates = [
        current_file.parents[2] / "key" / "dashscope.key",
    ]

    for workspace_root in _iter_workspace_candidates():
        candidates.append(workspace_root / "src" / "llm_node_py" / "key" / "dashscope.key")
        candidates.append(
            workspace_root / "install" / "llm_node_py" / "share" / "llm_node_py" / "key" / "dashscope.key"
        )

    for path in candidates:
        if path.exists():
            return path

    raise FileNotFoundError(
        "未找到 DashScope key 文件，期望路径如 src/llm_node_py/key/dashscope.key"
    )


def load_dashscope_api_key() -> str:
    key_file = _resolve_key_file()
    api_key = key_file.read_text(encoding="utf-8").strip()
    if not api_key:
        raise ValueError(f"DashScope key 文件为空: {key_file}")
    return api_key


class NurseAlertService(Node):
    """
    ROS2 Node 提供服务，用于判断是否需要呼叫护士
    """

    def __init__(self):
        super().__init__("nurse_alert_service_node")

        os.environ.pop("http_proxy", None)
        os.environ.pop("https_proxy", None)
        os.environ.pop("HTTP_PROXY", None)
        os.environ.pop("HTTPS_PROXY", None)
        os.environ.pop("all_proxy", None)
        os.environ.pop("ALL_PROXY", None)

        self.client = OpenAI(
            api_key=load_dashscope_api_key(),
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )

        self.history = [
            {
                "role": "system",
                "content": (
                    "你是医院的护士助手，负责监测病人的对话内容。"
                    "你不执行实际动作，只根据病人的描述判断是否需要呼叫护士，"
                    "并给出原因。同时生成给患者的回复内容。"
                    "回答格式必须是 JSON，包含三个字段："
                    "如果不需要呼叫护士，reponse 字段输出为空，"
                    "need_call（bool，是否需要呼叫护士），"
                    "reason（string，解释原因），"
                    "response（string，给患者的回复内容）。"
                ),
            },
            {
                "role": "assistant",
                "content": (
                    "好的，在接下来的对话中我将根据输入的用户问题进行判断是否需要呼叫护士，"
                    "并严格按照json格式进行输出，如果不需要呼叫护士那么我会把response设置为空。"
                ),
            },
            {
                "role": "user",
                "content": "我头好痛，胸闷，呼吸急促，帮我叫护士。",
            },
            {
                "role": "assistant",
                "content": (
                    '{"need_call": true, '
                    '"reason": "患者出现胸闷和呼吸急促，属于可能紧急症状，需要立即通知护士。", '
                    '"response": "好的，我这就为您呼叫护士。"}'
                ),
            },
            {
                "role": "user",
                "content": "我平时饮食应该注意些什么？",
            },
            {
                "role": "assistant",
                "content": (
                    '{"need_call": false, '
                    '"reason": "这是日常咨询问题，不涉及紧急症状，不需要呼叫护士。", '
                    '"response": "null"}'
                ),
            },
            {
                "role": "user",
                "content": "我头有点痛，有什么办法可以缓解吗？",
            },
            {
                "role": "assistant",
                "content": (
                    '{"need_call": false, '
                    '"reason": "患者的症状程度较轻，只需要自己采取缓解措施，不需要呼叫护士。", '
                    '"response": "null"}'
                ),
            },
            {
                "role": "user",
                "content": "你还是帮我叫护士吧。",
            },
            {
                "role": "assistant",
                "content": (
                    '{"need_call": true, '
                    '"reason": "患者明确表明想要呼叫护士，需要立即通知护士。", '
                    '"response": "好的，我立刻给您去叫护士。"}'
                ),
            },
        ]

        self.service = self.create_service(
            NurseAlert,
            "check_need_call_nurse",
            self.handle_service,
        )
        self.get_logger().info(
            "Nurse Alert Service is ready. Service name: /check_need_call_nurse"
        )

    def handle_service(
        self,
        request: NurseAlert.Request,
        response: NurseAlert.Response,
    ) -> NurseAlert.Response:
        """
        接收 NurseAlert 请求，返回是否需要呼叫护士
        """
        question = request.question

        try:
            prompt = [{"role": "user", "content": question}]
            completion = self.client.chat.completions.create(
                model="qwen3.5-flash",
                messages=self.history + prompt,  # type: ignore[arg-type]
                stream=False,
                extra_body={"enable_thinking": False},
            )

            reply_text = completion.choices[0].message.content
            if reply_text is None:
                raise ValueError("模型返回为空")

            result = json.loads(reply_text)

            need_call = result.get("need_call", False)
            reason = result.get("reason", "")
            response_text = result.get("response", "")

            self.get_logger().info(
                f"Q: {question} | need_call: {need_call} | reason: {reason} | response: {response_text}"
            )

            response.need_call = need_call
            response.comment = reason
            response.response = response_text
            return response

        except Exception as exc:
            self.get_logger().error(f"Error in nurse alert service: {exc}")
            response.need_call = False
            response.comment = "[Error] Failed to query LLM"
            response.response = ""
            return response


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None
    try:
        node = NurseAlertService()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if "node" in locals() and node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
