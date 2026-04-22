# 用来判断用户是否想结束对话

import rospy
import json
import sys, os

# 把 ros 生成的 srv 文件的地址加上
ROOT_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), 
        "../../../devel/lib/python3/dist-packages"
    )
)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

# 当前目录
ROOT_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), 
        "./"
    )
)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from llm_node.srv import (
    end_session, 
    end_sessionRequest, 
    end_sessionResponse,
)
from openai import OpenAI
from get_dashscope_key import get_dashscope_key


class EndDialogService:
    """
    ROS Node 提供服务，用于判断用户是否想结束对话
    """
    def __init__(self):
        rospy.init_node("end_dialog_service_node")

        # 取消所有代理，保证 OpenAI 客户端可以正常初始化
        os.environ.pop("http_proxy", None)
        os.environ.pop("https_proxy", None)
        os.environ.pop("HTTP_PROXY", None)
        os.environ.pop("HTTPS_PROXY", None)
        os.environ.pop("all_proxy", None)
        os.environ.pop("ALL_PROXY", None)
        
        # 初始化 OpenAI 兼容客户端（以 DashScope 为例）
        self.client = OpenAI(
            api_key=get_dashscope_key(),
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
        )

        # 历史对话示例，用于模型理解
        self.history = [
            {
                "role": "system",
                "content": (
                    "你是聊天助手，负责监测用户输入，"
                    "判断用户是否想结束本次对话。"
                    "你只判断意图，不执行实际操作。"
                    "返回 JSON 格式："
                    "need_end (bool, 是否想结束对话),"
                    "reason (string, 判断原因),"
                    "response (string, 给用户的回复内容，如果不需要结束则为空)"
                )
            },
            {
                "role": "assistant",
                "content": (
                    "好的，我会根据用户的输入判断是否想结束对话，"
                    "并且我会严格输出 JSON 格式，如果不想结束对话则 response 为空。"
                )
            },
            {
                "role": "user",
                "content": "没什么事儿了，你走吧。"
            },
            {
                "role": "assistant",
                "content": (
                    '{"need_end": true, '
                    '"reason": "用户明确表示想结束聊天。", '
                    '"response": "好的，那我们下次再聊。"}'
                )
            },
            {
                "role": "user",
                "content": "没事儿了，你滚蛋吧。"
            },
            {
                "role": "assistant",
                "content": (
                    '{"need_end": true, '
                    '"reason": "用户明确表示想结束聊天。", '
                    '"response": "好的，那我们下次再聊。"}'
                )
            },
            {
                "role": "user",
                "content": "肾病患者应该注意写什么？"
            },
            {
                "role": "assistant",
                "content": (
                    '{"need_end": false, '
                    '"reason": "用户仍在进行咨询或聊天，没有表示结束意图。", '
                    '"response": ""}'
                )
            },
        ]

        # 创建 ROS Service
        self.service = rospy.Service(
            "check_end_dialog",
            end_session,  # 自定义 srv
            self.handle_service
        )
        rospy.loginfo("End Dialog Service is ready. Service name: /check_end_dialog")

    def handle_service(self, req: end_sessionRequest) -> end_sessionResponse:
        """
        接收请求，判断用户是否想结束对话
        """
        user_text = req.question

        try:
            prompt = [{"role": "user", "content": user_text}]
            completion = self.client.chat.completions.create(
                model="qwen3.5-flash",
                messages=self.history + prompt,  # type: ignore
                stream=False,
                extra_body={"enable_thinking": False}
            ) # type: ignore

            reply_text = completion.choices[0].message.content
            result = json.loads(reply_text)

            need_end = result.get("need_end", False)
            reason = result.get("reason", "")
            response_text = result.get("response", "")

            rospy.loginfo(f"User input: {user_text}\nneed_end: {need_end}\nreason: {reason}\nresponse: {response_text}")

            return end_sessionResponse(
                need_end=need_end,
                comment=reason,
                response=response_text
            )

        except Exception as e:
            rospy.logerr(f"Error in end dialog service: {str(e)}")
            return end_sessionResponse(
                need_end=False,
                comment="[Error] Failed to query LLM",
                response=""
            )


if __name__ == "__main__":
    try:
        node = EndDialogService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    