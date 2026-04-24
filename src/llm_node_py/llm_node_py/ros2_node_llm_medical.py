import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http import HTTPStatus
from dashscope import Application
import threading
import time


"""
测试这个节点的时候也要把 dialog history 节点启动

向 llm medical 节点发送消息：

ros2 topic pub -1 /llm_to_dashapp std_msgs/msg/String "data: '肾病患者应该注意些什么？'"

 

监听发送过来的消息：

ros2 topic echo /ali_llm_output

"""

# input  -> llm_to_dashapp
# dialog history input -> get_llm_dialog_history
# output -> ali_llm_output(tts), huzhou_llm_res(ui), send_to_dialog_history

import os, sys
# 把 ros 生成的 srv 文件的地址加上（ROS2中可能需要调整路径，保留原逻辑）
ROOT_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), 
        "../../../install/llm_node_comm/lib/python3.12/site-packages"
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

from llm_node_comm.msg import LlmDialogHistoryMsg
from llm_node_comm.srv import LlmDialogHistorysSrv
from get_dashscope_key import get_dashscope_key

class KnowledgeBaseLLMNode(Node):

    def __init__(self):
        super().__init__('knowledgebase_llm_node')

        # 订阅问题输入
        self.sub = self.create_subscription(
            String,
            "/llm_to_dashapp",
            self.query_callback,
            10
        )

        # 发布模型流式输出
        self.pub = self.create_publisher(
            String,
            "/ali_llm_output",
            10
        )

        # UI 输出（可选，注释保留）
        # self.ui_pub = self.create_publisher(
        #     String,
        #     "/huzhou_llm_res",
        #     10
        # )

        # 发送到 dialog_history 的 topic
        self.dialog_pub = self.create_publisher(
            LlmDialogHistoryMsg,
            "/send_to_dialog_history",
            40
        )

        # 初始化 client，访问 dialog history server 得到对话历史
        self.dialog_history_client = self.create_client(
            LlmDialogHistorysSrv,
            "/get_llm_dialog_history"
        )

        self.get_logger().info("KnowledgeBase LLM Node started.")


    def query_callback(self, msg: String):
        question = msg.data.strip()
        self.get_logger().info(f"Received question: {question}")
        threading.Thread(target=self.handle_query, args=(question,), daemon=True).start()


    # 似乎不能在 query_callback 中去 call 一个 service，否则会卡住
    def handle_query(self, question: str):
        # 用于拼接完整回答
        full_response = ""

        try:
            # 等待服务可用
            if not self.dialog_history_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Service /get_llm_dialog_history not available")
                return

            # 调用 dialog history service 获取历史
            request = LlmDialogHistorysSrv.Request()
            request.clear_after_get = False
            future = self.dialog_history_client.call_async(request)
            while rclpy.ok() and not future.done():
                time.sleep(0.05)

            response = future.result()
            if response is None:
                raise RuntimeError("Failed to get dialog history")

            # 构造 dialog history
            history_list = []
            history_list.append(
                {
                    "user":
"有几个血透中心的病人，现在在血透中心做透析，你是巡诊的机器人，名字叫小医。你的大脑跟病人的测量数据是实时联系在一起的，下面几个病人开始对话。对话的时候不要长篇大论，像真人一样，使用对话句子，也就是我们一句来一句去这样。语气需要轻柔舒缓。所有使用单位名称转化为中文。尽可能不出现英文。你只需要监测，禁止输出执行实际动作,例如倒水。不要增加小动作。不要输出表情符号。在需要时，给出数据偏高或偏低，不需要具体数据。不要给出实际的执行动作，而是不用通过类似读取的方式。讲话必须亲切，温柔。科普类的内容可以丰富一些。叫护士或医生时回答好的，我马上帮您联系护士或医生。您先安心休息一会儿。禁止在回答中出现着急两个字。问题不严重时，你来提出解决办法，不要经常输出寻找医生。询问天气/温度等关键词，仅输出杭州本地核心信息.",
                    "bot": 
"好的，我记住了,我的名字是小医，我会采用真人对话风格那样进行对话，并使用轻柔温和的语气，我不会输出执行动作，不会输出表情符合，不会给出具体的数值，问题不严重时我会自己提出解决办法不会寻求医生的帮助，不会使用'着急'等急迫性质的词语。"
                }
            )

            for item in response.history:
                history_list.append({
                    "user": item.question,
                    "bot": item.response
                })

            # 向 LLM 发送问题
            responses = Application.call(
                api_key=get_dashscope_key(),
                app_id='b7ed8dad5c0c483d8e865a3be0b4e088',
                prompt=question,
                history=history_list,
                stream=True,
                incremental_output=True
            )

            # 通知开始
            self.pub.publish(String(data="[START]"))
            # self.ui_pub.publish(String(data="[START]"))

            # 流式接收
            for response in responses:
                if response.status_code == HTTPStatus.OK:
                    chunk = response.output.text
                    full_response += chunk
                    self.pub.publish(String(data=chunk))
                    # self.ui_pub.publish(String(data=chunk))
                else:
                    self.get_logger().error(f"LLM response error: {response}")
                    break

            # 通知结束
            self.pub.publish(String(data="[DONE]"))
            # self.ui_pub.publish(String(data="[DONE]"))

            # 发送完整问答到 dialog_history
            dialog_msg = LlmDialogHistoryMsg()
            dialog_msg.question = question
            dialog_msg.response = full_response.strip()
            self.dialog_pub.publish(dialog_msg)

            self.get_logger().info("Dialog pair sent to dialog_history.")

        except Exception as e:
            self.get_logger().error(f"Error querying KnowledgeBase LLM: {str(e)}")
            self.pub.publish(String(data="[ERROR] Failed to get response."))


def main(args=None):
    rclpy.init(args=args)
    node = KnowledgeBaseLLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
