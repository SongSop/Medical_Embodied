此目录下的脚本用来启动对应的 llm nodes

launch_ros_nodes.sh : 
输入对应的 py 路径或者 c++ 可执行文件路径，
把他们放到 tmux session 中执行

launch_depends_ui_question.sh ：
调用 launch_ros_nodes.sh 文件，
将 bridge, stream text, tts realtime, ui llm, question manager
放到 depends_ui_question session 中执行

launch_depends_asr_question.sh:
调用 launch_ros_nodes.sh 文件，
把剩下的 py 和 c++ 节点，
放到 depends_ui_question session 中执行

launch_dialog_manager_tmux.sh:
调用 launch_ros_nodes.sh 文件，
把 node manager，
放到 depends_ui_question session 中执行

launch_llm_nodes.sh:
启用上述三个文件

启动节点时只需要启动 launch_llm_nodes.sh
