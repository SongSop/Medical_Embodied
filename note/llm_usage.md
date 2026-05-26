# llm node usage doc

## 启动

```bash
# cd 到启动脚本所在的位置
cd ~/Medical_Embodied/launch/

# 运行启动脚本
# 不仅是启动脚本，也是重启脚本（会自动kill掉老 node，然后运行新的 node）
./launch_llm_nodes.sh
# 启动后会生成三个 tmux session
```

## 观察 node 输出

先右键，创建额外三个标签：

![](./llm_usage.assets/multi_lable.png)

```bash
# 在一个 terminal 标签中（连接到 depends_asr_question tmux session）：
tmux attach -t depends_asr_question

# 在第二个 terminal 标签中：（连接到 depends_ui_question tmux session）：
tmux attach -t depends_ui_question

# 在第三个 terminal 标签中：（连接到 dialog_manager tmux session）：
tmux attach -t dialog_manager
```

左下角是session 名称，点不同的位置（‘0 python’， ‘1 python’）切换到不同的node输出

![image-20260510221351202](./llm_usage.assets/image-20260510221351202.png)

## 测试 

```bash
# cd 到脚本所在的位置
cd ~/Medical_Embodied/src/llm_node_py/llm_node_py/

# 启动对应的 conda 环境
conda activate qwen_tts_online

# 以不同的 mode 运行测试脚本
python test_hci_action.py --mode passive
python test_hci_action.py --mode interrupt
python test_hci_action.py --mode alert
```

