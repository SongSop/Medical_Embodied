# Medical_Embodied
医疗接诊巡诊、大预言交互、异常检测

## 一、行为树项目安装指南（可选，便于测试）
### 项目地址 https://github.com/BehaviorTree/BehaviorTree.CPP
### 安装、编译、在ros中使用指南
1、安装conan并执行默认初始化程序

    pip install conan
    conan profile detect

2、选择一个文件夹(这里以/home/val/Downloads为例)，进入文件夹后执行 

    git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git

3、进入项目根目录 `/home/val/Downloads/BehaviorTree.CPP`，依次执行执行以下指令

    conan install . -s build_type=Release --build=missing
    cmake --preset conan-release
    cmake --build --preset conan-release

以上指令中第一行是自动搜索依赖然后拉取源进行构建安装，第二行是生成CMake配置编译器，第三行才开始编译

4、编译成功后，进入项目根目录中的`build\Release`目录，这里保存了所有编译成功的结果，然后再`Release`目录下执行

    cmake --install . --prefix /home/val/Downloads/BehaviorTree.CPP/install
以上指令会把编译好的库安装在`/home/val/Downloads/BehaviorTree.CPP/install`文件夹下，完成安装

5、编写CMakeLists.txt

设定安装路径，寻找包，添加可执行文件，对库进行连接

    set(behaviortree_cpp_DIR "/home/val/Downloads/BehaviorTree.CPP/install/lib/cmake/behaviortree_cpp")
    find_package(behaviortree_cpp REQUIRED)
    add_executable(your_node src/your_node.cpp)
    target_link_libraries(your_node BT::behaviortree_cpp)

## 二、功能包与节点说明

`behavior_tree`  
作用：行为树执行与调度（BT.CPP 4.x），`ros_bt_runner` 负责将 ROS topic/service/action 映射为 BT 节点。  
接口：订阅 `/battery`、`/fault`、`/call_signal`、`/patrol_triggered`。调用 `navigate`、`llm_interaction`、`call_nurse` 三个 action。调用 `~detect_anomaly`、`~patrol_trigger`、`~face_identify` 服务（私有命名空间：`/ros_bt_runner/...`），以及全局服务 `/loadconfig/set_config`。  
BT 节点映射：`NavgateTo`->`Navigate.action`（`nav_type`: `goal/stop/dock`）；`Detect_BedProcess`->`DetectAnomaly.srv`（`mode`: `area/bed`，`area_bed_id`）；`FaceIdentify`->`FaceIdentify.srv`；`LLMInteraction`->`LLMInteraction.action`（`mode`: `alert/passive/interrupt`）；`CallDutyNurse`->`CallNurse.action`。  
需实现：BT XML 逻辑迭代与稳定性（在 `src/behavior_tree/BH_xml/medical.xml`），真实硬件/算法节点替换 mock server。

`interfaces`  
作用：统一消息/服务/动作定义。  
接口：  
Action：  
- `Navigate.action`（goal: `target_index`, `nav_type`; result: `status`, `message`; feedback: `progress`）  
- `LLMInteraction.action`（goal: `mode`, `person_id`, `context`; result: `status`, `summary`, `need_call_nurse`; feedback: `partial`）  
- `CallNurse.action`（goal: `bed_ids[]`, `summarys[]`; result: `status`, `message`; feedback: `progress`）  
Service：  
- `DetectAnomaly.srv`（req: `mode`, `area_bed_id`; res: `is_anomaly`, `details`, `bed_ids[]`, `urgencies[]`）  
- `PatrolTrigger.srv`、`Dock.srv`、`ChargeUntil.srv`、`FaceIdentify.srv`、`SetConfig.srv`  
Message：`Battery.msg`、`Fault.msg`、`ActionStatus.msg` 等。  
需实现：对接真实系统时保持接口兼容。

`monitor`  
作用：模拟监控输入（电量、故障、呼叫信号）与异常检测/人脸识别服务。  
接口：发布 `/battery`、`/fault`、`/call_signal`；提供 `/ros_bt_runner/detect_anomaly`、`/ros_bt_runner/face_identify`。  
需实现：接入真实监控数据源与异常检测算法。

`nav`  
作用：导航 action mock。  
接口：`navigate` action（goal: `target_index`, `nav_type`；`nav_type`: `goal=0`、`stop=1`、`dock=2`）。  
需实现：接入真实导航栈与路径执行。

`dialog`  
作用：对话 action mock。  
接口：`llm_interaction` action（goal: `mode`, `person_id`, `context`；result: `need_call_nurse`；`mode`: `alert=0`、`passive=1`、`interrupt=2`）。  
需实现：接入真实对话引擎与上下文管理。

`nurse_call`  
作用：呼叫护士 action mock。  
接口：`call_nurse` action（goal: `bed_ids[]`, `summarys[]`）。  
需实现：接入护士呼叫系统与业务流程。

`patrol`  
作用：巡检触发服务与状态发布。  
接口：`patrol_trigger` service，发布 `/patrol_triggered`。  
需实现：接入真实巡检调度触发逻辑。

`charge`  
作用：充电对接与充电目标服务 mock。  
接口：`dock` service（`Dock.srv`）、`charge_until` service（`ChargeUntil.srv`）。  
需实现：接入真实充电桩/回桩控制与 SOC 反馈闭环。

`loadconfig`  
作用：读取巡检配置并发布参数。  
接口：`/loadconfig/set_config` service（`SetConfig.srv`），发布参数 `/loadconfig/patrol_route_id`、`/loadconfig/patrol_cycles`、`/loadconfig/patrol_points` 等。  
需实现：接入真实配置管理系统。

## 三、BT 测试与参考实现

`src/behavior_tree/src/medical_bt_test.cpp`  
作用：纯 C++ 行为树单元测试。用本地 mock 节点替代 ROS，用日志验证每条分支是否按预期执行。  
使用方式：直接运行生成的测试二进制，查看 `logs/medical_bt_test.log`。  
参考点：每个 mock 节点的输入/输出与日志格式，方便你对照真实节点需要产生的状态与行为。

`src/behavior_tree/src/ros_bt_runner.cpp`  
作用：ROS 版行为树执行器，将话题/服务/action 与 BT 节点绑定。  
关键绑定：  
- 订阅 `/battery` `/fault` `/call_signal` `/patrol_triggered`，更新黑板状态。  
- 调用 `~detect_anomaly` `~face_identify` `~patrol_trigger` 服务，驱动异常检测/人脸识别/触发控制。  
- 调用 `navigate` `llm_interaction` `call_nurse` action，驱动执行节点。  
参考点：节点输入输出字段、黑板键名、以及故障/电量/呼叫等触发条件的判断。

`src/behavior_tree/scripts/medical_bt_ros_test_driver.py`  
作用：ROS 端集成测试驱动器，提供完整 mock（topic/service/action）并输出结构化 log。  
你写新节点时的参考：  
- 如何设计可测的 action/service 接口（输入/输出尽量小而明确）。  
- 如何驱动黑板触发条件（call_signal、fault、battery 等）。  
- 日志格式 `[SET]/[ACT]/[COND]/[START]/[RUN]/[DONE]`，便于和 `medical_bt_test.cpp` 对齐。
