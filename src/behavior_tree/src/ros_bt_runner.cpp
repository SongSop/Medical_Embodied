#include "behaviortree_cpp/bt_factory.h"

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "interfaces/ActionStatus.h"
#include "interfaces/Battery.h"
#include "interfaces/CallNurseAction.h"
#include "interfaces/FaceIdentify.h"
#include "interfaces/Fault.h"
#include "interfaces/LLMInteractionAction.h"
#include "interfaces/NavigateAction.h"
#include "interfaces/DetectAnomaly.h"
#include "interfaces/SetConfig.h"
#include "interfaces/FaceIdentify.h"
#include "loadconfig/mode_define.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <xmlrpcpp/XmlRpcValue.h>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace BT;

namespace {

int navTypeFromString(const std::string& s)
{
    if (!s.empty())
    {
        char* end = nullptr;
        long v = std::strtol(s.c_str(), &end, 10);
        if (end && *end == '\0')
        {
            return static_cast<int>(v);
        }
    }
    if (s == "goal")
    {
        return NAVIGATION::GOAL;
    }
    if (s == "stop")
    {
        return NAVIGATION::STOP;
    }
    if (s == "dock")
    {
        return NAVIGATION::DOCK;
    }
    return NAVIGATION::STOP;
}

int detectModeFromString(const std::string& s)
{
    if (!s.empty())
    {
        char* end = nullptr;
        long v = std::strtol(s.c_str(), &end, 10);
        if (end && *end == '\0')
        {
            return static_cast<int>(v);
        }
    }
    if (s == "area")
    {
        return DETECT::AREA;
    }
    if (s == "bed")
    {
        return DETECT::BED;
    }
    return DETECT::AREA;
}

int interactionModeFromString(const std::string& s)
{
    if (!s.empty())
    {
        char* end = nullptr;
        long v = std::strtol(s.c_str(), &end, 10);
        if (end && *end == '\0')
        {
            return static_cast<int>(v);
        }
    }
    if (s == "passive")
    {
        return INTERACTION::PASSIVE;
    }
    if (s == "alert")
    {
        return INTERACTION::ALERT;
    }
    if (s == "interrupt")
    {
        return INTERACTION::INTERUPT;
    }
    return INTERACTION::PASSIVE;
}

struct RosContext
{
    ros::NodeHandle nh;
    ros::Subscriber battery_sub;
    ros::Subscriber fault_sub;
    ros::Subscriber call_signal_sub;
    ros::Subscriber patrol_trigger_sub;
    ros::ServiceClient anomaly_client;
    ros::ServiceClient face_identify_client;

    float battery_soc = 100.0f;
    bool battery_charging = false;
    float battery_voltage = 0.0f;
    std::string fault_type;
    int fault_severity = 0;
    bool call_signal = false;
    bool patrol_triggered = false;
    double battery_low_threshold = 20.0;
};

Blackboard::Ptr g_root_bb;
std::shared_ptr<RosContext> g_ctx;
ros::ServiceClient g_loadconfig_client;
std::string g_config_id = "default";
bool g_config_loaded = false;

struct PatrolContext
{
    std::string route_id = "route_a";
    int cycles_total = 1;
    int cycles_remaining = 1;
    int point_index = 0;
    std::vector<std::string> points = {"p0", "p1"};
    bool complete = false;
};

std::shared_ptr<PatrolContext> g_patrol_ctx;

bool readPatrolPoints(std::vector<std::string>& out_points)
{
    XmlRpc::XmlRpcValue value;
    if (!ros::param::get("/loadconfig/patrol_points", value))
    {
        return false;
    }
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return false;
    }
    out_points.clear();
    for (int i = 0; i < value.size(); ++i)
    {
        if (value[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            continue;
        }
        out_points.emplace_back(static_cast<std::string>(value[i]));
    }
    return !out_points.empty();
}

bool getBool(const Blackboard::Ptr& bb, const std::string& key, bool default_value = false)
{
    bool value = default_value;
    if (!bb->get(key, value))
    {
        return default_value;
    }
    return value;
}

bool getBool(const Blackboard* bb, const std::string& key, bool default_value = false)
{
    if (!bb)
    {
        return default_value;
    }
    bool value = default_value;
    if (!bb->get(key, value))
    {
        return default_value;
    }
    return value;
}

int getInt(const Blackboard::Ptr& bb, const std::string& key, int default_value = 0)
{
    int value = default_value;
    if (!bb->get(key, value))
    {
        return default_value;
    }
    return value;
}

std::string getString(const Blackboard::Ptr& bb, const std::string& key, const std::string& default_value = "")
{
    std::string value = default_value;
    if (!bb->get(key, value))
    {
        return default_value;
    }
    return value;
}

std::string getString(const Blackboard* bb, const std::string& key, const std::string& default_value = "")
{
    if (!bb)
    {
        return default_value;
    }
    std::string value = default_value;
    if (!bb->get(key, value))
    {
        return default_value;
    }
    return value;
}

void batteryCb(const interfaces::Battery::ConstPtr& msg)
{
    if (!g_ctx)
    {
        return;
    }
    g_ctx->battery_soc = msg->soc;
    g_ctx->battery_charging = msg->charging;
    g_ctx->battery_voltage = msg->voltage;
}

void faultCb(const interfaces::Fault::ConstPtr& msg)
{
    if (!g_ctx)
    {
        return;
    }
    g_ctx->fault_type = msg->fault_type;
    g_ctx->fault_severity = msg->severity;
}

void callSignalCb(const std_msgs::Bool::ConstPtr& msg)
{
    if (!g_ctx)
    {
        return;
    }
    g_ctx->call_signal = msg->data;
}

void patrolTriggerCb(const std_msgs::Bool::ConstPtr& msg)
{
    if (!g_ctx)
    {
        return;
    }
    g_ctx->patrol_triggered = msg->data;
}

bool actionOk(const interfaces::ActionStatus& status)
{
    return status.status == interfaces::ActionStatus::OK;
}

class IdleWait : public StatefulActionNode
{
public:
    IdleWait(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus onStart() override { return NodeStatus::RUNNING; }
    NodeStatus onRunning() override { 
        std::cout<<"[IdleWait] RUNNING\n";
        return NodeStatus::RUNNING; 
    }
    void onHalted() override {}
};

class NavgateTo : public StatefulActionNode
{
public:
    NavgateTo(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), ac_("navigate", false)
    {
    }

    static PortsList providedPorts()
    {
        return { InputPort<int>("target"), InputPort<std::string>("nav_type") };
    }

    NodeStatus onStart() override
    {
        auto target = getInput<int>("target").value_or(-1);
        auto nav_type_str = getInput<std::string>("nav_type").value_or("stop");
        int nav_type = navTypeFromString(nav_type_str);
        if (!ac_.waitForServer(ros::Duration(1.0)))
        {
            std::cout << "[ERR ] NavgateTo no server\n";
            return NodeStatus::FAILURE;
        }
        interfaces::NavigateGoal goal;
        goal.target_index = target;
        goal.nav_type = nav_type;
        ac_.sendGoal(goal);
        std::cout << "[START] NavgateTo target=" << target << " type=" << nav_type << "\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (!ac_.getState().isDone())
        {
            std::cout<<"[RUNNING] Navgating\n";
            return NodeStatus::RUNNING;
        }
        auto result = ac_.getResult();
        if (!result)
        {
            std::cout << "[ERR ] NavgateTo no result\n";
            return NodeStatus::FAILURE;
        }
        if (actionOk(result->status))
        {
            std::cout << "[DONE] NavgateTo\n";
            return NodeStatus::SUCCESS;
        }
        std::cout << "[FAIL] NavgateTo status=" << static_cast<int>(result->status.status) << "\n";
        return NodeStatus::FAILURE;
    }

    void onHalted() override
    {
        ac_.cancelGoal();
        std::cout << "[HALT] NavgateTo\n";
    }

private:
    actionlib::SimpleActionClient<interfaces::NavigateAction> ac_;
};

class LLMInteraction : public StatefulActionNode
{
public:
    LLMInteraction(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), ac_("llm_interaction", false)
    {
    }

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("mode"), 
                 InputPort<int>("person_id"), 
                 InputPort<std::string>("context"),
                 OutputPort<bool>("need_call_nurse"),
                 OutputPort<std::string>("summary") };
    }

    NodeStatus onStart() override
    {
        auto mode_str = getInput<std::string>("mode").value_or("passive");
        int mode = interactionModeFromString(mode_str);
        int person_id = getInput<int>("person_id").value_or(-1);
        auto context = getInput<std::string>("context").value_or("none");
        if (!ac_.waitForServer(ros::Duration(1.0)))
        {
            std::cout << "[ERR ] LLMInteraction no server\n";
            return NodeStatus::FAILURE;
        }
        interfaces::LLMInteractionGoal goal;
        goal.mode = mode;
        goal.person_id = person_id;
        goal.context = context;
        ac_.sendGoal(goal);
        std::cout << "[START] LLMInteraction mode=" << mode << " person_id=" << person_id << "\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (!ac_.getState().isDone())
        {
            std::cout<<"[RUNNING] LLMInteraction\n";
            return NodeStatus::RUNNING;
        }
        auto result = ac_.getResult();
        if (!result)
        {
            std::cout << "[ERR ] LLMInteraction no result\n";
            return NodeStatus::FAILURE;
        }
        bool need_call_nurse = result->need_call_nurse;
        auto summary = result->summary;
        setOutput("need_call_nurse", need_call_nurse);
        setOutput("summary",summary);

        std::cout << "[DONE] LLMInteraction need_call_nurse=" << (need_call_nurse ? "true" : "false") << "\n";
        return actionOk(result->status) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    void onHalted() override
    {
        ac_.cancelGoal();
        std::cout << "[HALT] LLMInteraction\n";
    }

private:
    actionlib::SimpleActionClient<interfaces::LLMInteractionAction> ac_;
};

class CallDutyNurse : public StatefulActionNode
{
public:
    CallDutyNurse(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), ac_("call_nurse", false){}

    static PortsList providedPorts() { return { InputPort<int>("bed_id"),
                                                InputPort<std::string>("summary"),
                                                OutputPort<bool>("need_call_nurse") }; }

    NodeStatus onStart() override
    {
        int bed_id = getInput<int>("bed_id").value_or(-1);
        auto summary = getInput<std::string>("summary").value_or("unknow");
        if (!ac_.waitForServer(ros::Duration(1.0)))
        {
            std::cout << "[ERR ] CallDutyNurse no server\n";
            return NodeStatus::FAILURE;
        }
        need_nurse_beds.push_back(bed_id);
        summarys.push_back(summary);
        interfaces::CallNurseGoal goal;
        goal.bed_ids = need_nurse_beds;
        goal.summarys = summarys;
        ac_.sendGoal(goal);
        std::cout << "[START] CallDutyNurse" << "\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (!ac_.getState().isDone())
        {
            std::cout<<"[RUNNING] CalldutyNurse\n";
            return NodeStatus::RUNNING;
        }
        auto result = ac_.getResult();
        if (!result)
        {
            std::cout << "[ERR ] CallDutyNurse no result\n";
            return NodeStatus::FAILURE;
        }
        bool nurse_response = actionOk(result->status);
        setOutput<bool>("need_call_nurse",!nurse_response);
        std::cout << "[DONE] CallDutyNurse ok=" << (nurse_response ? "true" : "false") << "\n";
        return nurse_response ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    void onHalted() override
    {
        ac_.cancelGoal();
        std::cout << "[HALT] CallDutyNurse\n";
    }

private:
    actionlib::SimpleActionClient<interfaces::CallNurseAction> ac_;
    std::vector<int> need_nurse_beds;
    std::vector<std::string> summarys;
};

class SelectNextBed : public SyncActionNode
{
public:
    SelectNextBed(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { InputPort<std::vector<int>>("bed_queue"),
                                                OutputPort<int>("bed_id") }; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        auto queue = getInput<std::vector<int>>("bed_queue").value();

        if (!queue.empty())
        {
            int bed_id = queue.front();
            queue.erase(queue.begin());
            bb->set("bed_queue", queue);
            setOutput("bed_id", bed_id);
            std::cout << "[SET ] SelectNextBed -> bed_id=" << bed_id << " remaining=" << queue.size() << "\n";
            return NodeStatus::SUCCESS;
        }
        setOutput("bed_id",-1);
        std::cout<< "bed queue is empty<<\n";
        return NodeStatus::FAILURE;
    }
};

class FaceIdentify : public SyncActionNode
{
public:
    FaceIdentify(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { OutputPort<int>("person_id"), OutputPort<float>("confidence") }; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        
        // 调用人脸识别服务
        interfaces::FaceIdentify srv;
        
        if (g_ctx->face_identify_client.call(srv))
        {
            // 获取识别结果
            int person_id = srv.response.person_id;
            float confidence = srv.response.confidence;
            
            bb->set("person_id", person_id);
            bb->set("face_confidence", confidence);
            setOutput("person_id", person_id);
            setOutput("confidence", confidence);
            
            if (person_id > 0)
            {
                std::cout << "[ACT ] FaceIdentify 识别成功: person_id=" << person_id << ", confidence=" << confidence << "\n";
            }
            else
            {
                std::cout << "[ACT ] FaceIdentify 未识别到已知人脸\n";
            }
            
            return NodeStatus::SUCCESS;  // 无论识别成功与否都返回 SUCCESS
        }
        else
        {
            std::cerr << "[ACT ] FaceIdentify 服务调用失败\n";
            bb->set("person_id", -1);
            bb->set("face_confidence", 0.0f);
            setOutput("person_id", -1);
            setOutput("confidence", 0.0f);
            return NodeStatus::SUCCESS;  // 失败也返回 SUCCESS，不中断流程
        }
    }
};


class ClearInteractionMode : public SyncActionNode
{
public:
    ClearInteractionMode(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        if (g_root_bb)
        {
            g_root_bb->set("interaction_mode", std::string("none"));
        }
        std::cout << "[ACT ] ClearInteractionMode\n";
        return NodeStatus::SUCCESS;
    }
};

class IsInteractionMode : public ConditionNode
{
public:
    IsInteractionMode(const std::string& name, const NodeConfig& config) : ConditionNode(name, config) {}

    static PortsList providedPorts() { return { InputPort<std::string>("mode") }; }

    NodeStatus tick() override
    {
        auto mode = getInput<std::string>("mode").value_or("none");
        return (mode == "passive" || mode == "alert") ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class LoadPatrolPlan : public SyncActionNode
{
public:
    LoadPatrolPlan(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { InputPort<std::string>("route_id"), InputPort<int>("cycles") }; }

    NodeStatus tick() override
    {
        auto route_id = getInput<std::string>("route_id").value_or("route_default");
        int cycles = getInput<int>("cycles").value_or(1);
        std::cout << "[ACT ] LoadPatrolPlan (noop) route_id=" << route_id << " cycles=" << cycles << "\n";
        return NodeStatus::SUCCESS;
    }
};

class NextPatrolPoint : public SyncActionNode
{
public:
    NextPatrolPoint(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { OutputPort<std::string>("patrol_point") }; }

    NodeStatus tick() override
    {
        if (!g_patrol_ctx)
        {
            return NodeStatus::FAILURE;
        }
        if (g_patrol_ctx->cycles_remaining <= 0)
        {
            return NodeStatus::FAILURE;
        }
        if (g_patrol_ctx->points.empty())
        {
            g_patrol_ctx->complete = true;
            g_patrol_ctx->cycles_remaining = 0;
            return NodeStatus::FAILURE;
        }
        int index = g_patrol_ctx->point_index;
        if (index < 0 || index >= static_cast<int>(g_patrol_ctx->points.size()))
        {
            index = 0;
        }
        std::string point = g_patrol_ctx->points[index];
        index += 1;
        if (index >= static_cast<int>(g_patrol_ctx->points.size()))
        {
            index = 0;
            g_patrol_ctx->cycles_remaining -= 1;
            if (g_patrol_ctx->cycles_remaining <= 0)
            {
                g_patrol_ctx->complete = true;
            }
        }
        g_patrol_ctx->point_index = index;
        setOutput("patrol_point", point);
        std::cout << "[ACT ] NextPatrolPoint -> " << point << "\n";
        return NodeStatus::SUCCESS;
    }
};

class Detect_BedProcess: public SyncActionNode
{
public:
    Detect_BedProcess(const std::string& name, const NodeConfig& config): SyncActionNode(name,config){}
    static PortsList providedPorts (){
        return {InputPort<std::string>("mode"),
                InputPort<int>("patrol_bed_id"),
                OutputPort<std::vector<int>>("bed_queue"),
                OutputPort<std::string>("interaction_mode"),
                OutputPort<std::string>("context")};
    }
    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        auto mode_str = getInput<std::string>("mode").value_or("area");
        int scan_mode = detectModeFromString(mode_str);
        interfaces::DetectAnomaly srv;
        srv.request.area_bed_id=getInput<int>("patrol_bed_id").value_or(-1);
        if (scan_mode == DETECT::BED){
            srv.request.mode=DETECT::BED;
            if (g_ctx && g_ctx->anomaly_client.call(srv)){
                if(srv.response.is_anomaly){
                    setOutput<std::string>("interaction_mode","alert");
                    setOutput<std::string>("context",srv.response.details);
                }
                else{
                    setOutput<std::string>("interaction_mode","passive");
                }
            }
            std::cout << "[SET ] AnomalyDetect bed=" << srv.request.area_bed_id 
                      << " anomaly=" << (srv.response.is_anomaly ? "true" : "false")
                      << " details=" << srv.response.details << "\n";
            return NodeStatus::SUCCESS;
        }
        else if (scan_mode == DETECT::AREA){
            srv.request.mode=DETECT::AREA;
            if (g_ctx && g_ctx->anomaly_client.call(srv)){
                std::vector<int> bed_queue(srv.response.bed_ids.begin(),srv.response.bed_ids.end());
                setOutput<std::vector<int>>("bed_queue",bed_queue);
                std::cout << "[SET ] AnomalyDetect scan beds=" << bed_queue.size() << "\n";
            }
            return NodeStatus::SUCCESS;  
        }
        else{
            std::cout<<"[ERROR] Detect_bedProcess scan_mode can not be recognized\n";
            return NodeStatus::FAILURE;
        }
    }
};

class AlertUnfinishedOnce : public SyncActionNode
{
public:
    AlertUnfinishedOnce(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        bool warned = getBool(bb, "unfinished_warned", false);
        if (!warned)
        {
            bb->set("unfinished_warned", true);
            std::cout << "[ACT ] AlertUnfinishedOnce\n";
        }
        return NodeStatus::SUCCESS;
    }
};

class WaitChargeUntil : public StatefulActionNode
{
public:
    WaitChargeUntil(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}

    static PortsList providedPorts() { return { InputPort<int>("percent") }; }

    NodeStatus onStart() override
    {
        ticks_ = 0;
        target_percent_ = getInput<int>("percent").value_or(50);
        std::cout << "[START] WaitChargeUntil target=" << target_percent_ << "\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        ++ticks_;
        int current = ticks_ * 10;
        std::cout << "[RUN ] Charging " << current << "%\n";
        if (current >= target_percent_)
        {
            std::cout << "[DONE] Charge reached\n";
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    int ticks_ = 0;
    int target_percent_ = 50;
};

class ClearPatrolTrigger : public SyncActionNode
{
public:
    ClearPatrolTrigger(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        if (g_patrol_ctx)
        {
            g_patrol_ctx->complete = false;
            g_patrol_ctx->cycles_remaining = g_patrol_ctx->cycles_total;
            g_patrol_ctx->point_index = 0;
        }
        if (g_ctx)
        {
            g_ctx->patrol_triggered = false;
        }
        std::cout << "[ACT ] ClearPatrolTrigger\n";
        return NodeStatus::SUCCESS;
    }
};

class IsPatrolTriggered : public ConditionNode
{
public:
    IsPatrolTriggered(const std::string& name, const NodeConfig& config) : ConditionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        return (g_ctx && g_ctx->patrol_triggered) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class LambdaAction : public SyncActionNode
{
public:
    using Func = std::function<NodeStatus(LambdaAction&)>;

    LambdaAction(const std::string& name, const NodeConfig& config, Func fn)
        : SyncActionNode(name, config), fn_(std::move(fn))
    {
    }

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("fault_type") };
    }

    NodeStatus tick() override { return fn_(*this); }

    Blackboard::Ptr blackboard() { return config().blackboard; }

private:
    Func fn_;
};

class LambdaCondition : public ConditionNode
{
public:
    using Func = std::function<NodeStatus(LambdaCondition&)>;

    LambdaCondition(const std::string& name, const NodeConfig& config, Func fn)
        : ConditionNode(name, config), fn_(std::move(fn))
    {
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override { return fn_(*this); }

    Blackboard::Ptr blackboard() { return config().blackboard; }

private:
    Func fn_;
};

} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_bt_runner");
    ros::NodeHandle nh("~");

    g_ctx = std::make_shared<RosContext>();
    g_patrol_ctx = std::make_shared<PatrolContext>();
    g_ctx->battery_low_threshold = nh.param("battery_low_threshold", 20.0);
    g_config_id = nh.param<std::string>("config_id", "default");
    g_loadconfig_client = nh.serviceClient<interfaces::SetConfig>("/loadconfig/set_config");
    g_ctx->battery_sub = nh.subscribe("/battery", 1, batteryCb);
    g_ctx->fault_sub = nh.subscribe("/fault", 1, faultCb);
    g_ctx->call_signal_sub = nh.subscribe("/call_signal", 1, callSignalCb);
    g_ctx->patrol_trigger_sub = nh.subscribe("/patrol_triggered", 1, patrolTriggerCb);
    g_ctx->anomaly_client = nh.serviceClient<interfaces::DetectAnomaly>("/detect_anomaly");
    g_ctx->face_identify_client = nh.serviceClient<interfaces::FaceIdentify>("/face_identify");

    BehaviorTreeFactory factory;

    factory.registerBuilder<IdleWait>("IdleWait", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<IdleWait>(name, config);
    });

    factory.registerBuilder<NavgateTo>("NavgateTo", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<NavgateTo>(name, config);
    });

    factory.registerBuilder<LLMInteraction>("LLMInteraction", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LLMInteraction>(name, config);
    });

    factory.registerBuilder<CallDutyNurse>("CallDutyNurse", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<CallDutyNurse>(name, config);
    });

    factory.registerBuilder<SelectNextBed>("SelectNextBed", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<SelectNextBed>(name, config);
    });

    factory.registerBuilder<FaceIdentify>("FaceIdentify", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<FaceIdentify>(name, config);
    });

    factory.registerBuilder<ClearInteractionMode>("ClearInteractionMode", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<ClearInteractionMode>(name, config);
    });

    factory.registerBuilder<IsInteractionMode>("IsInteractionMode", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<IsInteractionMode>(name, config);
    });

    factory.registerBuilder<LoadPatrolPlan>("LoadPatrolPlan", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LoadPatrolPlan>(name, config);
    });

    factory.registerBuilder<NextPatrolPoint>("NextPatrolPoint", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<NextPatrolPoint>(name, config);
    });

    factory.registerBuilder<Detect_BedProcess>("Detect_BedProcess",[](const std::string& name, const NodeConfig& config){
        return std::make_unique<Detect_BedProcess>(name,config);
    });

    factory.registerBuilder<AlertUnfinishedOnce>("AlertUnfinishedOnce", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<AlertUnfinishedOnce>(name, config);
    });

    factory.registerBuilder<WaitChargeUntil>("WaitChargeUntil", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<WaitChargeUntil>(name, config);
    });

    factory.registerBuilder<ClearPatrolTrigger>("ClearPatrolTrigger", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<ClearPatrolTrigger>(name, config);
    });

    factory.registerBuilder<IsPatrolTriggered>("IsPatrolTriggered", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<IsPatrolTriggered>(name, config);
    });

    factory.registerBuilder<LambdaAction>("LoadConfig", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction& node) {
            auto bb = node.blackboard();
            if (!g_config_loaded)
            {
                const ros::Duration wait_interval(0.2);
                const ros::Duration wait_timeout(5.0);
                ros::Time start = ros::Time::now();
                while (ros::ok() && !g_loadconfig_client.exists())
                {
                    if ((ros::Time::now() - start) > wait_timeout)
                    {
                        std::cout << "[WARN] LoadConfig service timeout\n";
                        break;
                    }
                    wait_interval.sleep();
                }
                if (g_loadconfig_client.exists())
                {
                    interfaces::SetConfig srv;
                    srv.request.config_id = g_config_id;
                    if (g_loadconfig_client.call(srv) && srv.response.ok)
                    {
                        g_config_loaded = true;
                    }
                    else
                    {
                        std::cout << "[WARN] LoadConfig service failed\n";
                    }
                }
                else
                {
                    std::cout << "[WARN] LoadConfig service not available\n";
                }
            }

            std::string route_id = "route_a";
            int cycles = 2;
            std::vector<std::string> points = {"p0", "p1"};
            ros::param::get("/loadconfig/patrol_route_id", route_id);
            ros::param::get("/loadconfig/patrol_cycles", cycles);
            if (!readPatrolPoints(points))
            {
                points = {"p0", "p1"};
            }
            if (cycles <= 0)
            {
                cycles = 1;
            }
            if (g_patrol_ctx)
            {
                g_patrol_ctx->route_id = route_id;
                g_patrol_ctx->cycles_total = cycles;
                g_patrol_ctx->cycles_remaining = cycles;
                g_patrol_ctx->point_index = 0;
                g_patrol_ctx->points = points;
                g_patrol_ctx->complete = false;
            }
            bb->set("config_loaded", true);
            bb->set("patrol_started", false);
            bb->set("bed_id", -1);
            bb->set("anomaly", false);
            bb->set("call_nurse", false);
            bb->set("nurse_connected", false);
            bb->set("call_signal", false);
            bb->set("person_id", -1);
            bb->set("unfinished_warned", false);
            if (g_root_bb)
            {
                g_root_bb->set("interaction_mode", std::string("none"));
            }
            std::cout << "[SET ] LoadConfig\n";
            return NodeStatus::SUCCESS;
        });
    });

    factory.registerBuilder<LambdaAction>("ManagePatrolStart", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction& node) {
            auto bb = node.blackboard();
            if (!getBool(bb, "patrol_started", false))
            {
                bb->set("patrol_started", true);
                if (g_patrol_ctx)
                {
                    g_patrol_ctx->complete = false;
                    g_patrol_ctx->cycles_remaining = g_patrol_ctx->cycles_total;
                    g_patrol_ctx->point_index = 0;
                }
                std::cout << "[SET ] ManagePatrolStart (init)\n";
            }
            return NodeStatus::SUCCESS;
        });
    });

    factory.registerBuilder<LambdaAction>("SaveContext", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction&) {
            std::cout << "[ACT ] SaveContext\n";
            return NodeStatus::SUCCESS;
        });
    });

    factory.registerBuilder<LambdaAction>("RestoreContext", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction&) {
            std::cout << "[ACT ] RestoreContext\n";
            return NodeStatus::SUCCESS;
        });
    });

    factory.registerBuilder<LambdaAction>("AlertFault", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction& node) {
            auto fault_type = node.getInput<std::string>("fault_type").value_or("unknown");
            std::cout << "[ACT ] AlertFault type=" << fault_type << "\n";
            return NodeStatus::SUCCESS;
        });
    });

    factory.registerBuilder<LambdaAction>("AlertChargeFault", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction&) {
            std::cout << "[ACT ] AlertChargeFault\n";
            return NodeStatus::SUCCESS;
        });
    });


    factory.registerBuilder<LambdaCondition>("IsAnomaly", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            auto bb = node.blackboard();
            return getBool(bb, "anomaly", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });


    factory.registerBuilder<LambdaCondition>("IsPatrolComplete", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            if (!g_patrol_ctx)
            {
                return NodeStatus::FAILURE;
            }
            return (g_patrol_ctx->cycles_remaining <= 0) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsPatrolIncomplete", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            if (!g_patrol_ctx)
            {
                return NodeStatus::FAILURE;
            }
            return (g_patrol_ctx->cycles_remaining > 0) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsCallSignal", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition&) {
            if (g_ctx && g_ctx->call_signal)
            {
                std::cout << "[COND] IsCallSignal=true\n";
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsBatteryLow", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition&) {
            if (g_ctx && g_ctx->battery_soc <= g_ctx->battery_low_threshold)
            {
                std::cout << "[COND] IsBatteryLow=true\n";
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsLocalizationAbnormal", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition&) {
            if (!g_ctx || g_ctx->fault_severity == 0)
            {
                return NodeStatus::FAILURE;
            }
            return (g_ctx->fault_type == "localization") ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsNavFault", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition&) {
            if (!g_ctx || g_ctx->fault_severity == 0)
            {
                return NodeStatus::FAILURE;
            }
            return (g_ctx->fault_type == "navigation") ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsSelfFault", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition&) {
            if (!g_ctx || g_ctx->fault_severity == 0)
            {
                return NodeStatus::FAILURE;
            }
            return (g_ctx->fault_type == "self") ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    try
    {
        g_root_bb = Blackboard::create();
        auto tree = factory.createTreeFromFile("/home/val/BIH_ws/Medical_Embodied/src/behavior_tree/BH_xml/medical.xml", g_root_bb);

        int max_ticks = nh.param("max_ticks", 400);
        constexpr int kTickHz = 20;
        constexpr auto kTickPeriod = std::chrono::milliseconds(1000 / kTickHz);
        std::cout << "[INFO] tick_rate=" << kTickHz << "Hz period=" << kTickPeriod.count() << "ms\n";

        for (int tick = 0; tick < max_ticks && ros::ok(); ++tick)
        {
            ros::spinOnce();
            std::cout << "\n=== TICK " << tick << " ===\n";
            auto status = tree.tickOnce();
            std::cout << "[TREE] status=" << toStr(status) << "\n";
            std::this_thread::sleep_for(kTickPeriod);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
