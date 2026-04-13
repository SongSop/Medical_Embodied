#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace BT;

namespace BT {
template <>
inline int convertFromString(StringView str)
{
    std::string s(str.data(), str.size());
    if (s == "current")
    {
        return -1;
    }
    if (s == "charge_dock")
    {
        return -2;
    }
    if (!s.empty() && s[0] == 'p')
    {
        bool ok = true;
        for (size_t i = 1; i < s.size(); ++i)
        {
            if (!std::isdigit(static_cast<unsigned char>(s[i])))
            {
                ok = false;
                break;
            }
        }
        if (ok && s.size() > 1)
        {
            return std::stoi(s.substr(1));
        }
    }
    char* end = nullptr;
    long v = std::strtol(s.c_str(), &end, 10);
    if (end && *end == '\0')
    {
        return static_cast<int>(v);
    }
    throw RuntimeError("cannot convert to int: ", s);
}
} // namespace BT

namespace {

Blackboard::Ptr g_root_bb;

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
        return 0;
    }
    if (s == "stop")
    {
        return 1;
    }
    if (s == "dock")
    {
        return 2;
    }
    return 1;
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
        return 0;
    }
    if (s == "bed")
    {
        return 1;
    }
    return 0;
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
    if (s == "alert")
    {
        return 0;
    }
    if (s == "passive")
    {
        return 1;
    }
    if (s == "interrupt")
    {
        return 2;
    }
    return 1;
}

struct Metrics
{
    int nav_start = 0;
    int llm_done = 0;
    int llm_call_response = 0;
    int llm_abnormal = 0;
    int llm_passive = 0;
    int alert_fault_localization = 0;
    int alert_fault_navigation = 0;
    int alert_fault_self = 0;
    int enqueue_nurse = 0;
    int call_duty_nurse = 0;
    int wait_nurse_connected_done = 0;
    int patrol_point = 0;
    int anomaly_scan = 0;
    int anomaly_bed = 0;
    int wait_charge_start = 0;
};

Metrics g_metrics;

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

class IdleWait : public StatefulActionNode
{
public:
    IdleWait(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus onStart() override { return NodeStatus::RUNNING; }
    NodeStatus onRunning() override { return NodeStatus::RUNNING; }
    void onHalted() override {}
};

class NavgateTo : public StatefulActionNode
{
public:
    NavgateTo(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { InputPort<int>("target"), InputPort<std::string>("nav_type") };
    }

    NodeStatus onStart() override
    {
        ticks_ = 0;
        auto target = getInput<int>("target").value_or(-1);
        auto nav_type_str = getInput<std::string>("nav_type").value_or("stop");
        int nav_type = navTypeFromString(nav_type_str);
        g_metrics.nav_start += 1;
        std::cout << "[START] NavgateTo target=" << target << " type=" << nav_type << "\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        ++ticks_;
        std::cout << "[RUN ] NavgateTo step " << ticks_ << "/" << ticks_required_ << "\n";
        if (ticks_ >= ticks_required_)
        {
            std::cout << "[DONE] NavgateTo\n";
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[HALT] NavgateTo\n";
        ticks_ = 0;
    }

private:
    int ticks_required_ = 3;
    int ticks_ = 0;
};

class LLMInteraction : public StatefulActionNode
{
public:
    LLMInteraction(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}

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
        ticks_ = 0;
        auto mode_str = getInput<std::string>("mode").value_or("passive");
        mode_ = interactionModeFromString(mode_str);
        person_id_ = getInput<int>("person_id").value_or(-1);
        context_ = getInput<std::string>("context").value_or("");
        setOutput("need_call_nurse", false);
        setOutput("summary", std::string("pending"));
        std::cout << "[START] LLMInteraction mode=" << mode_ << " person_id=" << person_id_ << "\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        ++ticks_;
        std::cout << "[RUN ] LLMInteraction step " << ticks_ << "/" << ticks_required_ << "\n";
        if (ticks_ >= ticks_required_)
        {
            bool need_call_nurse = (mode_ == 0);
            setOutput("need_call_nurse", need_call_nurse);
            setOutput("summary", context_.empty() ? std::string("mock_summary") : context_);
            g_metrics.llm_done += 1;
            if (mode_ == 2)
            {
                g_metrics.llm_call_response += 1;
            }
            else if (mode_ == 0)
            {
                g_metrics.llm_abnormal += 1;
            }
            else if (mode_ == 1)
            {
                g_metrics.llm_passive += 1;
            }
            std::cout << "[DONE] LLMInteraction need_call_nurse=" << (need_call_nurse ? "true" : "false") << "\n";
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[HALT] LLMInteraction\n";
        ticks_ = 0;
    }

private:
    int ticks_required_ = 3;
    int ticks_ = 0;
    int mode_ = 1;
    std::string context_;
    int person_id_ = -1;
};

class SelectNextBed : public SyncActionNode
{
public:
    SelectNextBed(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { InputPort<std::vector<int>>("bed_queue"),
                 OutputPort<int>("bed_id") };
    }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        std::vector<int> queue = getInput<std::vector<int>>("bed_queue").value();
        if (!queue.empty())
        {
            int bed_id = queue.front();
            queue.erase(queue.begin());
            bb->set("bed_queue", queue);
            bb->set("beds_remaining", static_cast<int>(queue.size()));
            bb->set("bed_id", bed_id);
            setOutput("bed_id", bed_id);
            std::cout << "[SET ] SelectNextBed -> bed_id=" << bed_id << " remaining=" << queue.size() << "\n";
            return NodeStatus::SUCCESS;
        }
        bb->set("bed_id", -1);
        setOutput("bed_id", -1);
        return NodeStatus::FAILURE;
    }
};

class FaceIdentify : public SyncActionNode
{
public:
    FaceIdentify(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { OutputPort<int>("person_id") }; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        int next_id = getInt(bb, "person_id", 0) + 1;
        bb->set("person_id", next_id);
        setOutput("person_id", next_id);
        std::cout << "[ACT ] FaceIdentify person_id=" << next_id << "\n";
        return NodeStatus::SUCCESS;
    }
};

class EnqueueNurseCall : public SyncActionNode
{
public:
    EnqueueNurseCall(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { InputPort<std::string>("bed_id") }; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        std::string bed_id = getInput<std::string>("bed_id").value_or(getString(bb, "bed_id", "unknown"));
        if (g_root_bb)
        {
            g_root_bb->set("nurse_call_pending", true);
            g_root_bb->set("nurse_call_bed_id", bed_id);
        }
        g_metrics.enqueue_nurse += 1;
        std::cout << "[ACT ] EnqueueNurseCall bed_id=" << bed_id << "\n";
        return NodeStatus::SUCCESS;
    }
};

class PopNurseCall : public SyncActionNode
{
public:
    PopNurseCall(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { OutputPort<std::string>("bed_id") }; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        std::string bed_id = getString(g_root_bb.get(), "nurse_call_bed_id", "unknown");
        if (g_root_bb)
        {
            g_root_bb->set("nurse_call_pending", false);
        }
        bb->set("nurse_bed_id", bed_id);
        setOutput("bed_id", bed_id);
        std::cout << "[ACT ] PopNurseCall bed_id=" << bed_id << "\n";
        return NodeStatus::SUCCESS;
    }
};

class CallDutyNurse : public SyncActionNode
{
public:
    CallDutyNurse(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { InputPort<int>("bed_id"),
                 InputPort<std::string>("summary"),
                 OutputPort<bool>("need_call_nurse") };
    }

    NodeStatus tick() override
    {
        int bed_id = getInput<int>("bed_id").value_or(-1);
        auto summary = getInput<std::string>("summary").value_or("unknown");
        g_metrics.call_duty_nurse += 1;
        setOutput("need_call_nurse", false);
        std::cout << "[ACT ] CallDutyNurse bed_id=" << bed_id << " summary=" << summary << "\n";
        return NodeStatus::SUCCESS;
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

class HasNextBed : public ConditionNode
{
public:
    HasNextBed(const std::string& name, const NodeConfig& config) : ConditionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        std::vector<int> queue;
        if (bb->get("bed_queue", queue) && !queue.empty())
        {
            return NodeStatus::SUCCESS;
        }
        int remaining = getInt(bb, "beds_remaining", 0);
        return remaining > 0 ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class LoadPatrolPlan : public SyncActionNode
{
public:
    LoadPatrolPlan(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return { InputPort<std::string>("route_id"), InputPort<int>("cycles") }; }

    NodeStatus tick() override
    {
        auto bb = config().blackboard;
        auto route_id = getInput<std::string>("route_id").value_or("route_default");
        int cycles = getInput<int>("cycles").value_or(1);
        bb->set("patrol_route_id", route_id);
        bb->set("patrol_cycles", cycles);
        bb->set("patrol_remaining", cycles);
        bb->set("patrol_complete", false);
        std::cout << "[ACT ] LoadPatrolPlan route_id=" << route_id << " cycles=" << cycles << "\n";
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
        auto bb = config().blackboard;
        int remaining = getInt(bb, "patrol_remaining", 0);
        if (remaining <= 0)
        {
            return NodeStatus::FAILURE;
        }
        remaining -= 1;
        bb->set("patrol_remaining", remaining);
        if (remaining == 0)
        {
            bb->set("patrol_complete", true);
        }
        int index = getInt(bb, "patrol_index", 0);
        std::string point = "p" + std::to_string(index % 2);
        bb->set("patrol_point", point);
        bb->set("patrol_index", index + 1);
        setOutput("patrol_point", point);
        g_metrics.patrol_point += 1;
        std::cout << "[ACT ] NextPatrolPoint -> " << point << "\n";
        return NodeStatus::SUCCESS;
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

class Detect_BedProcess : public SyncActionNode
{
public:
    Detect_BedProcess(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("mode"),
                 InputPort<int>("bed_id"),
                 OutputPort<std::vector<int>>("bed_queue"),
                 OutputPort<std::string>("interaction_mode"),
                 OutputPort<std::string>("context") };
    }

    NodeStatus tick() override
    {
        auto mode = getInput<std::string>("mode").value_or("area");
        int scan_mode = detectModeFromString(mode);
        int bed_id = getInput<int>("bed_id").value_or(-1);

        if (scan_mode == 1)
        {
            bool anomaly = (bed_id >= 0) ? (bed_id % 2 == 0) : false;
            setOutput<std::string>("interaction_mode", anomaly ? "alert" : "passive");
            setOutput<std::string>("context", anomaly ? "anomaly" : "normal");
            g_metrics.anomaly_bed += 1;
            std::cout << "[SET ] AnomalyDetect bed=" << bed_id << " anomaly=" << (anomaly ? "true" : "false") << "\n";
            return NodeStatus::SUCCESS;
        }

        std::vector<int> queue = {1, 0};
        setOutput("bed_queue", queue);
        g_metrics.anomaly_scan += 1;
        std::cout << "[SET ] AnomalyDetect scan beds=2\n";
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
        g_metrics.wait_charge_start += 1;
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
        if (g_root_bb)
        {
            g_root_bb->set("patrol_triggered", false);
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
        return getBool(g_root_bb.get(), "patrol_triggered", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class WaitNurseConnected : public StatefulActionNode
{
public:
    WaitNurseConnected(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}
    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override
    {
        std::cout << "[START] WaitNurseConnected\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        auto bb = config().blackboard;
        if (getBool(bb, "nurse_connected", false))
        {
            g_metrics.wait_nurse_connected_done += 1;
            std::cout << "[DONE] Nurse connected\n";
            return NodeStatus::SUCCESS;
        }
        std::cout << "[RUN ] Waiting for nurse connection\n";
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}
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

int main()
{
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

    factory.registerBuilder<SelectNextBed>("SelectNextBed", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<SelectNextBed>(name, config);
    });

    factory.registerBuilder<FaceIdentify>("FaceIdentify", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<FaceIdentify>(name, config);
    });

    factory.registerBuilder<EnqueueNurseCall>("EnqueueNurseCall", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<EnqueueNurseCall>(name, config);
    });

    factory.registerBuilder<PopNurseCall>("PopNurseCall", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<PopNurseCall>(name, config);
    });

    factory.registerBuilder<CallDutyNurse>("CallDutyNurse", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<CallDutyNurse>(name, config);
    });

    factory.registerBuilder<ClearInteractionMode>("ClearInteractionMode", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<ClearInteractionMode>(name, config);
    });

    factory.registerBuilder<IsInteractionMode>("IsInteractionMode", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<IsInteractionMode>(name, config);
    });

    factory.registerBuilder<HasNextBed>("HasNextBed", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<HasNextBed>(name, config);
    });

    factory.registerBuilder<LoadPatrolPlan>("LoadPatrolPlan", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LoadPatrolPlan>(name, config);
    });

    factory.registerBuilder<NextPatrolPoint>("NextPatrolPoint", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<NextPatrolPoint>(name, config);
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

    factory.registerBuilder<WaitNurseConnected>("WaitNurseConnected", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<WaitNurseConnected>(name, config);
    });

    factory.registerBuilder<LambdaAction>("LoadConfig", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction& node) {
            auto bb = node.blackboard();
            bb->set("config_loaded", true);
            bb->set("patrol_started", false);
            bb->set("patrol_complete", false);
            bb->set("beds_remaining", 0);
            bb->set("bed_index", -1);
            bb->set("bed_id", -1);
            bb->set("anomaly", false);
            bb->set("call_nurse", false);
            bb->set("nurse_connected", false);
            bb->set("call_signal", false);
            bb->set("battery_low", false);
            bb->set("localization_abnormal", false);
            bb->set("nav_fault", false);
            bb->set("self_fault", false);
            bb->set("person_id", -1);
            bb->set("patrol_route_id", std::string("route_a"));
            bb->set("patrol_cycles", 2);
            bb->set("patrol_remaining", 0);
            bb->set("patrol_index", 0);
            bb->set("unfinished_warned", false);
            if (g_root_bb)
            {
                g_root_bb->set("nurse_call_pending", false);
                g_root_bb->set("nurse_call_bed_id", std::string("unknown"));
                g_root_bb->set("interaction_mode", std::string("none"));
                g_root_bb->set("patrol_triggered", false);
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
                bb->set("patrol_complete", false);
                bb->set("patrol_points_remaining", 1);
                bb->set("patrol_point", std::string("p1"));
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

    factory.registerBuilder<LambdaAction>("RequestAnomalyDetection", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction& node) {
            auto bb = node.blackboard();
            bb->set("beds_remaining", 2);
            bb->set("bed_index", -1);
            std::cout << "[SET ] RequestAnomalyDetection (beds=2)\n";
            return NodeStatus::SUCCESS;
        });
    });

    factory.registerBuilder<Detect_BedProcess>("Detect_BedProcess", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<Detect_BedProcess>(name, config);
    });

    factory.registerBuilder<LambdaAction>("AlertAbnormal", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction&) {
            std::cout << "[ACT ] AlertAbnormal\n";
            return NodeStatus::SUCCESS;
        });
    });

    factory.registerBuilder<LambdaAction>("AlertFault", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction& node) {
            auto fault_type = node.getInput<std::string>("fault_type").value_or("unknown");
            if (fault_type == "localization")
            {
                g_metrics.alert_fault_localization += 1;
            }
            else if (fault_type == "navigation")
            {
                g_metrics.alert_fault_navigation += 1;
            }
            else if (fault_type == "self")
            {
                g_metrics.alert_fault_self += 1;
            }
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

    factory.registerBuilder<LambdaAction>("WaitAtChargeDock", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaAction>(name, config, [](LambdaAction&) {
            std::cout << "[ACT ] WaitAtChargeDock\n";
            return NodeStatus::SUCCESS;
        });
    });


    factory.registerBuilder<LambdaCondition>("IsAnomaly", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            auto bb = node.blackboard();
            return getBool(bb, "anomaly", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("ShouldCallNurse", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            auto bb = node.blackboard();
            return getBool(bb, "call_nurse", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("HasNurseCall", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            auto bb = node.blackboard();
            if (getBool(g_root_bb.get(), "nurse_call_pending", false))
            {
                std::cout << "[COND] HasNurseCall=true\n";
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsPatrolComplete", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            auto bb = node.blackboard();
            return getBool(bb, "patrol_complete", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsPatrolIncomplete", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            auto bb = node.blackboard();
            return !getBool(bb, "patrol_complete", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsCallSignal", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            if (getBool(g_root_bb.get(), "call_signal", false))
            {
                std::cout << "[COND] IsCallSignal=true\n";
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsBatteryLow", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            if (getBool(g_root_bb.get(), "battery_low", false))
            {
                std::cout << "[COND] IsBatteryLow=true\n";
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsLocalizationAbnormal", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            return getBool(g_root_bb.get(), "localization_abnormal", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsNavFault", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            return getBool(g_root_bb.get(), "nav_fault", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    factory.registerBuilder<LambdaCondition>("IsSelfFault", [](const std::string& name, const NodeConfig& config) {
        return std::make_unique<LambdaCondition>(name, config, [](LambdaCondition& node) {
            return getBool(g_root_bb.get(), "self_fault", false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });
    });

    try
    {
        constexpr int kTickHz = 20;
        constexpr auto kTickPeriod = std::chrono::milliseconds(1000 / kTickHz);
        std::cout << "[INFO] tick_rate=" << kTickHz << "Hz period=" << kTickPeriod.count() << "ms\n";

        g_root_bb = Blackboard::create();
        auto tree = factory.createTreeFromFile("/home/val/BIH_ws/Medical_Embodied/src/behavior_tree/BH_xml/medical.xml", g_root_bb);
        std::unique_ptr<Groot2Publisher> publisher;
        try
        {
            publisher = std::make_unique<Groot2Publisher>(tree, 1667);
        }
        catch (const std::exception& e)
        {
            std::cout << "[WARN] Groot2Publisher disabled: " << e.what() << "\n";
        }
        auto bb = tree.rootBlackboard();

        for (int tick = 0; tick < 140; ++tick)
        {
            if (tick == 0)
            {
                std::cout << "\n[TEST] Section: baseline idle.\n";
                std::cout << "[TEST] 目标: 无触发，树保持空闲。\n";
                std::cout << "[TEST] 结果: (区段结束后给出)\n";
                int base = g_metrics.llm_done + g_metrics.alert_fault_localization + g_metrics.alert_fault_navigation +
                           g_metrics.alert_fault_self + g_metrics.patrol_point;
                bb->set("section_base_m", base);
            }
            if (tick == 4)
            {
                std::cout << "\n[TEST] Section end: baseline idle.\n";
                int start = getInt(bb, "section_base_m", 0);
                int current = g_metrics.llm_done + g_metrics.alert_fault_localization + g_metrics.alert_fault_navigation +
                              g_metrics.alert_fault_self + g_metrics.patrol_point;
                int delta = current - start;
                std::cout << "[TEST] 结果: " << (delta == 0 ? "PASS" : "FAIL")
                          << " (unexpected_events=" << delta << ")\n";
            }
            bb->set("call_signal", false);
            bb->set("battery_low", false);
            bb->set("localization_abnormal", false);
            bb->set("nav_fault", false);
            bb->set("self_fault", false);

            if (tick >= 5 && tick <= 8)
            {
                bb->set("call_signal", true);
                if (tick == 5)
                {
                    std::cout << "\n[TEST] Section: call signal response (ticks 5-8).\n";
                    std::cout << "[TEST] 目标: 触发呼叫响应，LLMInteraction 模式为 interrupt。\n";
                    std::cout << "[TEST] 结果: (区段结束后给出)\n";
                    bb->set("section_call_signal_m", g_metrics.llm_call_response);
                }
                std::cout << "[SIM ] call_signal=true\n";
            }
            if (tick == 12)
            {
                std::cout << "\n[TEST] Section end: call signal response.\n";
                int start = getInt(bb, "section_call_signal_m", 0);
                int delta = g_metrics.llm_call_response - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (llm_call_response+=" << delta << ")\n";
            }
            if (tick >= 15 && tick <= 18)
            {
                bb->set("localization_abnormal", true);
                if (g_root_bb)
                {
                    g_root_bb->set("patrol_triggered", true);
                }
                if (tick == 15)
                {
                    std::cout << "\n[TEST] Section: localization fault handling (ticks 15-18).\n";
                    std::cout << "[TEST] 目标: 触发定位故障处理并报警。\n";
                    std::cout << "[TEST] 结果: (区段结束后给出)\n";
                    bb->set("section_loc_m", g_metrics.alert_fault_localization);
                }
                std::cout << "[SIM ] localization_abnormal=true\n";
            }
            if (tick == 20)
            {
                std::cout << "\n[TEST] Section end: localization fault handling.\n";
                int start = getInt(bb, "section_loc_m", 0);
                int delta = g_metrics.alert_fault_localization - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (alert_fault_localization+=" << delta << ")\n";
                if (g_root_bb)
                {
                    g_root_bb->set("patrol_triggered", false);
                }
            }
            if (tick >= 25 && tick <= 28)
            {
                bb->set("nav_fault", true);
                if (g_root_bb)
                {
                    g_root_bb->set("patrol_triggered", true);
                }
                if (tick == 25)
                {
                    std::cout << "\n[TEST] Section: navigation fault handling (ticks 25-28).\n";
                    std::cout << "[TEST] 目标: 触发导航故障处理并报警。\n";
                    std::cout << "[TEST] 结果: (区段结束后给出)\n";
                    bb->set("section_nav_m", g_metrics.alert_fault_navigation);
                }
                std::cout << "[SIM ] nav_fault=true\n";
            }
            if (tick == 31)
            {
                std::cout << "\n[TEST] Section end: navigation fault handling.\n";
                int start = getInt(bb, "section_nav_m", 0);
                int delta = g_metrics.alert_fault_navigation - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (alert_fault_navigation+=" << delta << ")\n";
                if (g_root_bb)
                {
                    g_root_bb->set("patrol_triggered", false);
                }
            }
            if (tick >= 35 && tick <= 38)
            {
                bb->set("self_fault", true);
                if (g_root_bb)
                {
                    g_root_bb->set("patrol_triggered", true);
                }
                if (tick == 35)
                {
                    std::cout << "\n[TEST] Section: self fault handling (ticks 35-38).\n";
                    std::cout << "[TEST] 目标: 触发自检故障处理并报警。\n";
                    std::cout << "[TEST] 结果: (区段结束后给出)\n";
                    bb->set("section_self_m", g_metrics.alert_fault_self);
                }
                std::cout << "[SIM ] self_fault=true\n";
            }
            if (tick == 41)
            {
                std::cout << "\n[TEST] Section end: self fault handling.\n";
                int start = getInt(bb, "section_self_m", 0);
                int delta = g_metrics.alert_fault_self - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (alert_fault_self+=" << delta << ")\n";
                if (g_root_bb)
                {
                    g_root_bb->set("patrol_triggered", false);
                }
            }
            if (tick >= 68 && tick <= 74)
            {
                bb->set("battery_low", true);
                if (tick == 68)
                {
                    std::cout << "\n[TEST] Section: low battery handling (ticks 68-74).\n";
                    std::cout << "[TEST] 目标: 触发低电量流程并进入充电等待。\n";
                    std::cout << "[TEST] 结果: (区段结束后给出)\n";
                    bb->set("section_batt_m", g_metrics.wait_charge_start);
                }
                std::cout << "[SIM ] battery_low=true (near patrol completion)\n";
            }
            if (tick == 75)
            {
                std::cout << "\n[TEST] Section end: low battery handling.\n";
                int start = getInt(bb, "section_batt_m", 0);
                int delta = g_metrics.wait_charge_start - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (wait_charge_start+=" << delta << ")\n";
            }
            if (tick == 45 && g_root_bb)
            {
                g_root_bb->set("patrol_triggered", true);
                bb->set("patrol_cycles", 2);
                std::cout << "\n[TEST] Section: patrol run (cycles=2).\n";
                std::cout << "[TEST] 目标: 多次巡检并触发异常检测与交互。\n";
                std::cout << "[TEST] 结果: (区段结束后给出)\n";
                bb->set("section_patrol_m", g_metrics.patrol_point);
                std::cout << "[SIM ] patrol_triggered=true (cycles=2)\n";
                std::cout << "\n[TEST] Section: nurse call handling (during patrol).\n";
                std::cout << "[TEST] 目标: 异常告警触发后产生呼叫护士。\n";
                std::cout << "[TEST] 结果: (区段结束后给出)\n";
                bb->set("section_nurse_m", g_metrics.call_duty_nurse);
            }
            if (tick == 61)
            {
                std::cout << "\n[TEST] Section end: patrol run (cycles=2).\n";
                int start = getInt(bb, "section_patrol_m", 0);
                int delta = g_metrics.patrol_point - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (patrol_point+=" << delta << ")\n";
                std::cout << "\n[TEST] Section end: nurse call handling.\n";
                int nstart = getInt(bb, "section_nurse_m", 0);
                int ndelta = g_metrics.call_duty_nurse - nstart;
                std::cout << "[TEST] 结果: " << (ndelta >= 1 ? "PASS" : "FAIL")
                          << " (call_duty_nurse+=" << ndelta << ")\n";
            }
            if (tick == 90 && g_root_bb)
            {
                g_root_bb->set("patrol_triggered", true);
                bb->set("patrol_cycles", 1);
                std::cout << "\n[TEST] Section: patrol run (cycles=1).\n";
                std::cout << "[TEST] 目标: 单次巡检流程。\n";
                std::cout << "[TEST] 结果: (区段结束后给出)\n";
                bb->set("section_patrol1_m", g_metrics.patrol_point);
                std::cout << "[SIM ] patrol_triggered=true (cycles=1)\n";
            }
            if (tick == 110)
            {
                std::cout << "\n[TEST] Section end: patrol run (cycles=1).\n";
                int start = getInt(bb, "section_patrol1_m", 0);
                int delta = g_metrics.patrol_point - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (patrol_point+=" << delta << ")\n";
            }
            if (tick >= 95 && tick <= 98)
            {
                bb->set("call_signal", true);
                if (tick == 95)
                {
                    std::cout << "\n[TEST] Section: call signal late in run (ticks 95-98).\n";
                    std::cout << "[TEST] 目标: 后期呼叫响应。\n";
                    std::cout << "[TEST] 结果: (区段结束后给出)\n";
                    bb->set("section_call2_m", g_metrics.llm_call_response);
                }
                std::cout << "[SIM ] call_signal=true\n";
            }
            if (tick == 106)
            {
                std::cout << "\n[TEST] Section end: call signal late in run.\n";
                int start = getInt(bb, "section_call2_m", 0);
                int delta = g_metrics.llm_call_response - start;
                std::cout << "[TEST] 结果: " << (delta >= 1 ? "PASS" : "FAIL")
                          << " (llm_call_response+=" << delta << ")\n";
            }

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
