#include"behaviortree_cpp/bt_factory.h"
using namespace std;
using namespace BT;


class IsEmergency: public ConditionNode
{
public:
    IsEmergency(const string& name, const NodeConfig& config):
    ConditionNode(name,config) {}

    static PortsList providedPorts()
    {
        return {InputPort<int>("flag")};
    }
    NodeStatus tick() override
    {
        int flag;
        getInput("flag",flag);
        cout<<"IsEmergency get flag="<<flag<<endl;
        if(flag==0){
            return NodeStatus::SUCCESS;
        }
        else{
            return NodeStatus::FAILURE;
        }
    }
};
class NeedCharge: public ConditionNode
{
public:
    NeedCharge(const string& name, const NodeConfig& config):
    ConditionNode(name,config) {}
    static PortsList providedPorts()
    {
        return {InputPort<double>("state")};
    }
    NodeStatus tick() override
    {
        double state;
        getInput("state",state);
        cout<<"battery state="<<state<<endl;
        if(state>=48.0){
            return NodeStatus::SUCCESS;
        }
        else{
            return NodeStatus::FAILURE;
        }
    }
};
class IsCalled: public ConditionNode
{
public:
    IsCalled(const string& name, const NodeConfig& config):
    ConditionNode(name,config) {}
    static PortsList providedPorts()
    {
        return {InputPort<string>("words")};
    }
    NodeStatus tick() override
    {
        string words;
        getInput("words",words);
        cout<<"called words="<<words<<endl;
        if(words.size()<5){
            return NodeStatus::FAILURE;
        }
        else{
            return NodeStatus::SUCCESS;
        }
    }
};

class PerformTask: public StatefulActionNode
{
public:
    PerformTask(const string& name, const NodeConfig& config)
    : StatefulActionNode(name, config), _count(0) {}

    static PortsList providedPorts()
    {
        return {};
    }

    NodeStatus onStart() override
    {
        _count = 0;
        cout << "PerformTask start\n";
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        cout << "PerformTask running step " << _count << "\n";
        if (_count < 100) {
            _count++;
            return NodeStatus::RUNNING;
        }
        return NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        cout << "PerformTask halted at step " << _count << "\n";
        _count = 0;
    }

private:
    int _count;
};

int main(){
    BehaviorTreeFactory factory;
    factory.registerNodeType<IsEmergency>("IsEmergency");
    factory.registerNodeType<NeedCharge>("NeedCharge");
    factory.registerNodeType<IsCalled>("IsCalled");
    factory.registerNodeType<PerformTask>("PerformTask");

    factory.registerSimpleAction("IdleWait",[](TreeNode&){
        cout<<"perform IdelWait\n";
        return NodeStatus::SUCCESS;
    });
    auto tree = factory.createTreeFromFile("/home/val/BIH_ws/Medical_Embodied/src/behavior_tree/BH_xml/ReactiveFallback.xml");
    auto blackboard = tree.rootBlackboard();
    
    for(int i=0;i<100;i++){
        if(i<=10){
            blackboard->set("emergency_flag",10);
            blackboard->set("battery_state",46.0);
            blackboard->set("called","cal");
        }
        else{
            blackboard->set("called","call robot name with long words\n");
        }
        NodeStatus status = tree.tickOnce();
        // if (status == NodeStatus::SUCCESS||status == NodeStatus::FAILURE){
        //     break;
        // }
        this_thread::sleep_for(chrono::microseconds(10000));
    }
    return 0;
}
