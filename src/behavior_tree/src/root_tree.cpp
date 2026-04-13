#include "behaviortree_cpp/bt_factory.h"

using namespace std;
using namespace BT;
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name, const BT::NodeConfig& config) :
      BT::SyncActionNode(name, config)
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class SaySomething : public SyncActionNode
{
public:
  // If your Node has ports, you must use this constructor signature 
  SaySomething(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config)
  { }

  // It is mandatory to define this STATIC method.
  static PortsList providedPorts()
  {
    // This action has a single input port called "message"
    return { InputPort<std::string>("message") };
  }

  // Override the virtual function tick()
  NodeStatus tick() override
  {
    Expected<std::string> msg = getInput<std::string>("message");
    // Check if expected is valid. If not, throw its error
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error() );
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
  }
};
class ThinkWhatToSay : public SyncActionNode
{
public:
  ThinkWhatToSay(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config)
  { }

  static PortsList providedPorts()
  {
    return { OutputPort<std::string>("text2") };
  }

  // This Action writes a value into the port "text"
  NodeStatus tick() override
  {
    // the output may change at each tick(). Here we keep it simple.
    // setOutput("text", "The answer is 42" );
    setOutput("text2", "The answer is 46" );
    return NodeStatus::SUCCESS;
  }
};



BT::NodeStatus CheckBattery(){
    cout<<"[Battery: OK]" <<endl;
    return BT::NodeStatus::SUCCESS;
}
class GripperInterface
{
public:
    GripperInterface(): _open(true){}
    BT::NodeStatus open(){
        _open=true;
        cout<<"GripperInterface::open"<<endl;
        return BT::NodeStatus::SUCCESS;
    }
    BT::NodeStatus close(){
        _open=false;
        cout<<"GripperInterface::close"<<endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _open;
};


int main(){
    // BT::BehaviorTreeFactory factory;
    // factory.registerNodeType<ApproachObject>("ApproachObject");
    // factory.registerSimpleCondition("CheckBattery", [&](BT::TreeNode&){return CheckBattery();});
    // GripperInterface gripper;
    // factory.registerSimpleAction("OpenGripper", [&](BT::TreeNode&){return gripper.open();});
    // // factory.registerSimpleAction("OpenGripper",[&](BT::TreeNode&){ 
    // //     cout<<"using labmead gripper open\n";
    // //     return BT::NodeStatus::SUCCESS;});
    // factory.registerSimpleAction("CloseGripper",[&](BT::TreeNode&){return gripper.close();});
    
    BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");


    auto tree = factory.createTreeFromFile("/home/val/BIH_ws/Medical_Embodied/src/behavior_tree/BH_xml/my_tree.xml");
    tree.tickWhileRunning();
    return 0;
}