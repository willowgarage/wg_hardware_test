#include <ros/node_handle.h>
#include <std_msgs/Empty.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace digital_trigger_controller{

class DigitalTriggerController: public pr2_controller_interface::Controller
{
private:
  pr2_hardware_interface::DigitalOut* digital_out_;
  bool trigger_;
  ros::Subscriber subscriber_;
  ros::NodeHandle node_;

public:
  DigitalTriggerController() : digital_out_(NULL), trigger_(false) { }
  void triggerCB(std_msgs::EmptyConstPtr const &e);
  //const boost::shared_ptr<const std_msgs::Empty> &e);

  bool init(pr2_mechanism_model::RobotState *robot,
           ros::NodeHandle &node);
  //void starting();
  void update();
  //void stopping();
};

void DigitalTriggerController::triggerCB(std_msgs::EmptyConstPtr const &e) 
{ 
  trigger_=true;
}

bool DigitalTriggerController::init(pr2_mechanism_model::RobotState *robot,ros::NodeHandle &node)
{  
  //ROS_ERROR("DigitalTriggerController: INIT");

  std::string name;
  if (!node.getParam("digital_out_name", name)){
    ROS_ERROR("DigitalTriggerController: 'digital_out_name' not found on parameter namespace: %s)",
              node.getNamespace().c_str());
    return false;
  }

  node_ = node;
  subscriber_ = node.subscribe("trigger", 1u, &DigitalTriggerController::triggerCB, this);

  digital_out_ = robot->model_->hw_->getDigitalOut(name);
  if (digital_out_==NULL) {
    ROS_ERROR("DigitalTriggerController: Could not find digital out named : %s)",
              name.c_str());
    BOOST_FOREACH(const pr2_hardware_interface::DigitalOutMap::value_type &v, robot->model_->hw_->digital_outs_)
    {
      ROS_INFO("digital out : %s", v.first.c_str());
    }
    return false;
  }

  return true;
}

void DigitalTriggerController::update()
{
  if (!digital_out_)
    return;

  if (trigger_) {
    trigger_ = false;
    digital_out_->command_.data_ = 1;
  } else {
    digital_out_->command_.data_ = 0;
  }
}

} // end namespace digital_trigger_controller_ns



/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(DigitalTriggerController, 
                         digital_trigger_controller::DigitalTriggerController, 
                         pr2_controller_interface::Controller)
