#include <ros/node_handle.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>

#include <mtrace_tools/MotorResistanceSample.h>
#include <mtrace_tools/MotorResistance.h>

namespace motor_resistance_recorder {

class MotorResistanceRecorder : public pr2_controller_interface::Controller
{
private:
  pr2_hardware_interface::Actuator *actuator_;
  //pr2_mechanism_model::RobotState *robot_;
  ros::NodeHandle node_;
  realtime_tools::RealtimePublisher<mtrace_tools::MotorResistance> *publisher_;
  unsigned buffer_size_;
  std::vector<mtrace_tools::MotorResistanceSample> buffer_;
  unsigned buffer_index_;
  double last_velocity_;
  double last_position_;
  bool positive_direction_;

public:
  MotorResistanceRecorder();
  ~MotorResistanceRecorder();
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node);
  void starting();
  void update();
  //void stopping();
};

MotorResistanceRecorder::MotorResistanceRecorder() : 
  actuator_(NULL), 
  publisher_(NULL),
  buffer_size_(100),
  buffer_(buffer_size_),
  buffer_index_(0),
  last_velocity_(0),
  last_position_(0),
  positive_direction_(false)
{
  
}

MotorResistanceRecorder::~MotorResistanceRecorder()
{
  delete publisher_;
}

bool MotorResistanceRecorder::init(pr2_mechanism_model::RobotState *robot,ros::NodeHandle &node)
{  
  //ROS_ERROR("MotorResistanceRecorder: INIT");
  node_ = node;

  std::string name;
  if (!node.getParam("actuator_name", name)){
    ROS_ERROR("MotorResistanceRecorder: 'actuator' not found on parameter namespace: %s)",
              node.getNamespace().c_str());
    return false;
  }

  pr2_hardware_interface::HardwareInterface* hw = robot->model_->hw_;
  actuator_ = hw->getActuator(name);
  if (actuator_ == NULL) {
    ROS_ERROR("MotorResistanceRecorder: Could not find actuator named : %s)", name.c_str());
    BOOST_FOREACH(const pr2_hardware_interface::ActuatorMap::value_type &v, hw->actuators_)
    {
      ROS_INFO("actuator : %s", v.first.c_str());
    }
    return false;
  }

  publisher_ = new realtime_tools::RealtimePublisher<mtrace_tools::MotorResistance>(node, "resistance", 1, true);
  if (publisher_ == NULL)
    return false;
  publisher_->msg_.samples.resize(buffer_size_);


  return true;
}

void MotorResistanceRecorder::starting()
{
  assert(actuator_!=NULL);
  const pr2_hardware_interface::ActuatorState &a(actuator_->state_);
  last_velocity_ = a.velocity_;
  last_position_ = a.position_;
}

void MotorResistanceRecorder::update()
{
  assert(publisher_!=NULL);
  assert(actuator_!=NULL);


  // Sample when velocity == 0 or crosses 0
  const pr2_hardware_interface::ActuatorState &a(actuator_->state_);
  if (a.position_ != last_position_)
  {
    positive_direction_ = a.position_ > last_position_;
  }
  if ( (a.velocity_==0)  ||  ((last_velocity_>0)&&(a.velocity_<0))  ||  ((last_velocity_<0)&&(a.velocity_>0)) )
  {
    assert(buffer_.size() == buffer_size_);
    if (buffer_index_ < buffer_size_)
    {
      mtrace_tools::MotorResistanceSample &s(buffer_[buffer_index_]);
      s.velocity = a.velocity_;
      s.current  = a.last_measured_current_;
      s.voltage  = a.motor_voltage_;
      s.position = a.position_;
      s.positive_direction = positive_direction_;
      ++buffer_index_;
    }
  }
  last_velocity_ = a.velocity_;
  last_position_ = a.position_;

  // Try to publish when buffer is full
  if (buffer_index_ >= buffer_.size())
  {
    if (publisher_->trylock())
    {
      publisher_->msg_.header.stamp = ros::Time::now();  
      buffer_.swap(publisher_->msg_.samples);
      assert(buffer_.size() == buffer_size_);
      buffer_index_ = 0;
      publisher_->unlockAndPublish();
    }
  }
}


} // end namespace motor_resistance_recorder



/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(MotorResistanceRecorder, 
                         motor_resistance_recorder::MotorResistanceRecorder, 
                         pr2_controller_interface::Controller)
