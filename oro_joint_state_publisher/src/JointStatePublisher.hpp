#ifndef JOINTSTATEPUBLISHER_HPP
#define JOINTSTATEPUBLISHER_HPP

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include "sensor_msgs/JointState.h"
#include "oro_servo_msgs/ServoStates.h"

class JointStatePublisher : public RTT::TaskContext
{
public:
  JointStatePublisher(const std::string& name);
  ~JointStatePublisher();

  bool configureHook();
  void updateHook();
protected:
  RTT::InputPort<oro_servo_msgs::ServoStates> servo_state_port;
  RTT::OutputPort<sensor_msgs::JointState> joint_state_port;

  RTT::Property<unsigned int> number_of_joints_prop;
private:
  sensor_msgs::JointState joint_state_;
  oro_servo_msgs::ServoStates servo_state_;
  unsigned int number_of_joints_;
  std::vector<std::string> names_;
};

#endif

