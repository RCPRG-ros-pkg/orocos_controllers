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
  RTT::InputPort<oro_servo_msgs::ServoStates> msrJnt_port;
  RTT::OutputPort<sensor_msgs::JointState> jointState_port;

  RTT::Property<unsigned int> nJoints_prop;
private:
  sensor_msgs::JointState jState;
  oro_servo_msgs::ServoStates msrJnt;
  unsigned int nJoints;
  std::vector<std::string> names;
};

#endif

