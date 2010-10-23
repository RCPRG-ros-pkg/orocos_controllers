#ifndef JOINTSTATEPUBLISHER_HPP
#define JOINTSTATEPUBLISHER_HPP

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include "sensor_msgs/JointState.h"
#include "JointState.hpp"

class JointStatePublisher : public RTT::TaskContext
{
public:
  JointStatePublisher(const std::string& name);
  ~JointStatePublisher();

  bool configureHook();
  void updateHook();
protected:
  RTT::InputPort<std::vector<JointState> > msrJnt_port;
  RTT::OutputPort<sensor_msgs::JointState> jointState_port;

  RTT::Property<int> nJoints_prop;
private:
  sensor_msgs::JointState jState;
  std::vector<JointState> msrJnt;
  int nJoints;
  std::vector<std::string> names;
};

#endif

