// Copyright WUT 2015

#ifndef WRENCHPUBLISHER_H_
#define WRENCHPUBLISHER_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "kdl_conversions/kdl_msg.h"

#include <force_control_msgs/ToolGravityParam.h>

class WrenchPublisher : public RTT::TaskContext {
 public:
  explicit WrenchPublisher(const std::string& name);
  virtual ~WrenchPublisher();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:
  RTT::InputPort<geometry_msgs::Wrench> port_input_wrist_wrench_;
  RTT::OutputPort<geometry_msgs::WrenchStamped> port_output_wrist_wrench_;
};

#endif  // WRENCHPUBLISHER_H_
