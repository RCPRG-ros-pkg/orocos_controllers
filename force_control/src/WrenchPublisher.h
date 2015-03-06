// Copyright WUT 2015
/*
 * WrenchPublisher.cpp
 *
 *  Created on: 5 March 2015
 *      Author: twiniars
 */

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
  RTT::InputPort<geometry_msgs::Wrench> port_input_wrench_;
  RTT::OutputPort<geometry_msgs::WrenchStamped> port_output_wrench_stamped_;

  geometry_msgs::WrenchStamped output_wrench_stamped_;

  // Properties
  std::string frame_id_;


};

#endif  // WRENCHPUBLISHER_H_
