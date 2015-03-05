// Copyright WUT 2014
/*
 * ForceTransformation.h
 *
 *  Created on: 1 July 2014
 *      Author: twiniars
 */


#ifndef FORCETRANSFORMATION_H_
#define FORCETRANSFORMATION_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"
#include "kdl_conversions/kdl_msg.h"
#include <Eigen/Dense>
#include <force_control_msgs/ToolGravityParam.h>

class ForceTransformation : public RTT::TaskContext {
 public:
  explicit ForceTransformation(const std::string& name);
  virtual ~ForceTransformation();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:
  RTT::InputPort<geometry_msgs::Pose> port_current_wrist_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_fast_filtered_wrench_;
  RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_slow_filtered_wrench_;
  RTT::OutputPort<geometry_msgs::Wrench> port_output_wrist_wrench_;
  RTT::OutputPort<geometry_msgs::Wrench> port_output_end_effector_wrench_;
  RTT::InputPort<geometry_msgs::Pose> port_tool_;

  RTT::InputPort<force_control_msgs::ToolGravityParam> port_current_tool_gravity_param_;

  KDL::Wrench force_offset_;

  // ForceTrans
  double tool_weight_property_;
  geometry_msgs::Vector3 gravity_arm_in_wrist_property_;

  KDL::Wrench gravity_force_torque_in_base_;
  KDL::Wrench reaction_force_torque_in_wrist_;
  KDL::Vector gravity_arm_in_wrist_kdl_;
  KDL::Frame tool_mass_center_translation_;

  geometry_msgs::Pose sensor_frame_property_;
  bool is_right_turn_frame_property_;

  KDL::Frame sensor_frame_kdl_;
};

#endif  // FORCETRANSFORMATION_H_
