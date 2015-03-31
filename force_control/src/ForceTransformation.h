/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FORCETRANSFORMATION_H_
#define FORCETRANSFORMATION_H_

#include <string>

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
