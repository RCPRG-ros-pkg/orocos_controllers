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

#ifndef FORCECONTROLLAW_H_
#define FORCECONTROLLAW_H_
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <force_control_msgs/ForceControl.h>
#include <string>



class ForceControlLaw : public RTT::TaskContext {
 public:
  explicit ForceControlLaw(const std::string& name);
  virtual ~ForceControlLaw();

  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  void updateHook();

  double fcl(const double & rdam, const double & inertia, const double & fm,
             const double & fd, const double & dvel, const double & pvel);

 private:
  RTT::InputPort<geometry_msgs::Pose> port_current_end_effector_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_end_effector_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_end_effector_wrench_;

  RTT::InputPort<force_control_msgs::ForceControl> port_current_fcl_param_;
  RTT::OutputPort<bool> port_generator_active_;
  RTT::InputPort<bool> port_is_synchronised_;

  KDL::Frame cl_ef_pose_kdl_;
  KDL::Twist p_vel_;
  double step_duration_;
};

#endif  // FORCECONTROLLAW_H_
