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
#include <rtt/Component.hpp>
#include <string>

#include "ForceControlLaw.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

ForceControlLaw::ForceControlLaw(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      step_duration_(0.002) {

  this->ports()->addPort("CurrentEndEffectorPose",
                         port_current_end_effector_pose_);
  this->ports()->addPort("OutputEndEffectorPose",
                         port_output_end_effector_pose_);
  this->ports()->addPort("CurrentEndEffectorWrench",
                         port_current_end_effector_wrench_);
  this->ports()->addPort("CurrentFclParam", port_current_fcl_param_);
  this->ports()->addPort("GeneratorActiveOut", port_generator_active_);
  this->ports()->addPort("IsSynchronisedIn", port_is_synchronised_);
}

ForceControlLaw::~ForceControlLaw() {
}

bool ForceControlLaw::configureHook() {
  return true;
}

bool ForceControlLaw::startHook() {
  bool is_synchronised = true;
  // tool determination
  geometry_msgs::Pose cl_ef_pose;
  if (port_current_end_effector_pose_.read(cl_ef_pose) == RTT::NoData) {
    return false;
  }

  tf::poseMsgToKDL(cl_ef_pose, cl_ef_pose_kdl_);

  force_control_msgs::ForceControl fcl_param;
  // controller parameters have to be set before the component is started
  if (port_current_fcl_param_.read(fcl_param) == RTT::NoData) {
    return false;
  }
  port_is_synchronised_.read(is_synchronised);

  if (!is_synchronised) {
    return false;
  }

  port_generator_active_.write(true);
  return true;
}

void ForceControlLaw::stopHook() {
  port_generator_active_.write(false);
}

void ForceControlLaw::updateHook() {
  port_generator_active_.write(true);
  // current wrench determination
  geometry_msgs::Wrench current_end_effector_wrench;
  port_current_end_effector_wrench_.read(current_end_effector_wrench);
  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_end_effector_wrench, input_force);

  force_control_msgs::ForceControl fcl_param;
  port_current_fcl_param_.read(fcl_param);

  KDL::Twist target_vel;

  target_vel.vel[0] = fcl(fcl_param.reciprocaldamping.translation.x,
                          fcl_param.inertia.translation.x,
                          input_force.force.x(), fcl_param.wrench.force.x,
                          fcl_param.twist.linear.x, p_vel_.vel[0]);

  target_vel.vel[1] = fcl(fcl_param.reciprocaldamping.translation.y,
                          fcl_param.inertia.translation.y,
                          input_force.force.y(), fcl_param.wrench.force.y,
                          fcl_param.twist.linear.y, p_vel_.vel[1]);

  target_vel.vel[2] = fcl(fcl_param.reciprocaldamping.translation.z,
                          fcl_param.inertia.translation.z,
                          input_force.force.z(), fcl_param.wrench.force.z,
                          fcl_param.twist.linear.z, p_vel_.vel[2]);

  target_vel.rot[0] = fcl(fcl_param.reciprocaldamping.rotation.x,
                          fcl_param.inertia.rotation.x, input_force.torque.x(),
                          fcl_param.wrench.torque.x, fcl_param.twist.angular.x,
                          p_vel_.rot[0]);

  target_vel.rot[1] = fcl(fcl_param.reciprocaldamping.rotation.y,
                          fcl_param.inertia.rotation.y, input_force.torque.y(),
                          fcl_param.wrench.torque.y, fcl_param.twist.angular.y,
                          p_vel_.rot[1]);

  target_vel.rot[2] = fcl(fcl_param.reciprocaldamping.rotation.z,
                          fcl_param.inertia.rotation.z, input_force.torque.z(),
                          fcl_param.wrench.torque.z, fcl_param.twist.angular.z,
                          p_vel_.rot[2]);

  p_vel_ = target_vel;

  target_vel = cl_ef_pose_kdl_.M * target_vel;

  cl_ef_pose_kdl_ = KDL::addDelta(cl_ef_pose_kdl_, target_vel, step_duration_);

  geometry_msgs::Pose cl_ef_pose;

  tf::poseKDLToMsg(cl_ef_pose_kdl_, cl_ef_pose);

  port_output_end_effector_pose_.write(cl_ef_pose);
}

double ForceControlLaw::fcl(const double & rdam, const double & inertia,
                            const double & fm, const double & fd,
                            const double & dvel, const double & pvel) {
  return ((rdam * (fd - fm) + dvel) * step_duration_ + rdam * inertia * pvel)
      / (step_duration_ + rdam * inertia);
}

ORO_CREATE_COMPONENT(ForceControlLaw)

