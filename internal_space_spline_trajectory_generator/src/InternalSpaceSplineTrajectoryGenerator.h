/*
 * Copyright (c) 2010, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

/*
 * InternalSpaceSplineTrajectoryGenerator.h
 *
 * Generator for both the motor and joint spline interpolation
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#ifndef INTERNALSPACESPLINETRAJECTORYGENERATOR_H_
#define INTERNALSPACESPLINETRAJECTORYGENERATOR_H_

#include <Eigen/Dense>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>

#include "velocityprofile_spline.hpp"

class InternalSpaceSplineTrajectoryGenerator : public RTT::TaskContext {
 public:
  explicit InternalSpaceSplineTrajectoryGenerator(const std::string& name);
  virtual ~InternalSpaceSplineTrajectoryGenerator();

  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

 protected:
  RTT::InputPort<trajectory_msgs::JointTrajectoryConstPtr> port_trajectory_;

  RTT::OutputPort<Eigen::VectorXd> port_internal_space_position_command_;
  RTT::InputPort<Eigen::VectorXd> port_internal_space_position_measurement_;
  RTT::OutputPort<bool> port_generator_active_;
  RTT::InputPort<bool> port_is_synchronised_;

 private:
  bool last_point_not_set_;
  bool trajectory_active_;
  std::vector<KDL::VelocityProfile_Spline> vel_profile_;

  trajectory_msgs::JointTrajectoryPoint trajectory_old_;
  trajectory_msgs::JointTrajectoryPoint trajectory_new_;

  Eigen::VectorXd des_jnt_pos_, setpoint_, old_point_;

  trajectory_msgs::JointTrajectoryConstPtr trajectory_;
  size_t trajectory_ptr_;
  int number_of_joints_;
};

#endif  // INTERNALSPACESPLINETRAJECTORYGENERATOR_H_

