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

#ifndef JOINTSTATEPUBLISHER_HPP
#define JOINTSTATEPUBLISHER_HPP

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <Eigen/Dense>

#include "sensor_msgs/JointState.h"

class JointStatePublisher : public RTT::TaskContext
{
public:
  JointStatePublisher(const std::string& name);
  ~JointStatePublisher();

  bool configureHook();
  void updateHook();
protected:
  RTT::InputPort<Eigen::VectorXd > port_joint_position_;
  RTT::InputPort<Eigen::VectorXd > port_joint_velocity_;
  RTT::InputPort<Eigen::VectorXd > port_joint_effort_;
  RTT::OutputPort<sensor_msgs::JointState> joint_state_port_;

  RTT::Property<std::vector<std::string> > joint_names_prop;
private:
  sensor_msgs::JointState joint_state_;
  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_velocity_;
  Eigen::VectorXd joint_effort_;
  unsigned int number_of_joints_;
  std::vector<std::string> names_;
};

#endif

