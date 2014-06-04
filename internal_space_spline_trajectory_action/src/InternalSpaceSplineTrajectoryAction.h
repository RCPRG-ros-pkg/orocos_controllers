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
 * InterrnalSpaceTrajectoryAction.h
 *
 * Action for both the motor and joint spline interpolation
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz
 */

#ifndef INTERNALSPACETRAJECTORYACTION_H_
#define INTERNALSPACETRAJECTORYACTION_H_

#include <string>
#include <vector>
#include <Eigen/Dense>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

class InternalSpaceSplineTrajectoryAction : public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;
  typedef boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> Goal;

 public:
  InternalSpaceSplineTrajectoryAction(const std::string& name);
  virtual ~InternalSpaceSplineTrajectoryAction();

  bool configureHook();
  bool startHook();
  void updateHook();

 protected:
  RTT::OutputPort<trajectory_msgs::JointTrajectoryConstPtr> trajectory_ptr_port_;

  RTT::Property<int> numberOfJoints_prop_;

  RTT::InputPort<trajectory_msgs::JointTrajectory> command_port_;

  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_joint_position_command_;

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  void commandCB();
  void compleatCB();
  void bufferReadyCB();

  std::vector<std::string> jointNames_;
  unsigned int numberOfJoints_;

  std::vector<double> lowerLimits_;
  std::vector<double> upperLimits_;

  std::vector<int> remapTable_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd desired_joint_position_;

  ros::Time trajectory_finish_time_;

  // RTT action server
  rtt_actionlib::RTTActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  bool goal_active_;
  GoalHandle activeGoal_;
  bool enable_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
};

#endif /* INTERNALSPACETRAJECTORYACTION_H_ */
