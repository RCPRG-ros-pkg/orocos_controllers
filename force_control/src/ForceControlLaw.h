// Copyright WUT 2014
/*
 * ForceControlLaw.h
 *
 *  Created on: 1 July 2014
 *      Author: twiniars
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

class ForceControlLaw : public RTT::TaskContext {
 public:
  explicit ForceControlLaw(const std::string& name);
  virtual ~ForceControlLaw();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

  double fcl(const double & rdam, const double & inertia, const double & fm,
             const double & fd, const double & dvel, const double & pvel);

 private:
  RTT::InputPort<geometry_msgs::Pose> port_current_end_effector_pose_;
  RTT::OutputPort<geometry_msgs::Pose> port_output_end_effector_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_end_effector_wrench_;

  RTT::InputPort<force_control_msgs::ForceControl> port_current_fcl_param_;

  KDL::Frame cl_ef_pose_kdl_;
  KDL::Twist p_vel_;
  double step_duration_;
};

#endif  // FORCECONTROLLAW_H_
