/*
 * Copyright (c) 2010-2015 Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * InternalSpaceSplineTrajectoryGenerator.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <rtt/Component.hpp>
#include <string>
#include <exception>
#include "InternalSpaceSplineTrajectoryGenerator.h"

#include "rtt_rosclock/rtt_rosclock.h"

InternalSpaceSplineTrajectoryGenerator::InternalSpaceSplineTrajectoryGenerator(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      last_point_not_set_(false),
      trajectory_active_(false),
      trajectory_ptr_(0),
      number_of_joints_(0) {
  this->ports()->addPort("trajectoryPtr", port_trajectory_);
  this->ports()->addPort("JointPositionCommand",
                         port_internal_space_position_command_);
  this->ports()->addPort("JointPosition",
                         port_internal_space_position_measurement_);
  this->ports()->addPort("GeneratorActiveOut", port_generator_active_);
  this->ports()->addPort("IsSynchronisedIn", port_is_synchronised_);

  this->addProperty("number_of_joints", number_of_joints_);

  return;
}

InternalSpaceSplineTrajectoryGenerator::~InternalSpaceSplineTrajectoryGenerator() {
  return;
}

bool InternalSpaceSplineTrajectoryGenerator::configureHook() {
  try {
    if (number_of_joints_ <= 0)
      return false;

    vel_profile_.resize(number_of_joints_);

    des_jnt_pos_.resize(number_of_joints_);
    port_internal_space_position_command_.setDataSample(des_jnt_pos_);

    return true;
  } catch (std::exception &e) {
    RTT::Logger::log(RTT::Logger::Error) << e.what() << RTT::endlog();
    return false;
  } catch (...) {
    RTT::Logger::log(RTT::Logger::Error) << "unknown exception !!!"
        << RTT::endlog();
    return false;
  }
}

bool InternalSpaceSplineTrajectoryGenerator::startHook() {
  bool is_synchronised = true;

  if (port_internal_space_position_measurement_.read(setpoint_)
      == RTT::NoData) {
    return false;
  }

  port_is_synchronised_.read(is_synchronised);

  if (!is_synchronised) {
    return false;
  }

  port_generator_active_.write(true);
  last_point_not_set_ = false;
  trajectory_active_ = false;
  return true;
}

void InternalSpaceSplineTrajectoryGenerator::stopHook() {
  port_generator_active_.write(false);
}

void InternalSpaceSplineTrajectoryGenerator::updateHook() {
  port_generator_active_.write(true);

  trajectory_msgs::JointTrajectoryConstPtr trj_ptr_tmp;
  if (port_trajectory_.read(trj_ptr_tmp) == RTT::NewData) {
    trajectory_ = trj_ptr_tmp;
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
//   std::cout << std::endl<< "InternalSpaceSplineTrajectoryGenerator new trj" << std::endl<< std::endl<< std::endl;
  }

// std::cout << "InternalSpaceSplineTrajectoryGenerator" << std::endl;
  ros::Time now = rtt_rosclock::host_now();
  if (trajectory_active_ && trajectory_ && (trajectory_->header.stamp < now)) {
    for (; trajectory_ptr_ < trajectory_->points.size(); trajectory_ptr_++) {
      ros::Time trj_time = trajectory_->header.stamp
          + trajectory_->points[trajectory_ptr_].time_from_start;
      if (trj_time > now) {
        for (unsigned int i = 0; i < number_of_joints_; i++) {
          if (trajectory_ptr_ < 1) {
            if (trajectory_->points[trajectory_ptr_].accelerations.size() > 0
                && trajectory_->points[trajectory_ptr_].velocities.size() > 0) {
              vel_profile_[i].SetProfileDuration(
                  old_point_(i), 0.0, 0.0,
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  trajectory_->points[trajectory_ptr_].accelerations[i],
                  trajectory_->points[trajectory_ptr_].time_from_start.toSec());
            } else if (trajectory_->points[trajectory_ptr_].velocities.size()
                > 0) {
              vel_profile_[i].SetProfileDuration(
                  old_point_(i), 0.0,
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  trajectory_->points[trajectory_ptr_].time_from_start.toSec());
            } else {
              vel_profile_[i].SetProfileDuration(
                  old_point_(i),
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].time_from_start.toSec());
            }
          } else {
            if (trajectory_->points[trajectory_ptr_ - 1].accelerations.size()
                > 0
                && trajectory_->points[trajectory_ptr_].accelerations.size()
                    > 0) {
              vel_profile_[i].SetProfileDuration(
                  trajectory_->points[trajectory_ptr_ - 1].positions[i],
                  trajectory_->points[trajectory_ptr_ - 1].velocities[i],
                  trajectory_->points[trajectory_ptr_ - 1].accelerations[i],
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  trajectory_->points[trajectory_ptr_].accelerations[i],
                  (trajectory_->points[trajectory_ptr_].time_from_start
                      - trajectory_->points[trajectory_ptr_ - 1].time_from_start)
                      .toSec());
            } else if (trajectory_->points[trajectory_ptr_ - 1].velocities.size()
                > 0
                && trajectory_->points[trajectory_ptr_].velocities.size() > 0) {
              vel_profile_[i].SetProfileDuration(
                  trajectory_->points[trajectory_ptr_ - 1].positions[i],
                  trajectory_->points[trajectory_ptr_ - 1].velocities[i],
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  (trajectory_->points[trajectory_ptr_].time_from_start
                      - trajectory_->points[trajectory_ptr_ - 1].time_from_start)
                      .toSec());
            } else {
              vel_profile_[i].SetProfileDuration(
                  trajectory_->points[trajectory_ptr_ - 1].positions[i],
                  trajectory_->points[trajectory_ptr_].positions[i],
                  (trajectory_->points[trajectory_ptr_].time_from_start
                      - trajectory_->points[trajectory_ptr_ - 1].time_from_start)
                      .toSec());
            }
          }
        }
        break;
      }
    }

    if (trajectory_ptr_ < trajectory_->points.size()) {
      double t;
      if (trajectory_ptr_ < 1) {
        t = (now - trajectory_->header.stamp).toSec();
      } else {
        t = (now - trajectory_->header.stamp).toSec()
            - trajectory_->points[trajectory_ptr_ - 1].time_from_start.toSec();
      }

      for (unsigned int i = 0; i < number_of_joints_; i++) {
        setpoint_(i) = vel_profile_[i].Pos(t);
        // setpoint_.setpoints[i].velocity = velProfile_[i].Vel(time * dt);
        // setpoint_.setpoints[i].acceleration = velProfile_[i].Acc(time * dt);
      }

    } else if (last_point_not_set_) {
      for (unsigned int i = 0; i < number_of_joints_; i++) {
        setpoint_(i) = trajectory_->points[trajectory_->points.size() - 1]
            .positions[i];
      }
      trajectory_ = trajectory_msgs::JointTrajectoryConstPtr();
      last_point_not_set_ = false;
    }
  }

  port_internal_space_position_command_.write(setpoint_);
}

ORO_CREATE_COMPONENT(InternalSpaceSplineTrajectoryGenerator)

