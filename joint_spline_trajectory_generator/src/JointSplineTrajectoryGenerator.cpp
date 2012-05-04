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
 * JointSplineTrajectoryGenerator.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include <exception>

#include "JointSplineTrajectoryGenerator.h"

JointSplineTrajectoryGenerator::JointSplineTrajectoryGenerator(const std::string& name) : RTT::TaskContext(name, PreOperational), number_of_joints_prop_("number_of_joints", "number of joints used", 0)
{
  this->ports()->addPort("trajectory_point", trajectory_point_port_);
  this->ports()->addPort("buffer_ready", buffer_ready_port_);
  this->ports()->addPort("JointPositionCommand", jnt_pos_port_);
  this->ports()->addPort("DesiredJointPosition", cmd_jnt_pos_port_);
  this->ports()->addPort("trajectory_compleat", trajectory_compleat_port_);
  this->ports()->addEventPort("CommandPeriod", command_period_port_);

  this->addProperty(number_of_joints_prop_);
  
  return;
}

JointSplineTrajectoryGenerator::~JointSplineTrajectoryGenerator()
{
  return;
}

bool JointSplineTrajectoryGenerator::configureHook()
{
  try
  {
    if ((number_of_joints_ = number_of_joints_prop_.get()) <= 0)
      throw std::logic_error("number of joints must be positive");

    trajectory_old_.positions.reserve(number_of_joints_);
    trajectory_old_.velocities.reserve(number_of_joints_);
    trajectory_old_.accelerations.reserve(number_of_joints_);

    trajectory_new_.positions.reserve(number_of_joints_);
    trajectory_new_.velocities.reserve(number_of_joints_);
    trajectory_new_.accelerations.reserve(number_of_joints_);

    vel_profile_.resize(number_of_joints_);

    des_jnt_pos_.resize(number_of_joints_);
    jnt_pos_port_.setDataSample(des_jnt_pos_);
	
	return true;
  }
  catch (std::exception &e)
  {
	  RTT::Logger::log(RTT::Logger::Error) << e.what() << RTT::endlog();
	  return false;
  }
  catch (...)
  {
	  RTT::Logger::log(RTT::Logger::Error) << "unknown exception !!!" << RTT::endlog();
	  return false;
  }
}

bool JointSplineTrajectoryGenerator::startHook()
{

//  if(command_period_port_.read(dt_) != RTT::NewData)
//    return false;

  time_ = 0;
  trajectory_ready_ = false;
  buffer_ready_ = true;

  buffer_ready_port_.write(buffer_ready_);

  return true;
}

void JointSplineTrajectoryGenerator::updateHook()
{

  command_period_port_.read(dt_);
  
  if (trajectory_ready_)
  {
    for (unsigned int i = 0; i < number_of_joints_; i++)
    {
      des_jnt_pos_[i] = vel_profile_[i].Pos(dt_ * time_);
     // setpoint_.setpoints[i].velocity = velProfile_[i].Vel(time * dt);
     // setpoint_.setpoints[i].acceleration = velProfile_[i].Acc(time * dt);
     
     //RTT::Logger::log(RTT::Logger::Debug) << "time = " << time_ << " joint [" << i << "] pos : " << des_jnt_pos_[i] << RTT::endlog();
     
    }

    jnt_pos_port_.write(des_jnt_pos_);

    if (time_++ >= end_time_)
    {
      if (!buffer_ready_)
      {
        RTT::Logger::log(RTT::Logger::Debug) << "processing trajectory point [trajectory_ready = true]" << RTT::endlog();

      //  RTT::Logger::log(RTT::Logger::Debug) << "old : p: " << trajectory_old_.positions[0] << " v: " << trajectory_old_.velocities[0] << " new : p: " << trajectory_new_.positions[0] << " v: " << trajectory_new_.velocities[0] << RTT::endlog();

        for (unsigned int j = 0; j < number_of_joints_; ++j)
        {
          if (trajectory_old_.accelerations.size() > 0 && trajectory_new_.accelerations.size() > 0)
          {
            vel_profile_[j].SetProfileDuration(
              trajectory_old_.positions[j], trajectory_old_.velocities[j], trajectory_old_.accelerations[j],
              trajectory_new_.positions[j], trajectory_new_.velocities[j], trajectory_new_.accelerations[j],
              trajectory_new_.time_from_start.toSec());
          }
          else if (trajectory_old_.velocities.size() > 0 && trajectory_new_.velocities.size() > 0)
          {
            vel_profile_[j].SetProfileDuration(
              trajectory_old_.positions[j], trajectory_old_.velocities[j],
              trajectory_new_.positions[j], trajectory_new_.velocities[j],
              trajectory_new_.time_from_start.toSec());
          }
          else
          {
            vel_profile_[j].SetProfileDuration(trajectory_old_.positions[j], trajectory_new_.positions[j], trajectory_new_.time_from_start.toSec());
          }
        }

        end_time_ = trajectory_new_.time_from_start.toSec()/dt_;
        time_ = 0;
        trajectory_old_ = trajectory_new_;
        buffer_ready_ = true;
        //RTT::Logger::log(RTT::Logger::Debug) 	<< "trajectory_ready_ = true buffer_ready = " << buffer_ready_ << RTT::endlog();
        buffer_ready_port_.write(buffer_ready_);
      }
      else
      {
        RTT::Logger::log(RTT::Logger::Debug) << "generator : trajectory compleat" << RTT::endlog();
        trajectory_ready_ = false;
        trajectory_compleat_port_.write(true);
      }
    }
  }
  else
  {
    if (!buffer_ready_)
    {
      RTT::Logger::log(RTT::Logger::Debug) << "processing trajectory point [trajectory_ready = false]" << RTT::endlog();
      std::vector<double> servo;
      cmd_jnt_pos_port_.read(servo);

      trajectory_old_.positions.resize(number_of_joints_);
      trajectory_old_.velocities.resize(number_of_joints_);
      trajectory_old_.accelerations.resize(number_of_joints_);

      if(servo.size() != number_of_joints_)
      {
        std::cout << "servo.states size : " << servo.size() << " expected : " << number_of_joints_ << std::endl;
        return ;
      }      

      for (unsigned int i = 0; i < number_of_joints_; i++)
      {
        trajectory_old_.positions[i] = servo[i];
        trajectory_old_.velocities[i] = 0.0; //servo.setpoints[i].velocity;
        trajectory_old_.accelerations[i] = 0.0; // servo.setpoints[i].acceleration;
      }
      
      for (unsigned int j = 0; j < number_of_joints_; ++j)
      {
        if (trajectory_old_.accelerations.size() > 0 && trajectory_new_.accelerations.size() > 0)
        {
		  RTT::Logger::log(RTT::Logger::Debug) << "pos " << j << " old : " <<  trajectory_old_.positions[j] << " new : " << trajectory_new_.positions[j] << RTT::endlog();
		  RTT::Logger::log(RTT::Logger::Debug) << "vel " << j << " old : " <<  trajectory_old_.velocities[j] << " new : " << trajectory_new_.velocities[j] << RTT::endlog();
		  RTT::Logger::log(RTT::Logger::Debug) << "acc " << j << " old : " <<  trajectory_old_.accelerations[j] << " new : " << trajectory_new_.accelerations[j] << RTT::endlog();
          vel_profile_[j].SetProfileDuration(
            trajectory_old_.positions[j], trajectory_old_.velocities[j], trajectory_old_.accelerations[j],
            trajectory_new_.positions[j], trajectory_new_.velocities[j], trajectory_new_.accelerations[j],
            trajectory_new_.time_from_start.toSec());
        }
        else if (trajectory_old_.velocities.size() > 0 && trajectory_new_.velocities.size() > 0)
        {
		  RTT::Logger::log(RTT::Logger::Debug) << "pos " << j << " old : " <<  trajectory_old_.positions[j] << " new : " << trajectory_new_.positions[j] << RTT::endlog();
		  RTT::Logger::log(RTT::Logger::Debug) << "vel " << j << " old : " <<  trajectory_old_.velocities[j] << " new : " << trajectory_new_.velocities[j] << RTT::endlog();
          vel_profile_[j].SetProfileDuration(
            trajectory_old_.positions[j], trajectory_old_.velocities[j],
            trajectory_new_.positions[j], trajectory_new_.velocities[j],
            trajectory_new_.time_from_start.toSec());
        }
        else
        {
		  RTT::Logger::log(RTT::Logger::Debug) << "pos " << j << " old : " <<  trajectory_old_.positions[j] << " new : " << trajectory_new_.positions[j] << RTT::endlog();
          vel_profile_[j].SetProfileDuration(trajectory_old_.positions[j], trajectory_new_.positions[j], trajectory_new_.time_from_start.toSec());
        }
      }

      RTT::Logger::log(RTT::Logger::Debug) << "time : " << trajectory_new_.time_from_start.toSec() << RTT::endlog();

      trajectory_old_ = trajectory_new_;
      end_time_ = trajectory_new_.time_from_start.toSec()/dt_;
      time_ = 0;
      trajectory_ready_ = true;
      buffer_ready_ = true;
    }
  //  RTT::Logger::log(RTT::Logger::Debug) 	<< "trajectory_ready_ = false buffer_ready = " << buffer_ready_ << RTT::endlog();
    buffer_ready_port_.write(buffer_ready_);
    
  }

  if (trajectory_point_port_.read(trajectory_new_) == RTT::NewData)
  {
    RTT::Logger::log(RTT::Logger::Debug) << "Trajectory point received " << RTT::endlog();

    if (!buffer_ready_)
      RTT::Logger::log(RTT::Logger::Warning) 	<< "Trajectory point buffer not empty overwriteing " << RTT::endlog();

    if (trajectory_new_.positions.size() != number_of_joints_)
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point positions size invalid (received: "
      << trajectory_new_.positions.size()
      << " expected: "
      << number_of_joints_
      << ") " << RTT::endlog();
    }
    else if ((trajectory_new_.velocities.size() != number_of_joints_) && (trajectory_new_.velocities.size() > 0))
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point velocities size invalid (received: "
      << trajectory_new_.velocities.size()
      << " expected: "
      << number_of_joints_
      << " or 0) " << RTT::endlog();
    }
    else if ((trajectory_new_.accelerations.size() != number_of_joints_) && (trajectory_new_.accelerations.size() > 0))
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point accelerations size invalid (received: "
      << trajectory_new_.accelerations.size()
      << " expected: "
      << number_of_joints_
      << " or 0) " << RTT::endlog();
    }
    else if (trajectory_new_.time_from_start.toSec() < 0)
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point time_from_start invalid (<0) " << RTT::endlog();
    }
    else
    {
      buffer_ready_ = false;
      buffer_ready_port_.write(buffer_ready_);
    }
  }
  
  return;
}

ORO_CREATE_COMPONENT( JointSplineTrajectoryGenerator )
