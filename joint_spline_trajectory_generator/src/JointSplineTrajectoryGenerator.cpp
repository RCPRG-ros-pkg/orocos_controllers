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

#include "JointSplineTrajectoryGenerator.h"

JointSplineTrajectoryGenerator::JointSplineTrajectoryGenerator(const std::string& name) : RTT::TaskContext(name, PreOperational), trajectoryPoint_port("trajectory_point"), bufferReady_port("buffer_ready"), setpoint_port("setpoint"), jointState_port("servo_states"),trajectoryCompleat_port("trajectory_compleat"), numberOfJoints_prop("number_of_joints", "number of joints used", 0)
{
  this->ports()->addPort(trajectoryPoint_port);
  this->ports()->addPort(bufferReady_port);
  this->ports()->addPort(setpoint_port);
  this->ports()->addPort(jointState_port);
  this->ports()->addPort(trajectoryCompleat_port);

  this->addProperty(numberOfJoints_prop);
}

JointSplineTrajectoryGenerator::~JointSplineTrajectoryGenerator()
{

}

bool JointSplineTrajectoryGenerator::configureHook()
{

  if ((numberOfJoints = numberOfJoints_prop.get()) == 0)
    return false;

  trajectoryOld.positions.reserve(numberOfJoints);
  trajectoryOld.velocities.reserve(numberOfJoints);
  trajectoryOld.accelerations.reserve(numberOfJoints);

  trajectoryNew.positions.reserve(numberOfJoints);
  trajectoryNew.velocities.reserve(numberOfJoints);
  trajectoryNew.accelerations.reserve(numberOfJoints);

  velProfile_.resize(numberOfJoints);

  setpoint_.setpoints.resize(numberOfJoints);
  setpoint_port.setDataSample(setpoint_);

  dt = this->getPeriod();

  return true;
}

bool JointSplineTrajectoryGenerator::startHook()
{

  time = 0;
  trajectoryReady = false;
  bufferReady = true;

  bufferReady_port.write(bufferReady);

  return true;
}

void JointSplineTrajectoryGenerator::updateHook()
{

  if (trajectoryReady)
  {
    for (unsigned int i = 0; i < numberOfJoints; i++)
    {
      setpoint_.setpoints[i].position = velProfile_[i].Pos(time * dt);
      setpoint_.setpoints[i].velocity = velProfile_[i].Vel(time * dt);
      setpoint_.setpoints[i].acceleration = velProfile_[i].Acc(time * dt);
    }

    setpoint_port.write(setpoint_);

    if (time++ >= endTime)
    {
      if (!bufferReady)
      {
        RTT::Logger::log(RTT::Logger::Debug) << "processing trajectory point [trajectory_ready = true]" << RTT::endlog();

      //  RTT::Logger::log(RTT::Logger::Debug) << "old : p: " << trajectoryOld.positions[0] << " v: " << trajectoryOld.velocities[0] << " new : p: " << trajectoryNew.positions[0] << " v: " << trajectoryNew.velocities[0] << RTT::endlog();

        for (unsigned int j = 0; j < numberOfJoints; ++j)
        {
          if (trajectoryOld.accelerations.size() > 0 && trajectoryNew.accelerations.size() > 0)
          {
            velProfile_[j].SetProfileDuration(
              trajectoryOld.positions[j], trajectoryOld.velocities[j], trajectoryOld.accelerations[j],
              trajectoryNew.positions[j], trajectoryNew.velocities[j], trajectoryNew.accelerations[j],
              trajectoryNew.time_from_start.toSec());
          }
          else if (trajectoryOld.velocities.size() > 0 && trajectoryNew.velocities.size() > 0)
          {
            velProfile_[j].SetProfileDuration(
              trajectoryOld.positions[j], trajectoryOld.velocities[j],
              trajectoryNew.positions[j], trajectoryNew.velocities[j],
              trajectoryNew.time_from_start.toSec());;
          }
          else
          {
            velProfile_[j].SetProfileDuration(trajectoryOld.positions[j], trajectoryNew.positions[j], trajectoryNew.time_from_start.toSec());
          }
        }

        endTime = trajectoryNew.time_from_start.toSec()/dt;
        time = 0;
        trajectoryOld = trajectoryNew;
        bufferReady = true;
        bufferReady_port.write(bufferReady);
      }
      else
      {
        trajectoryReady = false;
        trajectoryCompleat_port.write(true);
      }
    }
  }
  else
  {
    if (!bufferReady)
    {
      RTT::Logger::log(RTT::Logger::Debug) << "processing trajectory point [trajectory_ready = false]" << RTT::endlog();
      oro_servo_msgs::ServoStates servo;
      jointState_port.read(servo);

      trajectoryOld.positions.resize(numberOfJoints);
      trajectoryOld.velocities.resize(numberOfJoints);
      trajectoryOld.accelerations.resize(numberOfJoints);

      if(servo.states.size() != numberOfJoints)
      {
        std::cout << "servo.states size : " << servo.states.size() << " expected : " << numberOfJoints << std::endl;
        return ;
      }      

      for (unsigned int i = 0; i < numberOfJoints; i++)
      {
        trajectoryOld.positions[i] = servo.setpoints[i].position;
        trajectoryOld.velocities[i] = servo.setpoints[i].velocity;
        trajectoryOld.accelerations[i] = servo.setpoints[i].acceleration;
      }
      
      for (unsigned int j = 0; j < numberOfJoints; ++j)
      {
        if (trajectoryOld.accelerations.size() > 0 && trajectoryNew.accelerations.size() > 0)
        {
		//  RTT::Logger::log(RTT::Logger::Debug) << "pos " << j << " old : " <<  trajectoryOld.positions[j] << " new : " << trajectoryNew.positions[j] << RTT::endlog();
		//  RTT::Logger::log(RTT::Logger::Debug) << "vel " << j << " old : " <<  trajectoryOld.velocities[j] << " new : " << trajectoryNew.velocities[j] << RTT::endlog();
		//  RTT::Logger::log(RTT::Logger::Debug) << "acc " << j << " old : " <<  trajectoryOld.accelerations[j] << " new : " << trajectoryNew.accelerations[j] << RTT::endlog();
          velProfile_[j].SetProfileDuration(
            trajectoryOld.positions[j], trajectoryOld.velocities[j], trajectoryOld.accelerations[j],
            trajectoryNew.positions[j], trajectoryNew.velocities[j], trajectoryNew.accelerations[j],
            trajectoryNew.time_from_start.toSec());
        }
        else if (trajectoryOld.velocities.size() > 0 && trajectoryNew.velocities.size() > 0)
        {
		//  RTT::Logger::log(RTT::Logger::Debug) << "pos " << j << " old : " <<  trajectoryOld.positions[j] << " new : " << trajectoryNew.positions[j] << RTT::endlog();
		 // RTT::Logger::log(RTT::Logger::Debug) << "vel " << j << " old : " <<  trajectoryOld.velocities[j] << " new : " << trajectoryNew.velocities[j] << RTT::endlog();
          velProfile_[j].SetProfileDuration(
            trajectoryOld.positions[j], trajectoryOld.velocities[j],
            trajectoryNew.positions[j], trajectoryNew.velocities[j],
            trajectoryNew.time_from_start.toSec());
        }
        else
        {
		//  RTT::Logger::log(RTT::Logger::Debug) << "pos " << j << " old : " <<  trajectoryOld.positions[j] << " new : " << trajectoryNew.positions[j] << RTT::endlog();
          velProfile_[j].SetProfileDuration(trajectoryOld.positions[j], trajectoryNew.positions[j], trajectoryNew.time_from_start.toSec());
        }
      }

      trajectoryOld = trajectoryNew;
      endTime = trajectoryNew.time_from_start.toSec()/dt;
      time = 0;
      trajectoryReady = true;
      bufferReady = true;
    }
    bufferReady_port.write(bufferReady);
  }

  if (trajectoryPoint_port.read(trajectoryNew) == RTT::NewData)
  {
    RTT::Logger::log(RTT::Logger::Debug) << "Trajectory point received " << RTT::endlog();

    if (!bufferReady)
      RTT::Logger::log(RTT::Logger::Warning) 	<< "Trajectory point buffer not empty overwriteing " << RTT::endlog();

    if (trajectoryNew.positions.size() != numberOfJoints)
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point positions size invalid (received: "
      << trajectoryNew.positions.size()
      << " expected: "
      << numberOfJoints
      << ") " << RTT::endlog();
    }
    else if ((trajectoryNew.velocities.size() != numberOfJoints) && (trajectoryNew.velocities.size() > 0))
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point velocities size invalid (received: "
      << trajectoryNew.velocities.size()
      << " expected: "
      << numberOfJoints
      << " or 0) " << RTT::endlog();
    }
    else if ((trajectoryNew.accelerations.size() != numberOfJoints) && (trajectoryNew.accelerations.size() > 0))
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point accelerations size invalid (received: "
      << trajectoryNew.accelerations.size()
      << " expected: "
      << numberOfJoints
      << " or 0) " << RTT::endlog();
    }
    else if (trajectoryNew.time_from_start.toSec() < 0)
    {
      RTT::Logger::log(RTT::Logger::Error) 	<< "Received trajectory point time_from_start invalid (<0) " << RTT::endlog();
    }
    else
    {
      bufferReady = false;
    }
  }
}

ORO_CREATE_COMPONENT( JointSplineTrajectoryGenerator )
