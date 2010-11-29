/*
 * JointSplineTrajectoryGenerator.cpp
 *
 *  Created on: 22-09-2010
 *      Author: konrad
 */

#include <ocl/Component.hpp>

#include "JointSplineTrajectoryGenerator.h"

static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
}

JointSplineTrajectoryGenerator::JointSplineTrajectoryGenerator(const std::string& name) : RTT::TaskContext(name, PreOperational), trajectoryPoint_port("trajectory_point"), bufferReady_port("buffer_ready"), setpoint_port("setpoint"), jointState_port("servo_states"), numberOfJoints_prop("number_of_joints", "number of joints used", 0)
{
  this->ports()->addPort(trajectoryPoint_port);
  this->ports()->addPort(bufferReady_port);
  this->ports()->addPort(setpoint_port);
  this->ports()->addPort(jointState_port);

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
    RTT::Logger::log(RTT::Logger::Debug) << "generating setpoint" << setpoint_.setpoints.size() << RTT::endlog();

    for (unsigned int i = 0; i < numberOfJoints; i++)
    {
      setpoint_.setpoints[i].position = velProfile_[i].Pos(time * dt);
      setpoint_.setpoints[i].velocity = velProfile_[i].Vel(time * dt);
      setpoint_.setpoints[i].acceleration = velProfile_[i].Acc(time * dt);
    }
    setpoint_port.write(setpoint_);

    if (time >= endTime)
    {
      if (!bufferReady)
      {
        RTT::Logger::log(RTT::Logger::Debug) << "processing trajectory point [trajectory_ready = true]" << RTT::endlog();

        RTT::Logger::log(RTT::Logger::Debug) << "old : p: " << trajectoryOld.positions[0] << " v: " << trajectoryOld.velocities[0] << " new : p: " << trajectoryNew.positions[0] << " v: " << trajectoryNew.velocities[0] << RTT::endlog();

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
            trajectoryNew.time_from_start.toSec());
        }
        else
        {
          velProfile_[j].SetProfileDuration(trajectoryOld.positions[j], trajectoryNew.positions[j], trajectoryNew.time_from_start.toSec());
        }
      }

      trajectoryOld = trajectoryNew;
      endTime = trajectoryNew.time_from_start.toSec()/dt;
      time = 0;
      trajectoryReady = true;
      bufferReady = true;
      bufferReady_port.write(bufferReady);
    }
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
  bufferReady_port.write(bufferReady);
  ++time;
}

ORO_CREATE_COMPONENT( JointSplineTrajectoryGenerator )
