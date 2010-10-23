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

JointSplineTrajectoryGenerator::JointSplineTrajectoryGenerator(const std::string& name) : RTT::TaskContext(name, PreOperational), trajectoryPoint_port("trajectoryPoint"), bufferReady_port("bufferReady"), setpoint_port("setpoint"), jointState_port("jointState"), numberOfJoints_prop("numberOfJoints", "number of joints used", 0)
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

  coeff.resize(numberOfJoints);
  for (unsigned int i = 0; i < numberOfJoints; i++)
    coeff[i].resize(6);

  setpoint.resize(numberOfJoints);
  setpoint_port.setDataSample(setpoint);

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
    RTT::Logger::log(RTT::Logger::Debug) << "generating setpoint" << setpoint.size() << RTT::endlog();

    for (unsigned int i = 0; i < numberOfJoints; i++)
      sampleSpline(coeff[i], time * dt, setpoint[i].position, setpoint[i].velocity, setpoint[i].acceleration);
    setpoint_port.write(setpoint);

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
            getQuinticSplineCoefficients(
              trajectoryOld.positions[j], trajectoryOld.velocities[j], trajectoryOld.accelerations[j],
              trajectoryNew.positions[j], trajectoryNew.velocities[j], trajectoryNew.accelerations[j],
              trajectoryNew.time_from_start.toSec(),
              coeff[j]);
          }
          else if (trajectoryOld.velocities.size() > 0 && trajectoryNew.velocities.size() > 0)
          {
            getCubicSplineCoefficients(
              trajectoryOld.positions[j], trajectoryOld.velocities[j],
              trajectoryNew.positions[j], trajectoryNew.velocities[j],
              trajectoryNew.time_from_start.toSec(),
              coeff[j]);
          }
          else
          {
            coeff[j][0] = trajectoryOld.positions[j];
            if (trajectoryNew.time_from_start.toSec() == 0.0)
              coeff[j][1] = 0.0;
            else
              coeff[j][1] = (trajectoryNew.positions[j] - trajectoryOld.positions[j]) / trajectoryNew.time_from_start.toSec();
            coeff[j][2] = 0.0;
            coeff[j][3] = 0.0;
            coeff[j][4] = 0.0;
            coeff[j][5] = 0.0;
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
      std::vector<JointState> joints;
      jointState_port.read(joints);

      trajectoryOld.positions.resize(numberOfJoints);
      trajectoryOld.velocities.resize(numberOfJoints);
      trajectoryOld.accelerations.resize(numberOfJoints);

      for (unsigned int i = 0; i < numberOfJoints; i++)
      {
        trajectoryOld.positions[i] = joints[i].position;
        trajectoryOld.velocities[i] = joints[i].velocity;
        trajectoryOld.accelerations[i] = joints[i].acceleration;
      }

      for (unsigned int j = 0; j < numberOfJoints; ++j)
      {
        if (trajectoryOld.accelerations.size() > 0 && trajectoryNew.accelerations.size() > 0)
        {
          getQuinticSplineCoefficients(
            trajectoryOld.positions[j], trajectoryOld.velocities[j], trajectoryOld.accelerations[j],
            trajectoryNew.positions[j], trajectoryNew.velocities[j], trajectoryNew.accelerations[j],
            trajectoryNew.time_from_start.toSec(),
            coeff[j]);
        }
        else if (trajectoryOld.velocities.size() > 0 && trajectoryNew.velocities.size() > 0)
        {
          getCubicSplineCoefficients(
            trajectoryOld.positions[j], trajectoryOld.velocities[j],
            trajectoryNew.positions[j], trajectoryNew.velocities[j],
            trajectoryNew.time_from_start.toSec(),
            coeff[j]);
        }
        else
        {
          coeff[j][0] = trajectoryOld.positions[j];
          if (trajectoryNew.time_from_start.toSec() == 0.0)
            coeff[j][1] = 0.0;
          else
            coeff[j][1] = (trajectoryNew.positions[j] - trajectoryOld.positions[j]) / trajectoryNew.time_from_start.toSec();
          coeff[j][2] = 0.0;
          coeff[j][3] = 0.0;
          coeff[j][4] = 0.0;
          coeff[j][5] = 0.0;
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

void JointSplineTrajectoryGenerator::getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc, double time, std::vector<double>& coefficients)
{
  coefficients.resize(6);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5*end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                       12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
    coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                       16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                       6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
  }
}

void JointSplineTrajectoryGenerator::getCubicSplineCoefficients(double start_pos, double start_vel,
    double end_pos, double end_vel, double time, std::vector<double>& coefficients)
{
  coefficients.resize(4);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
}

void JointSplineTrajectoryGenerator::sampleSpline(const std::vector<double>& coeff_, double time_, double& position, double& velocity, double& acceleration)
{
  double t[6];
  generatePowers(5, time_, t);

  position = t[0]*coeff_[0] +
             t[1]*coeff_[1] +
             t[2]*coeff_[2] +
             t[3]*coeff_[3] +
             t[4]*coeff_[4] +
             t[5]*coeff_[5];

  velocity = t[0]*coeff_[1] +
             2.0*t[1]*coeff_[2] +
             3.0*t[2]*coeff_[3] +
             4.0*t[3]*coeff_[4] +
             5.0*t[4]*coeff_[5];

  acceleration = 2.0*t[0]*coeff_[2] +
                 6.0*t[1]*coeff_[3] +
                 12.0*t[2]*coeff_[4] +
                 20.0*t[3]*coeff_[5];
}

ORO_CREATE_COMPONENT( JointSplineTrajectoryGenerator )
