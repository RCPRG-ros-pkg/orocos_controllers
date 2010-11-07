/*
 * FakeServo.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include "FakeServo.h"

FakeServo::FakeServo(const std::string& name) :
    RTT::TaskContext(name, PreOperational), setpoint_port("setpoint"), jointState_port(
      "servo_states"), numberOfJoints_prop("number_of_joints", "", 0)
{

  this->ports()->addPort(setpoint_port);
  this->ports()->addPort(jointState_port);

  this->addProperty(numberOfJoints_prop);
}

FakeServo::~FakeServo()
{

}

bool FakeServo::configureHook()
{
  if ((numberOfJoints_ = numberOfJoints_prop.get()) == 0)
    return false;
  initial_pos_.resize(numberOfJoints_);
  for (unsigned int i = 0; i < numberOfJoints_; i++)
  {
    RTT::base::PropertyBase* prop;
    prop = this->getProperty(std::string("joint") + (char) (i + 48) + "_position");
    if(prop)
    {
      initial_pos_[i] = ((RTT::Property<double>*) prop)->get();
    } else
    {
      initial_pos_[i] = 0.0;
    }
  }


  joint_state_.states.resize(numberOfJoints_);
  setpoint_.setpoints.resize(numberOfJoints_);
  return true;
}

bool FakeServo::startHook()
{

  for (unsigned int i = 0; i < numberOfJoints_; i++)
  {
    joint_state_.states[i].position = initial_pos_[i];
    joint_state_.states[i].velocity = 0.0;
    joint_state_.states[i].effort = 0.0;
  }
  return true;
}

void FakeServo::updateHook()
{
  if (setpoint_port.read(setpoint_) == RTT::NewData)
  {
    if (setpoint_.setpoints.size() == numberOfJoints_)
    {
      for (unsigned int i = 0; i < numberOfJoints_; i++)
      {
        joint_state_.states[i].position = setpoint_.setpoints[i].position;
        joint_state_.states[i].velocity = setpoint_.setpoints[i].velocity;
      }
    }
  }
  jointState_port.write(joint_state_);
}

ORO_CREATE_COMPONENT( FakeServo )
