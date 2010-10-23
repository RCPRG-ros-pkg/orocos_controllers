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
      "jointState"), numberOfJoints_prop("numberOfJoints", "", 0)
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


  joint_state_.resize(numberOfJoints_);
  setpoint_.resize(numberOfJoints_);
  return true;
}

bool FakeServo::startHook()
{

  for (unsigned int i = 0; i < numberOfJoints_; i++)
  {
    joint_state_[i].position = initial_pos_[i];
    joint_state_[i].velocity = 0.0;
    joint_state_[i].acceleration = 0.0;
  }
  return true;
}

void FakeServo::updateHook()
{
  if (setpoint_port.read(setpoint_) == RTT::NewData)
  {
    if (setpoint_.size() == numberOfJoints_)
    {
      for (unsigned int i = 0; i < numberOfJoints_; i++)
      {
        joint_state_[i].position = setpoint_[i].position;
        joint_state_[i].velocity = setpoint_[i].velocity;
        joint_state_[i].acceleration = setpoint_[i].acceleration;

      }
    }
  }
  jointState_port.write(joint_state_);
}

ORO_CREATE_COMPONENT( FakeServo )
