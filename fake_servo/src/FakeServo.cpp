/*
 * FakeServo.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include "FakeServo.h"

FakeServo::FakeServo(const std::string& name) :
    RTT::TaskContext(name, PreOperational), number_of_joints_prop_("number_of_joints", "", 0)
{

  this->ports()->addPort("JointPositionCommand", cmd_jnt_pos_port_);
  this->ports()->addPort("JointPosition", msr_jnt_pos_port_);
  this->ports()->addPort("DesiredJointPosition", des_jnt_pos_port_);
  this->ports()->addPort("CommandPeriod", command_period_port_);

  this->addProperty(number_of_joints_prop_);
}

FakeServo::~FakeServo()
{

}

bool FakeServo::configureHook()
{
  if ((number_of_joints_ = number_of_joints_prop_.get()) == 0)
    return false;
  initial_pos_.resize(number_of_joints_);
  for (unsigned int i = 0; i < number_of_joints_; i++)
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

  jnt_pos_.resize(number_of_joints_);
  return true;
}

bool FakeServo::startHook()
{
  jnt_pos_ = initial_pos_;

  dt_ = this->getPeriod();

  return true;
}

void FakeServo::updateHook()
{
  cmd_jnt_pos_port_.read(jnt_pos_);

  des_jnt_pos_port_.write(jnt_pos_);
  msr_jnt_pos_port_.write(jnt_pos_);
  
  command_period_port_.write(dt_);
  
}

ORO_CREATE_COMPONENT( FakeServo )
