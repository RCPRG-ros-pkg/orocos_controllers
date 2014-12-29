/*
 * Copyright (c) 2010-2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * FakeServo.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */
#include "FakeServo.h"

#include <string>

#include <ocl/Component.hpp>

FakeServo::FakeServo(const std::string& name) :
    RTT::TaskContext(name, PreOperational), number_of_joints_prop_("number_of_joints", "", 0) {
  this->ports()->addPort("JointPositionCommand", cmd_jnt_pos_port_);
  this->ports()->addPort("JointPosition", msr_jnt_pos_port_);
  this->ports()->addPort("DesiredJointPosition", des_jnt_pos_port_);
  this->ports()->addPort("CommandPeriod", command_period_port_);

  this->addProperty(number_of_joints_prop_);
}

FakeServo::~FakeServo() {
}

bool FakeServo::configureHook() {
  if ((number_of_joints_ = number_of_joints_prop_.get()) == 0) {
    return false;
  }
  initial_pos_.resize(number_of_joints_);
  for (unsigned int i = 0; i < number_of_joints_; i++) {
    RTT::base::PropertyBase* prop;
    prop = this->getProperty(std::string("joint") + static_cast<char>(i + 48) + "_position");
    if (prop) {
      initial_pos_[i] = ((RTT::Property<double>*) prop)->get();
    } else {
      initial_pos_[i] = 0.0;
    }
  }

  jnt_pos_.resize(number_of_joints_);
  return true;
}

bool FakeServo::startHook() {
  jnt_pos_ = initial_pos_;

  dt_ = this->getPeriod();

  return true;
}

void FakeServo::updateHook() {
  cmd_jnt_pos_port_.read(jnt_pos_);

  des_jnt_pos_port_.write(jnt_pos_);
  msr_jnt_pos_port_.write(jnt_pos_);

  command_period_port_.write(dt_);
}

ORO_CREATE_COMPONENT(FakeServo)
