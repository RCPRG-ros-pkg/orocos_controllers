/*
 * Copyright (c) 2010-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * LimitDetector.cpp
 *
 *  Created on: 2015
 *      Author: Tomasz Winiarski
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <rtt/extras/SlaveActivity.hpp>

#include <string>
#include "LimitDetector.h"

LimitDetector::LimitDetector(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      number_of_ports_(0),
      emergency_stop_out_("EmergencyStopOut") {

  this->addProperty("upper_pos_limit", upper_pos_limit_).doc("");
  this->addProperty("lower_pos_limit", lower_pos_limit_).doc("");
  this->addProperty("upper_vel_limit", upper_vel_limit_).doc("");
  this->addProperty("lower_vel_limit", lower_vel_limit_).doc("");

  this->ports()->addPort("InputPort", input_port_);
  this->ports()->addPort("OutputPort", output_port_);

  this->addPort(emergency_stop_out_).doc("Emergency Stop Out");
}

LimitDetector::~LimitDetector() {
}

bool LimitDetector::configureHook() {
  number_of_ports_ = upper_pos_limit_.size();

  if ((number_of_ports_ != lower_pos_limit_.size())
      || (number_of_ports_ != upper_vel_limit_.size())
      || (number_of_ports_ != lower_vel_limit_.size())) {
    return false;
  }

  for (int j = 0; j < number_of_ports_; j++) {
    previous_pos_.resize(number_of_ports_);
    current_pos_.resize(number_of_ports_);

    output_port_.setDataSample(current_pos_);
  }
  return true;
}

void LimitDetector::updateHook() {
  if (RTT::NewData == input_port_.read(current_pos_)) {
    bool check_succesed = true;
    for (int j = 0; j < number_of_ports_; j++) {
      if (current_pos_[j] < lower_pos_limit_[j]) {
        check_succesed = false;
      } else if (current_pos_[j] > upper_pos_limit_[j]) {
        check_succesed = false;
      }
    }
    if (check_succesed) {
      output_port_.write(current_pos_);
    } else {
      emergency_stop_out_.write(true);
    }
  }
}

ORO_CREATE_COMPONENT(LimitDetector)
