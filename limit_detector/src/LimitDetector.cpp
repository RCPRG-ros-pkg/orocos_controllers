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
 * FakeServo.cpp
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
      number_of_drives_(0) {

  this->addProperty("initial_pos", initial_pos_).doc("");
}

LimitDetector::~LimitDetector() {
}

bool LimitDetector::configureHook() {
  number_of_drives_ = initial_pos_.size();

  port_motor_position_command_list_.resize(number_of_drives_);
  port_motor_position_list_.resize(number_of_drives_);
  current_pos_.resize(number_of_drives_);

  for (int j = 0; j < number_of_drives_; j++) {
    char MotorPositionCommand_port_name[32];
    snprintf(MotorPositionCommand_port_name,
             sizeof(MotorPositionCommand_port_name), "MotorPositionCommand_%d",
             j);
    port_motor_position_command_list_[j] =
        new typeof(*port_motor_position_command_list_[j]);
    this->ports()->addPort(MotorPositionCommand_port_name,
                           *port_motor_position_command_list_[j]);

    char MotorPosition_port_name[32];
    snprintf(MotorPosition_port_name, sizeof(MotorPosition_port_name),
             "MotorPosition_%d", j);
    port_motor_position_list_[j] = new typeof(*port_motor_position_list_[j]);
    this->ports()->addPort(MotorPosition_port_name,
                           *port_motor_position_list_[j]);

    current_pos_[j] = initial_pos_[j];
  }
  return true;
}

void LimitDetector::updateHook() {
  double tmp_pos_command;

  for (int j = 0; j < number_of_drives_; j++) {
    if (port_motor_position_command_list_[j]->read(tmp_pos_command)
        == RTT::NewData) {
      current_pos_[j] = tmp_pos_command;
    }
    port_motor_position_list_[j]->write(current_pos_[j]);
  }
}

ORO_CREATE_COMPONENT(LimitDetector)
