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
#include <cmath>
#include "common_headers/string_colors.h"
#include "LimitDetector.h"

LimitDetector::LimitDetector(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      number_of_ports_(0),
      emergency_stop_out_("EmergencyStopOut"),
      pos_inc_initiated_(false),
      console_notification_active_(false),
      step_nr(0),
      last_step_valid(true) {

  this->addProperty("lower_limits", lower_limits_).doc("");
  this->addProperty("upper_limits", upper_limits_).doc("");
  this->addProperty("pos_inc_limit", pos_inc_limit_).doc("");
  this->addProperty("pos_limit_active", pos_limit_active_).doc("");
  this->addProperty("pos_inc_limit_active", pos_inc_limit_active_).doc("");
  this->addProperty("console_notification_active", console_notification_active_)
      .doc("");

  this->ports()->addPort("InputPort", input_port_);
  this->ports()->addPort("OutputPort", output_port_);

  this->addPort(emergency_stop_out_).doc("Emergency Stop Out");
}

LimitDetector::~LimitDetector() {
}

bool LimitDetector::configureHook() {
  number_of_ports_ = upper_limits_.size();

  if ((number_of_ports_ != lower_limits_.size())
      || (number_of_ports_ != pos_inc_limit_.size())
      || (number_of_ports_ != pos_limit_active_.size())
      || (number_of_ports_ != pos_inc_limit_active_.size())) {
    std::cout << std::endl << RED << "[error] limit detector " << getName()
              << "configuration failed: wrong properties in launch file."
              << RESET << std::endl;

    return false;
  }

  previous_pos_.resize(number_of_ports_);
  current_pos_.resize(number_of_ports_);
  pos_inc_.resize(number_of_ports_);

  output_port_.setDataSample(current_pos_);

  return true;
}

void LimitDetector::updateHook() {
  step_nr++;
  if (RTT::NewData == input_port_.read(current_pos_)) {
    bool check_succeed = true;

    bool write_to_console = console_notification_active_ && last_step_valid;

    for (int j = 0; j < number_of_ports_; j++) {
      if (!(std::isfinite(current_pos_[j]))) {
        if (write_to_console) {
          std::cout << std::endl << RED << "[error] limit detector: "
              << getName() << " step nr: " << step_nr
              << " infinite limit axis: " << j << " value: " << current_pos_[j]
              << RESET << std::endl;
        }
        check_succeed = false;
      }

      if (pos_limit_active_[j]) {
        if (current_pos_[j] < lower_limits_[j]) {
          if (write_to_console) {
            std::cout << std::endl << RED << "[error] limit detector: "
                << getName() << " step nr: " << step_nr
                << " lower pos limit axis: " << j << " value: "
                << current_pos_[j] << RESET << std::endl;
          }

          check_succeed = false;
        } else if (current_pos_[j] > upper_limits_[j]) {
          if (write_to_console) {
            std::cout << std::endl << RED << "[error] limit detector: "
                << getName() << " step nr: " << step_nr
                << " upper pos limit axis: " << j << " value: "
                << current_pos_[j] << RESET << std::endl;
          }
          check_succeed = false;
        }
      }

      if ((pos_inc_initiated_) &&(pos_inc_limit_active_[j])) {
        pos_inc_[j] = current_pos_[j] - previous_pos_[j];
        if (fabs(pos_inc_[j]) > pos_inc_limit_[j]) {
          if (write_to_console) {
            std::cout << std::endl << RED << "[error] limit detector: "
                << getName() << " step nr: " << step_nr
                << " pos inc limit axis: " << j << " value: " << pos_inc_[j]
                << RESET << std::endl;
          }
          check_succeed = false;
        }
      }
    }

    if (check_succeed) {
      previous_pos_ = current_pos_;
      pos_inc_initiated_ = true;
      output_port_.write(current_pos_);
      if (!last_step_valid) {
        if (console_notification_active_) {
          std::cout << std::endl << RED << "limit detector: " << getName()
                    << " step nr: " << step_nr << " valid values reestablished"
                    << RESET << std::endl;
        }
      }

      last_step_valid = true;
    } else {
      emergency_stop_out_.write(true);
      last_step_valid = false;
    }
  }
}

ORO_CREATE_COMPONENT(LimitDetector)
