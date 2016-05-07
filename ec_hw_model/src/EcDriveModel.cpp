/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

#include <rtt/plugin/ServicePlugin.hpp>
#include <string>
#include "EcDriveModel.h"
#include "string_colors.h"

EcDriveModel::EcDriveModel(const std::string &name, int iteration_per_step,
                           int step_per_second, double enc_res,
                           double torque_constant,
                           double input_current_multiplicator, double inertia,
                           double viscous_friction)
    : service_(new RTT::Service(name)),
      enc_motor_position_(0.0),
      enc_motor_velocity_(0.0),
      motor_position_(0.0),
      motor_velocity_(0.0),
      motor_acceleration_(0.0),
      desired_input_(0.0),
      desired_torque_(0.0),
      effective_torque_(0.0),
      m_factor_(0),
      iteration_per_step_(iteration_per_step),
      step_per_second_(step_per_second),
      enc_res_(enc_res),
      torque_constant_(torque_constant),
      input_current_multiplicator_(input_current_multiplicator),
      inertia_(inertia),
      viscous_friction_(viscous_friction),
      control_mode_(HOMING),
      enable_(false),
      homing_(false),
      reset_fault_(false),
      homing_done_(false),
      state_(SWITCH_ON) {
  this->provides()->addPort("motor_position", port_motor_position_);
  this->provides()->addPort("motor_position_command",
                            port_motor_position_command_);
  this->provides()->addPort("motor_velocity", port_motor_velocity_);
  this->provides()->addPort("motor_velocity_command",
                            port_motor_velocity_command_);
  this->provides()->addPort("motor_current", port_motor_current_);
  this->provides()->addPort("motor_current_command",
                            port_motor_current_command_);

  this->provides()->addAttribute("state", *(reinterpret_cast<int*>(&state_)));
  this->provides()->addAttribute("homing_done", homing_done_);

  this->provides()->addOperation("beginHoming", &EcDriveModel::beginHoming,
                                 this, RTT::OwnThread);
  this->provides()->addOperation("enable", &EcDriveModel::enable, this,
                                 RTT::OwnThread);
  this->provides()->addOperation("disable", &EcDriveModel::disable, this,
                                 RTT::OwnThread);
  this->provides()->addOperation("resetFault", &EcDriveModel::resetFault, this,
                                 RTT::OwnThread);
  this->provides()->addOperation("forceHomingDone",
                                 &EcDriveModel::forceHomingDone, this,
                                 RTT::OwnThread);

  m_factor_ = step_per_second_ * iteration_per_step_;
}

EcDriveModel::~EcDriveModel() {
}

RTT::Service::shared_ptr EcDriveModel::provides() {
  return service_;
}

void EcDriveModel::update() {
  if (RTT::NewData == port_motor_current_command_.read(desired_input_)) {
    desired_torque_ = desired_input_ * torque_constant_
        / input_current_multiplicator_;

    for (int iteration = 0; iteration < iteration_per_step_; iteration++) {
      effective_torque_ = desired_torque_ - motor_velocity_ * viscous_friction_;
      motor_acceleration_ = effective_torque_ / inertia_;
      motor_velocity_ += motor_acceleration_ / m_factor_;
      motor_position_ += motor_velocity_ / m_factor_;
      enc_motor_position_ = motor_position_ * enc_res_ / (2.0 * M_PI);
      enc_motor_velocity_ = motor_velocity_ * enc_res_ / (2.0 * M_PI);
    }
  }
  port_motor_position_.write(enc_motor_position_);
  port_motor_velocity_.write(enc_motor_velocity_);
  port_motor_current_.write(desired_input_);
}

bool EcDriveModel::enable() {
  if (state_ == SWITCH_ON) {
    state_ = OPERATION_ENABLED;
    enable_ = true;
  }
}

void EcDriveModel::disable() {
  if (enable_) {
    enable_ = false;
  }
}

bool EcDriveModel::beginHoming() {
  if (homing_done_ == false) {
    if (state_ == OPERATION_ENABLED) {
      homing_ = true;
      homing_done_ = true;
    }
  } else {
    RTT::log(RTT::Error) << "Drive not configured for homing" << RTT::endlog();
  }
  return homing_;
}

bool EcDriveModel::forceHomingDone() {
  if (state_ == OPERATION_ENABLED) {
    homing_done_ = true;
  }
  return homing_done_;
}

bool EcDriveModel::resetFault() {
  if (state_ == FAULT) {
    reset_fault_ = true;
    return true;
  } else {
    return false;
  }
}
