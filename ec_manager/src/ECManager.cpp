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

#include "ECManager.h"
#include "string_colors.h"
#include <vector>
#include <string>

ECManager::ECManager(const std::string& name)
: TaskContext(name),
  robot_state_(NOT_OPERATIONAL),
  number_of_servos_(0),
  last_servo_synchro_(0),
  servos_state_changed_(0){
  this->addProperty("hal_component_name", hal_component_name_).doc("");
  this->addProperty("scheme_component_name", scheme_component_name_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("sequent_synchro", sequent_synchro_).doc("");
  this->addProperty("fault_autoreset", fault_autoreset_).doc("");
  this->addProperty("services_names", services_names_).doc("");
  this->addProperty("regulators_names", regulators_names_).doc("");
  this->addOperation("resetFault", &ECManager::resetF, this, RTT::OwnThread).doc("");;
}

ECManager::~ECManager() {
}

bool ECManager::configureHook() {
  if (hal_component_name_.empty() || scheme_component_name_.empty()) {
    return false;
  }

  number_of_servos_ = services_names_.size();
  if (number_of_servos_ != regulators_names_.size()) {
    std::cout << std::endl << RED << "[error] EC Manager "
        << "configuration failed: wrong properties vector length in launch file."
        << RESET << std::endl;
    return false;
  }
  if (debug_) {
    std::cout << "servos: " << number_of_servos_ << std::endl;
  }

  servo_state_.resize(number_of_servos_);
  for (int i=0; i < number_of_servos_; i++)
  {
    servo_state_[i] = NOT_OPERATIONAL;
  }

  return true;
}

bool ECManager::startHook() {
  EC = RTT::TaskContext::getPeer(hal_component_name_);
  Scheme = RTT::TaskContext::getPeer(scheme_component_name_);

  if (debug_) {
    std::cout << "EC_active:" << EC->isActive() << std::endl;
    std::cout << "EC_running:" << EC->isRunning() << std::endl;
    std::cout << "Scheme_active:" << Scheme->isActive() << std::endl;
    std::cout << "Scheme_running:" << Scheme->isRunning() << std::endl;
  }

  return true;
}

void ECManager::updateHook() {

  switch (robot_state_) {

    case NOT_OPERATIONAL:
      for (int i = 0; i < number_of_servos_; i++)
      {
        RTT::Attribute<ECServoState> * servo_ec_state = (RTT::Attribute<ECServoState> *) EC
            ->provides(services_names_[i])->getAttribute("state");
        ec_servo_state_ = servo_ec_state->get();

        // set "enable" if powered on
        if (ec_servo_state_ == SWITCH_ON)
        {
          RTT::OperationCaller<bool(void)> enable;
          enable = EC->provides(services_names_[i])->getOperation("enable");
          enable.setCaller(this->engine());
          enable();
        }

        // servo enable
        if (ec_servo_state_ == OPERATION_ENABLED)
        {
          servo_state_[i] = NOT_SYNCHRONIZED;
          ++servos_state_changed_;
        }
      }

      // all servos enabled
      if (servos_state_changed_ == number_of_servos_)
      {
        robot_state_ = NOT_SYNCHRONIZED;
        servos_state_changed_ = 0;
      }
      break;

    case NOT_SYNCHRONIZED:
      for (int i = 0; i < number_of_servos_; i++)
      {
        RTT::Attribute<ECServoState> * servo_state = (RTT::Attribute<ECServoState> *) EC
            ->provides(services_names_[i])->getAttribute("state");
        ec_servo_state_ = servo_state->get();

        if (ec_servo_state_ == OPERATION_ENABLED)
        {
          switch (servo_state_[i]) {
            case NOT_SYNCHRONIZED:
              if (sequent_synchro_)
              {
                // next servo in line
                if (i == last_servo_synchro_)
                {
                  RTT::OperationCaller<bool(void)> beginHoming;
                  beginHoming = EC->provides(services_names_[i])->getOperation("beginHoming");
                  beginHoming.setCaller(this->engine());
                  beginHoming();
                  servo_state_[i] = SYNCHRONIZING;
                }
              }
              else
              {
                RTT::OperationCaller<bool(void)> beginHoming;
                beginHoming = EC->provides(services_names_[i])->getOperation("beginHoming");
                beginHoming.setCaller(this->engine());
                beginHoming();
                servo_state_[i] = SYNCHRONIZING;
              }
              break;
            case SYNCHRONIZING:
              RTT::Attribute<State> * homing = (RTT::Attribute<State> *) EC->provides(
                  services_names_[i])->getAttribute("homing_done");
              if (homing->get())
              {
                servo_state_[i] = SYNCHRONIZED;
                last_servo_synchro_ = i+1;
                ++servos_state_changed_;
              }
              break;
          }
        }
      }
      // all servos synhronized
      if (servos_state_changed_ == number_of_servos_)
      {
        robot_state_ = SYNCHRONIZED;
        servos_state_changed_ = 0;
      }
      break;

    case SYNCHRONIZED:
      for (int i = 0; i < number_of_servos_; i++)
      {
        disable_vec_.clear();
        enable_vec_.clear();
        enable_vec_.push_back(regulators_names_[i]);
        RTT::OperationCaller<
        bool(const std::vector<std::string> &disable_block_names,
             const std::vector<std::string> &enable_block_names,
             const bool strict, const bool force)> switchBlocks;
        switchBlocks = Scheme->getOperation("switchBlocks");
        switchBlocks.setCaller(this->engine());
        switchBlocks(disable_vec_, enable_vec_, true, true);
      }
      robot_state_ = RUNNING;
      break;

    case RUNNING:
      if (fault_autoreset_)
      {
        for (int i = 0; i < number_of_servos_; i++)
        {
          RTT::OperationCaller<bool(void)> resetFault;
          resetFault = EC->provides(services_names_[i])->getOperation("resetFault");
          resetFault.setCaller(this->engine());
          resetFault();
        }
      }
      break;
    default:
      break;
  }
}

void ECManager::resetF() {
  for (int i = 0; i < number_of_servos_; i++)
  {
    RTT::OperationCaller<bool(void)> resetFault;
    resetFault = EC->provides(services_names_[i])->getOperation("resetFault");
    resetFault.setCaller(this->engine());
    resetFault();
  }
}

ORO_CREATE_COMPONENT(ECManager)
