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
#include <vector>
#include <string>
#include "string_colors.h"

ECManager::ECManager(const std::string& name)
: TaskContext(name),
  robot_state_(NOT_OPERATIONAL),
  number_of_servos_(0),
  last_servo_synchro_(0),
  servos_state_changed_(0),
  auto_(false) {
  this->addProperty("hal_component_name", hal_component_name_).doc("");
  this->addProperty("scheme_component_name", scheme_component_name_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("autostart", autostart_).doc("");
  this->addProperty("fault_autoreset", fault_autoreset_).doc("");
  this->addProperty("services_names", services_names_).doc("");
  this->addProperty("regulators_names", regulators_names_).doc("");
  this->addOperation("auto", &ECManager::autoRun, this, RTT::OwnThread).doc("");
  this->addOperation("setSynchronized", &ECManager::setSynchronized, this, RTT::OwnThread).doc("");
  this->addOperation("resetFault", &ECManager::resetFaultAll, this, RTT::OwnThread).doc("");
  this->addOperation("disable", &ECManager::disableAll, this, RTT::OwnThread).doc("");
  this->addOperation("enable", &ECManager::enableAll, this, RTT::OwnThread).doc("");
  this->addOperation("beginHoming", &ECManager::beginHomingAll, this, RTT::OwnThread).doc("");
  this->addOperation("homingDone", &ECManager::homingDoneAll, this, RTT::OwnThread).doc("");
  this->addOperation("state", &ECManager::stateAll, this, RTT::OwnThread).doc("");
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
  for (int i = 0; i < number_of_servos_; i++) {
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
    stateAll();
  }

  auto_ = autostart_;

  return true;
}

void ECManager::updateHook() {
  switch (robot_state_) {
    case NOT_OPERATIONAL:

      for (int i = 0; i < number_of_servos_; i++) {
        if (servo_state_[i] != NOT_SYNCHRONIZED) {
          RTT::Attribute<ECServoState> * servo_ec_state = (RTT::Attribute<ECServoState> *) EC
              ->provides(services_names_[i])->getAttribute("state");
          ec_servo_state_ = servo_ec_state->get();

          if (auto_) {
            // set "enable" if powered on
            if (ec_servo_state_ == SWITCH_ON) {
              RTT::OperationCaller<bool(void)> enable;
              enable = EC->provides(services_names_[i])->getOperation("enable");
              enable.setCaller(this->engine());
              enable();
            }
          }

          // servo enabled
          if (ec_servo_state_ == OPERATION_ENABLED) {
            std::cout << services_names_[i] << ": ENABLED" << std::endl;
            servo_state_[i] = NOT_SYNCHRONIZED;
            ++servos_state_changed_;
          }
        }
      }

      // all servos enabled
      if (servos_state_changed_ == number_of_servos_) {
        robot_state_ = NOT_SYNCHRONIZED;
        std::cout << "ROBOT NOT SYNCHRONIZED" << std::endl;
        servos_state_changed_ = 0;
      }
      break;

    case NOT_SYNCHRONIZED:
      if (auto_) robot_state_ = SYNCHRONIZING;
      break;

    case SYNCHRONIZING:
      for (int i = 0; i < number_of_servos_; i++) {
        RTT::Attribute<ECServoState> * servo_state = (RTT::Attribute<ECServoState> *) EC
            ->provides(services_names_[i])->getAttribute("state");
        ec_servo_state_ = servo_state->get();

        if (ec_servo_state_ == OPERATION_ENABLED) {
          switch (servo_state_[i]) {
            case NOT_SYNCHRONIZED:
              if (i == last_servo_synchro_) {
                RTT::OperationCaller<bool(void)> beginHoming;
                beginHoming = EC->provides(services_names_[i])->getOperation("beginHoming");
                beginHoming.setCaller(this->engine());
                beginHoming();
                servo_state_[i] = SYNCHRONIZING;
                std::cout << services_names_[i] << ": SYNCHRONIZING" << std::endl;
              }
              break;
            case SYNCHRONIZING:
              RTT::Attribute<bool> * homing = (RTT::Attribute<bool> *) EC->provides(
                  services_names_[i])->getAttribute("homing_done");
              if (homing->get()) {
                servo_state_[i] = SYNCHRONIZED;
                std::cout << services_names_[i] << ": SYNCHRONIZED" << std::endl;
                last_servo_synchro_ = i+1;
                ++servos_state_changed_;

                // switch Regulator
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
                std::cout << regulators_names_[i] << ": ENABLED" << std::endl;
              }
              break;
          }
        }
      }
      // all servos synhronized
      if (servos_state_changed_ == number_of_servos_) {
        robot_state_ = SYNCHRONIZED;
        std::cout << "ROBOT SYNCHRONIZED" << std::endl;
        servos_state_changed_ = 0;
      }
      break;

    case SYNCHRONIZED:

      robot_state_ = RUNNING;
      std::cout << "ROBOT READY" << std::endl;
      break;

    case RUNNING:
      if (fault_autoreset_) {
        resetFaultAll();
        enableAll();
      }
      break;
    default:
      break;
  }
}

void ECManager::autoRun() {
  auto_ = true;
}

void ECManager::setSynchronized() {
  robot_state_ = SYNCHRONIZED;

  // switch Regulators
  for (int i = 0; i < number_of_servos_; i++) {
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
    std::cout << regulators_names_[i] << ": ENABLED" << std::endl;
  }
}

bool ECManager::resetFaultAll() {
  bool out = true;
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state = (RTT::Attribute<ECServoState> *) EC
        ->provides(services_names_[i])->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();

    // set "resetFault" if fault
    if (ec_servo_state_ == FAULT) {
      RTT::OperationCaller<bool(void)> resetFault;
      resetFault = EC->provides(services_names_[i])->getOperation("resetFault");
      resetFault.setCaller(this->engine());
      out = out && resetFault();
    }
  }
  return out;
}

bool ECManager::enableAll() {
  bool out = true;
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state = (RTT::Attribute<ECServoState> *) EC
        ->provides(services_names_[i])->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();

    // set "enable" if powered on
    if (ec_servo_state_ == SWITCH_ON) {
      RTT::OperationCaller<bool(void)> enable;
      enable = EC->provides(services_names_[i])->getOperation("enable");
      enable.setCaller(this->engine());
      out = out && enable();
    }
  }
  return out;
}

bool ECManager::disableAll() {
  bool out = true;
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state = (RTT::Attribute<ECServoState> *) EC
        ->provides(services_names_[i])->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();

    // set "disable" if powered on
    RTT::OperationCaller<bool(void)> disable;
    disable = EC->provides(services_names_[i])->getOperation("disable");
    disable.setCaller(this->engine());
    out = out && disable();
  }
  return out;
}

void ECManager::beginHomingAll() {
  if (robot_state_ == NOT_SYNCHRONIZED) robot_state_ = SYNCHRONIZING;
}

void ECManager::homingDoneAll() {
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<bool> * homing = (RTT::Attribute<bool> *) EC->provides(
        services_names_[i])->getAttribute("homing_done");
    std::cout << services_names_[i] << ": " << homing->get() << std::endl;
  }
}

void ECManager::stateAll() {
  for (int i = 0; i < number_of_servos_; i++) {
    RTT::Attribute<ECServoState> * servo_ec_state = (RTT::Attribute<ECServoState> *) EC
        ->provides(services_names_[i])->getAttribute("state");
    ec_servo_state_ = servo_ec_state->get();
    std::cout << services_names_[i] << ": " << state_text(ec_servo_state_) << std::endl;
  }
}

std::string ECManager::state_text(ECServoState state) {
  switch (state) {
    case INVALID:
      return "INVALID";
    case
    NOT_READY_TO_SWITCH_ON:
      return "NOT_READY_TO_SWITCH_ON";
    case
    SWITCH_ON_DISABLED:
      return "SWITCH_ON_DISABLED";
    case
    READY_TO_SWITCH_ON:
      return "READY_TO_SWITCH_ON";
    case
    SWITCH_ON:
      return "SWITCH_ON";
    case
    OPERATION_ENABLED:
      return "OPERATION_ENABLED";
    case
    QUICK_STOP_ACTIVE:
      return "QUICK_STOP_ACTIVE";
    case
    FAULT_REACTION_ACTIVE:
      return "FAULT_REACTION_ACTIVE";
    case
    FAULT:
      return "FAULT";
    default:
      return "";
  }
}

ORO_CREATE_COMPONENT(ECManager)
