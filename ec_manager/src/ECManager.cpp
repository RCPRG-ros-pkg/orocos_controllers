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

ECManager::ECManager(const std::string& name)
    : TaskContext(name),
      state_(NOT_SYNCHRONIZED) {
  this->addProperty("hal_component_name", hal_component_name_).doc("");
  this->addProperty("scheme_component_name", scheme_component_name_).doc("");
  this->addProperty("debug", debug_).doc("");
  this->addProperty("service", service_).doc("");
  this->addProperty("regulator", regulator_).doc("");
}

ECManager::~ECManager() {
}

bool ECManager::configureHook() {
  if (hal_component_name_.empty() || scheme_component_name_.empty()) {
    return false;
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

    std::cout << EC->provides(service_) << std::endl;
    std::vector < std::string > att_vec = EC->provides(service_)
        ->getAttributeNames();
    std::vector < std::string > ope_vec = EC->provides(service_)
        ->getOperationNames();

    for (int i = 0; i < ope_vec.size(); i++) {
      std::cout << service_ << "." << ope_vec[i] << std::endl;
    }
    for (int i = 0; i < att_vec.size(); i++) {
      std::cout << service_ << "." << att_vec[i] << std::endl;
    }
  }

  return true;
}

void ECManager::updateHook() {
  RTT::Attribute<ServoState> * servo_state = (RTT::Attribute<ServoState> *) EC
      ->provides(service_)->getAttribute("state");
  RTT::Attribute<State> * homing = (RTT::Attribute<State> *) EC->provides(
      service_)->getAttribute("homing_done");

  servo_state_ = servo_state->get();

  if (debug_) {
    std::cout << service_ << ".state = " << servo_state_ << std::endl;
    std::cout << service_ << ".homing_done = " << homing->get() << std::endl;
  }

  RTT::OperationCaller<bool(void)> enable;
  RTT::OperationCaller<bool(void)> resetFault;
  RTT::OperationCaller<bool(void)> beginHoming;
  RTT::OperationCaller<
      bool(const std::vector<std::string> &disable_block_names,
           const std::vector<std::string> &enable_block_names,
           const bool strict, const bool force)> switchBlocks;

  switch (servo_state_) {
    case INVALID:
      break;

    case NOT_READY_TO_SWITCH_ON:
      break;

    case SWITCH_ON_DISABLED:
      break;

    case READY_TO_SWITCH_ON:
      break;

    case SWITCH_ON:
      enable = EC->provides(service_)->getOperation("enable");
      enable.setCaller(this->engine());
      enable();
      break;

    case OPERATION_ENABLED:

      switch (state_) {
        case NOT_SYNCHRONIZED:
          beginHoming = EC->provides(service_)->getOperation("beginHoming");
          beginHoming.setCaller(this->engine());
          beginHoming();
          state_ = SYNCHRONIZING;
          break;
        case SYNCHRONIZING:
          if (homing->get())
            state_ = SYNCHRONIZED;
          break;
        case SYNCHRONIZED:
          disable_vec_.clear();
          enable_vec_.clear();
          enable_vec_.push_back(regulator_);
          switchBlocks = Scheme->getOperation("switchBlocks");
          switchBlocks.setCaller(this->engine());
          switchBlocks(disable_vec_, enable_vec_, true, true);
          state_ = RUNNING;
          break;
        case RUNNING:
          break;
        default:
          break;
      }

      break;

    case QUICK_STOP_ACTIVE:
      break;

    case FAULT_REACTION_ACTIVE:
      break;

    case FAULT:
      resetFault = EC->provides(service_)->getOperation("resetFault");
      resetFault.setCaller(this->engine());
      resetFault();
      break;

    default:
      break;
  }
}

ORO_CREATE_COMPONENT(ECManager)
