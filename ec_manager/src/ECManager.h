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

#ifndef ECMANAGER_H_
#define ECMANAGER_H_

#include <std_msgs/Bool.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/SendHandle.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/base/AttributeBase.hpp>

#include <rtt/extras/SlaveActivity.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>

typedef enum {
  NOT_OPERATIONAL,
  NOT_SYNCHRONIZED,
  SYNCHRONIZING,
  SYNCHRONIZED,
  RUNNING
} State;

typedef enum {
  PROFILE_POSITION = 1,
  PROFILE_VELOCITY = 2,
  PROFILE_CURRENT = 3,
  HOMING = 6,
  CYCLIC_CURRENT = 10,
  CYCLIC_VELOCITY = 9,
  CYCLIC_POSITION = 8
} ControlMode;

typedef enum {
  INVALID = 0,
  NOT_READY_TO_SWITCH_ON = 1,
  SWITCH_ON_DISABLED = 2,
  READY_TO_SWITCH_ON = 3,
  SWITCH_ON = 4,
  OPERATION_ENABLED = 5,
  QUICK_STOP_ACTIVE = 6,
  FAULT_REACTION_ACTIVE = 7,
  FAULT = 8
} ECServoState;

class ECManager : public RTT::TaskContext {
 private:
  TaskContext * EC;
  TaskContext * Scheme;

  ControlMode control_mode_;

  State robot_state_;
  std::vector<State> servo_state_;

  ECServoState ec_servo_state_;

  int number_of_servos_;
  int last_servo_synchro_;
  int servos_state_changed_;
  std::vector<std::string> disable_vec_;
  std::vector<std::string> enable_vec_;

  // Properties
  bool debug_;
  bool auto_;
  bool autostart_;
  bool fault_autoreset_;
  std::string hal_component_name_;
  std::string scheme_component_name_;
  std::vector<std::string> services_names_;
  std::vector<std::string> regulators_names_;

 public:
  explicit ECManager(const std::string& name);
  ~ECManager();

  bool configureHook();
  bool startHook();
  void updateHook();

  void autoRun();
  void setSynchronized();

  bool resetFaultAll();
  bool enableAll();
  bool disableAll();
  void beginHomingAll();
  void homingDoneAll();
  void stateAll();

  std::string state_text(ECServoState state);
};

#endif  // ECMANAGER_H_
