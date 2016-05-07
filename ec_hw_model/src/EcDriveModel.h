/*
 * Copyright (c) 2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

#ifndef ECDRIVEMODEL_H_
#define ECDRIVEMODEL_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <rtt/Port.hpp>
#include <rtt/Service.hpp>

#include <string>

class EcDriveModel {
 public:
  enum ControlMode {
    PROFILE_POSITION = 1,
    PROFILE_VELOCITY = 2,
    PROFILE_CURRENT = 3,
    HOMING = 6,
    CYCLIC_CURRENT = 10,
    CYCLIC_VELOCITY = 9,
    CYCLIC_POSITION = 8
  };
  enum ServoState {
    INVALID = 0,
    NOT_READY_TO_SWITCH_ON = 1,
    SWITCH_ON_DISABLED = 2,
    READY_TO_SWITCH_ON = 3,
    SWITCH_ON = 4,
    OPERATION_ENABLED = 5,
    QUICK_STOP_ACTIVE = 6,
    FAULT_REACTION_ACTIVE = 7,
    FAULT = 8
  };

  typedef boost::shared_ptr<EcDriveModel> Ptr;

  explicit EcDriveModel(const std::string &name, int iteration_per_step,
                        int step_per_second, double enc_res,
                        double torque_constant,
                        double input_current_multiplicator, double inertia,
                        double viscous_friction);
  ~EcDriveModel();

  RTT::Service::shared_ptr provides();

  void update();

 private:
  RTT::InputPort<double> port_motor_position_command_;
  RTT::InputPort<double> port_motor_velocity_command_;
  RTT::InputPort<double> port_motor_current_command_;
  RTT::OutputPort<double> port_motor_position_;
  RTT::OutputPort<double> port_motor_velocity_;
  RTT::OutputPort<double> port_motor_current_;

  bool enable();
  void disable();
  bool beginHoming();
  bool resetFault();
  bool forceHomingDone();

 protected:
  RTT::Service::shared_ptr service_;
  double enc_motor_position_, enc_motor_velocity_, motor_position_,
      motor_velocity_, motor_acceleration_, desired_input_, desired_torque_,
      effective_torque_;

  int iteration_per_step_, step_per_second_;
  int m_factor_;
  double enc_res_, torque_constant_, input_current_multiplicator_, inertia_,
      viscous_friction_;

  ControlMode control_mode_;
  ServoState state_;
  bool enable_;
  bool homing_;
  bool reset_fault_;
  bool homing_done_;
};

#endif  // ECDRIVEMODEL_H_
