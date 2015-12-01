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

#include "IM2RM.h"

#include <rtt/Component.hpp>
#include <string>

IM2RM::IM2RM(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("RadianMotorPosition", port_radian_motor_position_);
  this->ports()->addPort("IncrementalMotorPosition",
                         port_incremental_motor_position_);

  this->addProperty("enc_res", enc_res_).doc("");
  this->addProperty("number_of_servos", number_of_servos_).doc("");
}

IM2RM::~IM2RM() {
}

bool IM2RM::configureHook() {
  incremental_motor_position_.resize(number_of_servos_);
  radian_motor_position_.resize(number_of_servos_);
  return true;
}

void IM2RM::updateHook() {
  if (RTT::NewData
      == port_incremental_motor_position_.read(incremental_motor_position_)) {
    for (int i = 0; i < number_of_servos_; i++) {
      radian_motor_position_[i] = incremental_motor_position_[i] * (2.0 * M_PI)
          / enc_res_;
    }
    port_radian_motor_position_.write(radian_motor_position_);
  }
}

ORO_CREATE_COMPONENT(IM2RM)

