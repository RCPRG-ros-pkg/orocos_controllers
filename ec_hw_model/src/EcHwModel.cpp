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

#include <rtt/Component.hpp>
#include <string>

#include "EcHwModel.h"
#include "common_headers/string_colors.h"

EcHwModel::EcHwModel(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      number_of_servos_(0) {

  this->addProperty("services_names", services_names_).doc("");
  this->addProperty("iteration_per_step", iteration_per_step_);
  this->addProperty("step_per_second", step_per_second_);
  this->addProperty("torque_constant", torque_constant_);
  this->addProperty("input_current_multiplicator",
                    input_current_multiplicator_);
  this->addProperty("inertia", inertia_);
  this->addProperty("viscous_friction", viscous_friction_);
  this->addProperty("enc_res", enc_res_).doc("");
}

EcHwModel::~EcHwModel() {
}

bool EcHwModel::configureHook() {
  number_of_servos_ = torque_constant_.size();
  if ((number_of_servos_ != input_current_multiplicator_.size())
      || (number_of_servos_ != services_names_.size())
      || (number_of_servos_ != inertia_.size())
      || (number_of_servos_ != viscous_friction_.size())) {
    std::cout << std::endl << RED << "[error] hardware model " << getName()
        << "configuration failed: wrong properties vector length in launch file."
        << RESET << std::endl;
    return false;
  }

  drives_.resize(number_of_servos_);

  // dodanie do listy drive wszystkich symulowanych napędów
  for (int i = 0; i < number_of_servos_; i++) {
    EcDriveModel::Ptr drive(
        new EcDriveModel(services_names_[i], iteration_per_step_,
                         step_per_second_, enc_res_[i], torque_constant_[i],
                         input_current_multiplicator_[i], inertia_[i],
                         viscous_friction_[i]));
    drives_[i] = drive;
    this->provides()->addService(drive->provides());
  }

  return true;
}

void EcHwModel::updateHook() {
  for (int i = 0; i < number_of_servos_; i++) {
    drives_[i]->update();
  }
}

ORO_CREATE_COMPONENT(EcHwModel)
