/*
 * Copyright (c) 2010-2015 Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * LimitDetector.h
 *
 *  Created on: 2015
 *      Author: twiniars
 */

#ifndef LIMITDETECTOR_H_
#define LIMITDETECTOR_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

class LimitDetector : public RTT::TaskContext {
 public:
  explicit LimitDetector(const std::string& name);
  virtual ~LimitDetector();

  bool configureHook();
  void updateHook();

 protected:
  RTT::InputPort<Eigen::VectorXd> input_port_;
  RTT::OutputPort<Eigen::VectorXd> output_port_;
  RTT::OutputPort<bool> emergency_stop_out_;

 private:
  unsigned int number_of_ports_;
  bool pos_inc_initiated_;

  Eigen::VectorXd previous_pos_;
  Eigen::VectorXd current_pos_;
  Eigen::VectorXd pos_inc_;

  // Properties
  std::vector<double> upper_pos_limit_;
  std::vector<double> lower_pos_limit_;
  std::vector<double> pos_inc_limit_;
  std::vector<bool> pos_limit_active_;
  std::vector<bool> pos_inc_limit_active_;
  std::string detector_name_;
};

#endif  // LIMITDETECTOR_H_
