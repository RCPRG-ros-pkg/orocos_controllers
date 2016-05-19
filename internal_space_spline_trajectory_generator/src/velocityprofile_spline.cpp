/*
 * Copyright (c) 2010-2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

#include <limits>

#include "velocityprofile_spline.hpp"

namespace KDL {

static inline void generatePowers(int n, double x, double* powers) {
  powers[0] = 1.0;
  for (int i = 1; i <= n; i++) {
    powers[i] = powers[i - 1] * x;
  }
  return;
}

VelocityProfile_Spline::VelocityProfile_Spline() {
  duration_ = 0.0;

  coeff_[0] = 0.0;
  coeff_[1] = 0.0;
  coeff_[2] = 0.0;
  coeff_[3] = 0.0;
  coeff_[4] = 0.0;
  coeff_[5] = 0.0;

  return;
}

VelocityProfile_Spline::VelocityProfile_Spline(
    const VelocityProfile_Spline &p) {
  duration_ = p.duration_;

  coeff_[0] = p.coeff_[0];
  coeff_[1] = p.coeff_[1];
  coeff_[2] = p.coeff_[2];
  coeff_[3] = p.coeff_[3];
  coeff_[4] = p.coeff_[4];
  coeff_[5] = p.coeff_[5];

  return;
}

VelocityProfile_Spline::~VelocityProfile_Spline() {
  return;
}

void VelocityProfile_Spline::SetProfile(double pos1, double pos2) {
  return;
}

void VelocityProfile_Spline::SetProfileDuration(double pos1, double pos2,
                                                double duration) {
  duration_ = duration;
  if (duration <= std::numeric_limits<double>::epsilon()) {
    coeff_[0] = pos1;
    coeff_[1] = 0.0;
    coeff_[2] = 0.0;
    coeff_[3] = 0.0;
    coeff_[4] = 0.0;
    coeff_[5] = 0.0;
  } else {
    coeff_[0] = pos1;
    coeff_[1] = (pos2 - pos1) / duration;
    coeff_[2] = 0.0;
    coeff_[3] = 0.0;
    coeff_[4] = 0.0;
    coeff_[5] = 0.0;
  }

  return;
}

void VelocityProfile_Spline::SetProfileDuration(double pos1, double vel1,
                                                double pos2, double vel2,
                                                double duration) {
  double T[4];
  duration_ = duration;
  generatePowers(3, duration, T);

  if (duration <= std::numeric_limits<double>::epsilon()) {
    coeff_[0] = pos2;
    coeff_[1] = vel2;
    coeff_[2] = 0.0;
    coeff_[3] = 0.0;
    coeff_[4] = 0.0;
    coeff_[5] = 0.0;
  } else {
    coeff_[0] = pos1;
    coeff_[1] = vel1;
    coeff_[2] = (-3.0 * pos1 + 3.0 * pos2 - 2.0 * vel1 * T[1] - vel2 * T[1])
        / T[2];
    coeff_[3] = (2.0 * pos1 - 2.0 * pos2 + vel1 * T[1] + vel2 * T[1]) / T[3];
    coeff_[4] = 0.0;
    coeff_[5] = 0.0;
  }

  return;
}

void VelocityProfile_Spline::SetProfileDuration(double pos1, double vel1,
                                                double acc1, double pos2,
                                                double vel2, double acc2,
                                                double duration) {
  double T[6];
  generatePowers(5, duration, T);

  if (duration <= std::numeric_limits<double>::epsilon()) {
    coeff_[0] = pos2;
    coeff_[1] = vel2;
    coeff_[2] = 0.5 * acc2;
    coeff_[3] = 0.0;
    coeff_[4] = 0.0;
    coeff_[5] = 0.0;
  } else {
    coeff_[0] = pos1;
    coeff_[1] = vel1;
    coeff_[2] = 0.5 * acc1;
    coeff_[3] = (-20.0 * pos1 + 20.0 * pos2 - 3.0 * acc1 * T[2] + acc2 * T[2]
        - 12.0 * vel1 * T[1] - 8.0 * vel2 * T[1]) / (2.0 * T[3]);
    coeff_[4] = (30.0 * pos1 - 30.0 * pos2 + 3.0 * acc1 * T[2]
        - 2.0 * acc2 * T[2] + 16.0 * vel1 * T[1] + 14.0 * vel2 * T[1])
        / (2.0 * T[4]);
    coeff_[5] = (-12.0 * pos1 + 12.0 * pos2 - acc1 * T[2] + acc2 * T[2]
        - 6.0 * vel1 * T[1] - 6.0 * vel2 * T[1]) / (2.0 * T[5]);
  }

  return;
}

double VelocityProfile_Spline::Duration() const {
  return duration_;
}

double VelocityProfile_Spline::Pos(double time) const {
  double t[6];
  double position;
  generatePowers(5, time, t);

  position = t[0] * coeff_[0] + t[1] * coeff_[1] + t[2] * coeff_[2]
      + t[3] * coeff_[3] + t[4] * coeff_[4] + t[5] * coeff_[5];
  return position;
}

double VelocityProfile_Spline::Vel(double time) const {
  double t[5];
  double velocity;
  generatePowers(4, time, t);

  velocity = t[0] * coeff_[1] + 2.0 * t[1] * coeff_[2] + 3.0 * t[2] * coeff_[3]
      + 4.0 * t[3] * coeff_[4] + 5.0 * t[4] * coeff_[5];
  return velocity;
}

double VelocityProfile_Spline::Acc(double time) const {
  double t[4];
  double acceleration;
  generatePowers(3, time, t);

  acceleration = 2.0 * t[0] * coeff_[2] + 6.0 * t[1] * coeff_[3]
      + 12.0 * t[2] * coeff_[4] + 20.0 * t[3] * coeff_[5];
  return acceleration;
}

void VelocityProfile_Spline::Write(std::ostream& os) const {
  os << "coefficients : [ " << coeff_[0] << " " << coeff_[1] << " " << coeff_[2]
      << " " << coeff_[3] << " " << coeff_[4] << " " << coeff_[5] << " ]";
  return;
}

VelocityProfile* VelocityProfile_Spline::Clone() const {
  return new VelocityProfile_Spline(*this);
}
}  // namespace KDL

