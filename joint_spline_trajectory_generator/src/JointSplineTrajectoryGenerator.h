/*
 * Copyright (c) 2010, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * JointSplineTrajectoryGenerator.h
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#ifndef JOINTSPLINETRAJECTORYGENERATOR_H_
#define JOINTSPLINETRAJECTORYGENERATOR_H_

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include "velocityprofile_spline.hpp"

#include <trajectory_msgs/JointTrajectoryPoint.h>

class JointSplineTrajectoryGenerator : public RTT::TaskContext {
public:
	JointSplineTrajectoryGenerator(const std::string& name);
	virtual ~JointSplineTrajectoryGenerator();

	bool configureHook();
	bool startHook();
	void updateHook();

protected:
	RTT::InputPort<trajectory_msgs::JointTrajectoryPoint> trajectory_point_port_;
	RTT::OutputPort<bool> buffer_ready_port_;

	RTT::OutputPort<std::vector<double> > jnt_pos_port_;
	RTT::InputPort<std::vector<double> > cmd_jnt_pos_port_;

	RTT::OutputPort<bool> trajectory_compleat_port_;

	RTT::Property<int> number_of_joints_prop_;
private:

	std::vector<KDL::VelocityProfile_Spline> vel_profile_;

	trajectory_msgs::JointTrajectoryPoint trajectory_old_;
	trajectory_msgs::JointTrajectoryPoint trajectory_new_;

	std::vector<double> des_jnt_pos_;

	unsigned int number_of_joints_;
	bool trajectory_ready_;
	bool buffer_ready_;

	int64_t time_;
	int64_t end_time_;
	double dt_;
};

#endif /* JOINTSPLINETRAJECTORYGENERATOR_H_ */
