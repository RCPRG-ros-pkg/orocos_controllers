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
#include "oro_servo_msgs/ServoStates.h"
#include "oro_servo_msgs/Setpoints.h"

class JointSplineTrajectoryGenerator : public RTT::TaskContext {
public:
	JointSplineTrajectoryGenerator(const std::string& name);
	virtual ~JointSplineTrajectoryGenerator();

	bool configureHook();
	bool startHook();
	void updateHook();

protected:
	RTT::InputPort<trajectory_msgs::JointTrajectoryPoint> trajectoryPoint_port;
	RTT::OutputPort<bool> bufferReady_port;

	RTT::OutputPort<oro_servo_msgs::Setpoints> setpoint_port;
	RTT::InputPort<oro_servo_msgs::ServoStates> jointState_port;

	RTT::Property<int> numberOfJoints_prop;
private:

	std::vector<KDL::VelocityProfile_Spline> velProfile_;

	trajectory_msgs::JointTrajectoryPoint trajectoryOld;
	trajectory_msgs::JointTrajectoryPoint trajectoryNew;

  oro_servo_msgs::Setpoints setpoint_;

	unsigned int numberOfJoints;
	bool trajectoryReady;
	bool bufferReady;

	long long int time;
	long long int endTime;
	double dt;
};

#endif /* JOINTSPLINETRAJECTORYGENERATOR_H_ */
