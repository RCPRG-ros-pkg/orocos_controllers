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
 * InterrnalSpaceTrajectoryAction.cpp
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>
#include "rtt_rosclock/rtt_rosclock.h"

#include "InternalSpaceSplineTrajectoryAction.h"

InternalSpaceSplineTrajectoryAction::InternalSpaceSplineTrajectoryAction(
		const std::string& name) :
		RTT::TaskContext(name, PreOperational), numberOfJoints_prop(
				"number_of_joints", "", 0), command_port_("command") {
	// Add action server ports to this task's root service
	as.addPorts(this->provides());

	// Bind action server goal and cancel callbacks (see below)
	as.registerGoalCallback(
			boost::bind(&InternalSpaceSplineTrajectoryAction::goalCB, this,
					_1));
	as.registerCancelCallback(
			boost::bind(&InternalSpaceSplineTrajectoryAction::cancelCB, this,
					_1));

	this->addPort("trajectoryPtr", trajectory_ptr_port);
	this->addPort("JointPosition", port_joint_position_);
	this->addEventPort(command_port_,
			boost::bind(&InternalSpaceSplineTrajectoryAction::commandCB, this));
	this->addProperty("joint_names", jointNames);
}

InternalSpaceSplineTrajectoryAction::~InternalSpaceSplineTrajectoryAction() {

}

bool InternalSpaceSplineTrajectoryAction::configureHook() {
	if (jointNames.size() <= 0) {
		return false;
	}

	numberOfJoints = jointNames.size();

	return true;
}

bool InternalSpaceSplineTrajectoryAction::startHook() {
	as.start();
	goal_active = false;
	enable = true;
	return true;
}

void InternalSpaceSplineTrajectoryAction::updateHook() {

	if (port_joint_position_.read(joint_position_) == RTT::NoData) {

	}

	if (goal_active) {
		//	std::cout << "blabla: " << std::endl;
		ros::Time now = rtt_rosclock::host_now();

		if (now > trajectory_finish_time)

		{
			//	std::cout << "aaaaaa: " << std::endl;
			activeGoal.setSucceeded();
			goal_active = false;
		}

	}

	//std::cout << "aqq: " << joint_position_ << std::endl;

//	RTT::Logger::log(RTT::Logger::Error) << "aqq: " << joint_position_ << RTT::endlog();

// ma sprawdzic czy juz zakonyczl wykonywanie trajektorii - patrzymy czy pozycja zmierzona w stawach jest wystarczajaco blisko pozycji zadanej
// dodac port ze zmierzona pozycja

//jesli zakonczyl interpolacje i jest w pozycji zmierzonej to activeGoal.setsucceded
// http://docs.ros.org/hydro/api/actionlib/html/classactionlib_1_1ServerGoalHandle.html

// jesli nie to nic narazie
}

void InternalSpaceSplineTrajectoryAction::goalCB(GoalHandle gh) {
	if (!goal_active) {
		std::vector<int> remapTable;
		remapTable.resize(numberOfJoints);
		trajectory_msgs::JointTrajectory* trj_ptr =
				new trajectory_msgs::JointTrajectory;
		Goal g = gh.getGoal();

		RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contain "
				<< g->trajectory.points.size() << " points" << RTT::endlog();

		// fill remap table
		for (unsigned int i = 0; i < numberOfJoints; i++) {
			int jointId = -1;
			for (unsigned int j = 0; j < g->trajectory.joint_names.size();
					j++) {
				if (g->trajectory.joint_names[j] == jointNames[i]) {
					jointId = j;
					break;
				}
			}
			if (jointId < 0) {
				RTT::Logger::log(RTT::Logger::Error)
						<< "Trajectory contains invalid joint" << RTT::endlog();
				gh.setRejected();
				return;
			} else {
				remapTable[i] = jointId;
			}

		}

		//remap joints

		trj_ptr->header = g->trajectory.header;
		trj_ptr->points.resize(g->trajectory.points.size());

		for (unsigned int i = 0; i < g->trajectory.points.size(); i++) {
			trj_ptr->points[i].positions.resize(
					g->trajectory.points[i].positions.size());
			for (unsigned int j = 0;
					j < g->trajectory.points[i].positions.size(); j++) {
				trj_ptr->points[i].positions[j] =
						g->trajectory.points[i].positions[remapTable[j]];
			}

			trj_ptr->points[i].velocities.resize(
					g->trajectory.points[i].velocities.size());
			for (unsigned int j = 0;
					j < g->trajectory.points[i].velocities.size(); j++) {
				trj_ptr->points[i].velocities[j] =
						g->trajectory.points[i].velocities[remapTable[j]];
			}

			trj_ptr->points[i].accelerations.resize(
					g->trajectory.points[i].accelerations.size());
			for (unsigned int j = 0;
					j < g->trajectory.points[i].accelerations.size(); j++) {
				trj_ptr->points[i].accelerations[j] =
						g->trajectory.points[i].accelerations[remapTable[j]];
			}

			trj_ptr->points[i].time_from_start =
					g->trajectory.points[i].time_from_start;

		}

		trajectory_finish_time =
				g->trajectory.header.stamp
						+ g->trajectory.points[g->trajectory.points.size() - 1].time_from_start;

		activeGoal = gh;
		goal_active = true;

		bool ok = true;

		RTT::TaskContext::PeerList peers = this->getPeerList();
		for (size_t i = 0; i < peers.size(); i++) {
			RTT::Logger::log(RTT::Logger::Debug) << "Starting peer : "
					<< peers[i] << RTT::endlog();
			ok = ok && this->getPeer(peers[i])->start();
		}

		if (ok) {
			trajectory_msgs::JointTrajectoryConstPtr trj_cptr =
					trajectory_msgs::JointTrajectoryConstPtr(trj_ptr);

			trajectory_ptr_port.write(trj_cptr);

			gh.setAccepted();
			goal_active = true;
		} else {
			gh.setRejected();
			goal_active = false;
		}
	} else {
		gh.setRejected();
	}
}

void InternalSpaceSplineTrajectoryAction::cancelCB(GoalHandle gh) {
	goal_active = false;
}

void InternalSpaceSplineTrajectoryAction::commandCB() {

}

ORO_CREATE_COMPONENT( InternalSpaceSplineTrajectoryAction )
