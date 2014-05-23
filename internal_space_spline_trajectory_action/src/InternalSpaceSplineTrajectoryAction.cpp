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
#include "eigen_conversions/eigen_msg.h"

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
	this->addPort("JointPositionCommand", port_joint_position_command_);
	this->addEventPort(command_port_,
			boost::bind(&InternalSpaceSplineTrajectoryAction::commandCB, this));
	this->addProperty("joint_names", jointNames);
	this->addProperty("lower_limits", lowerLimits);
	this->addProperty("upper_limits", upperLimits);
}

InternalSpaceSplineTrajectoryAction::~InternalSpaceSplineTrajectoryAction() {

}

bool InternalSpaceSplineTrajectoryAction::configureHook() {
	if (jointNames.size() <= 0) {
		return false;
	}

	numberOfJoints = jointNames.size();

	feedback.actual.positions.reserve(numberOfJoints);
	feedback.desired.positions.reserve(numberOfJoints);
	feedback.error.positions.reserve(numberOfJoints);

	feedback.joint_names.reserve(numberOfJoints);
	for (int i = 0; i < jointNames.size(); i++) {
		feedback.joint_names.push_back(jointNames[i]);
	}

	remapTable.resize(numberOfJoints);

	if (lowerLimits.size() != numberOfJoints
			|| upperLimits.size() != numberOfJoints) {
		std::cout << "Limits not loaded" << std::endl;
		return false;
	}

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

	control_msgs::FollowJointTrajectoryResult res;

	port_joint_position_command_.read(desired_joint_position_);

	Goal g = activeGoal.getGoal();
	if (goal_active) {
		ros::Time now = rtt_rosclock::host_rt_now();

		if (now > trajectory_finish_time) {

			//sprawdzenie pozycji, jesli ok to wysyłamy SUCCESSFUL jeśli nie to po goal_time_tolerance wysyłamy GOAL_TOLERANCE_VIOLATED
			bool violated = false;
			for (int i = 0; i < numberOfJoints; i++) {
				for (int j = 0; j < g->goal_tolerance.size(); j++) {
					if (g->goal_tolerance[j].name
							== g->trajectory.joint_names[i]) {
						//jeśli istnieje ograniczenie to sprawdzam pozycję
						if (joint_position_[remapTable[i]]
								+ g->goal_tolerance[j].position
								< g->trajectory.points[g->trajectory.points.size()
										- 1].positions[i]
								|| joint_position_[remapTable[i]]
										- g->goal_tolerance[j].position
										> g->trajectory.points[g->trajectory.points.size()
												- 1].positions[i]) {
							violated = true;
							RTT::Logger::log(RTT::Logger::Debug)
									<< g->goal_tolerance[j].name
									<< " violated with position "
									<< joint_position_[remapTable[i]]
									<< RTT::endlog();
						}

					}

				}

			}

			if (violated
					&& now.toNSec()
							> trajectory_finish_time.toNSec()
									+ g->goal_time_tolerance.toNSec()) {
				res.error_code =
						control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
				activeGoal.setAborted(res, "");
				goal_active = false;
			} else if (!violated) {
				res.error_code =
						control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
				activeGoal.setSucceeded(res, "");
				goal_active = false;
			}

		}

		//wysyłanie feedback
		Eigen::VectorXd error = joint_position_ - desired_joint_position_;
		for (int i = 0; i < numberOfJoints; i++) {
			feedback.actual.positions.push_back((double) joint_position_[i]);
			feedback.desired.positions.push_back(
					(double) desired_joint_position_[i]);
			feedback.error.positions.push_back((double) error[i]);
		}

		feedback.header.stamp = rtt_rosclock::host_rt_now();
		activeGoal.publishFeedback(feedback);

		//sprawdzanie PATH_TOLRANCE_VIOLATED
		for (int i = 0; i < g->path_tolerance.size(); i++) {
			for (int j = 0; j < jointNames.size(); j++) {
				if (jointNames[j] == g->path_tolerance[i].name) {
					if (!checkTolerance(error[j], g->path_tolerance[i])) {

						trajectory_ptr_port.write(
								trajectory_msgs::JointTrajectoryConstPtr());
						res.error_code =
								control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
						activeGoal.setAborted(res);
						break;
					}
				}
			}
		}
	}
}

bool InternalSpaceSplineTrajectoryAction::checkTolerance(double err,
		control_msgs::JointTolerance tol) {

	if (fabs(err) > tol.position)
		return false;
	else
		return true;
}

void InternalSpaceSplineTrajectoryAction::goalCB(GoalHandle gh) {
	if (!goal_active) {

		trajectory_msgs::JointTrajectory* trj_ptr =
				new trajectory_msgs::JointTrajectory;
		Goal g = gh.getGoal();

		control_msgs::FollowJointTrajectoryResult res;

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
				res.error_code =
						control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
				gh.setRejected(res, "");
				return;
			} else {
				remapTable[i] = jointId;
			}

		}

		//Sprawdzenie ograniczeń w jointach INVALID_GOAL
		bool invalid_goal = false;
		for (unsigned int i = 0; i < numberOfJoints; i++) {
			for (int j = 0; j < g->trajectory.points.size(); j++) {
				if (g->trajectory.points[j].positions[i]
						> upperLimits[remapTable[i]]
						|| g->trajectory.points[j].positions[i]
								< lowerLimits[remapTable[i]]) {
					RTT::Logger::log(RTT::Logger::Debug) << "Invalid goal ["
							<< i << "]: " << upperLimits[remapTable[i]] << ">"
							<< g->trajectory.points[j].positions[i] << ">"
							<< lowerLimits[remapTable[i]] << RTT::endlog();
					invalid_goal = true;
				}
			}
		}
		if (invalid_goal) {
			RTT::Logger::log(RTT::Logger::Debug)
					<< "Trajectory contains invalid goal!" << RTT::endlog();
			res.error_code =
					control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
			gh.setRejected(res, "");
			goal_active = false;
			return;

		}

		//Sprawdzenie czasu w nagłówku OLD_HEADER_TIMESTAMP
		if (rtt_rosclock::host_rt_now() > g->trajectory.header.stamp) {
			RTT::Logger::log(RTT::Logger::Debug) << "Old header timestamp"
					<< RTT::endlog();
			res.error_code =
					control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP;
			gh.setRejected(res, "");

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

ORO_CREATE_COMPONENT(InternalSpaceSplineTrajectoryAction)
