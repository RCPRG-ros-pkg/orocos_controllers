/*
 * JointSplineTrajectoryGenerator.h
 *
 *  Created on: 22-09-2010
 *      Author: konrad
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
