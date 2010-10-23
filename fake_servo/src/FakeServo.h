/*
 * FakeServo.h
 *
 *  Created on: 22-09-2010
 *      Author: konrad
 */

#ifndef FAKESERVO_H_
#define FAKESERVO_H_

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include "Setpoint.hpp"
#include "JointState.hpp"

class FakeServo: public RTT::TaskContext {
public:
	FakeServo(const std::string& name);
	virtual ~FakeServo();

  bool configureHook();
	bool startHook();
	void updateHook();
protected:
	RTT::InputPort<std::vector<Setpoint> > setpoint_port;
	RTT::OutputPort<std::vector<JointState> > jointState_port;

	RTT::Property<int> numberOfJoints_prop;
private:
	std::vector<Setpoint> setpoint_;
	std::vector<JointState> joint_state_;

  std::vector<double> initial_pos_;

	unsigned int numberOfJoints_;
};

#endif /* FAKESERVO_H_ */
