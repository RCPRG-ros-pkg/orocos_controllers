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

#include "oro_servo_msgs/Setpoints.h"
#include "oro_servo_msgs/ServoStates.h"

class FakeServo: public RTT::TaskContext {
public:
	FakeServo(const std::string& name);
	virtual ~FakeServo();

  bool configureHook();
	bool startHook();
	void updateHook();
protected:
	RTT::InputPort<oro_servo_msgs::Setpoints> setpoint_port;
	RTT::OutputPort<oro_servo_msgs::ServoStates> jointState_port;

	RTT::Property<int> numberOfJoints_prop;
private:
  oro_servo_msgs::Setpoints setpoint_;
  oro_servo_msgs::ServoStates joint_state_;

  std::vector<double> initial_pos_;

	unsigned int numberOfJoints_;
};

#endif /* FAKESERVO_H_ */
