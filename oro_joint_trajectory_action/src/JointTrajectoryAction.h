/*
 * JointTrajectoryAction.h
 *
 *  Created on: 23-09-2010
 *      Author: konrad
 */

#ifndef JOINTTRAJECTORYACTION_H_
#define JOINTTRAJECTORYACTION_H_

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <oro_action_server.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

class JointTrajectoryAction : public RTT::TaskContext
{
private:
    typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
    typedef JTAS::GoalHandle GoalHandle;
    typedef boost::shared_ptr<const pr2_controllers_msgs::JointTrajectoryGoal> Goal;
public:
    JointTrajectoryAction(const std::string& name);
    virtual ~JointTrajectoryAction();

    bool configureHook();
    bool startHook();
    void updateHook();
protected:
    RTT::OutputPort<trajectory_msgs::JointTrajectoryPoint> trajectoryPoint_port;

    RTT::InputPort<bool> bufferReady_port;
    RTT::Property<int> numberOfJoints_prop;
private:

    void goalCB(GoalHandle gh);
    void cancelCB(GoalHandle gh);

    std::vector<trajectory_msgs::JointTrajectoryPoint  > trajectory;
    std::vector<std::string> jointNames;
    unsigned int numberOfJoints;

    unsigned int currentPoint;
    unsigned int endPoint;

    actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> as;
    bool goal_active;
    GoalHandle activeGoal;
};

#endif /* JOINTTRAJECTORYACTION_H_ */
