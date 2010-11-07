/*
 * JointTrajectoryAction.cpp
 *
 *  Created on: 23-09-2010
 *      Author: konrad
 */
#include <ocl/Component.hpp>

#include "JointTrajectoryAction.h"

JointTrajectoryAction::JointTrajectoryAction(const std::string& name) :
    RTT::TaskContext(name, PreOperational), trajectoryPoint_port(
      "trajectory_point"), bufferReady_port("buffer_ready"),
    numberOfJoints_prop("number_of_joints", "", 0), as(this,
        "JointTrajectoryAction", boost::bind(
          &JointTrajectoryAction::goalCB, this, _1),
        boost::bind(&JointTrajectoryAction::cancelCB, this, _1),
        true)
{

  this->addPort(trajectoryPoint_port);
  this->addPort(bufferReady_port);

  this->addProperty(numberOfJoints_prop);

}

JointTrajectoryAction::~JointTrajectoryAction()
{

}

bool JointTrajectoryAction::configureHook()
{
  if ((numberOfJoints = numberOfJoints_prop.get()) == 0)
  {
    return false;
  }

  jointNames.resize(numberOfJoints);

  for (unsigned int i = 0; i < numberOfJoints; i++)
  {
    jointNames[i] = ((RTT::Property<std::string>*) this->getProperty(
                       std::string("joint") + (char) (i + 48) + "_name"))->get();
  }

  return true;
}

bool JointTrajectoryAction::startHook()
{
  goal_active = false;
  return true;
}

void JointTrajectoryAction::updateHook()
{
  bool tmp;
  as.spinOnce();

  if (bufferReady_port.read(tmp) == RTT::NewData)
  {
    if (tmp && goal_active)
    {
      if (currentPoint < endPoint)
      {
        trajectoryPoint_port.write(trajectory[currentPoint]);
        ++currentPoint;
      }
      else
      {
        RTT::Logger::log(RTT::Logger::Debug) << "Trajectory complete" << RTT::endlog();
        goal_active = false;
        activeGoal.setSucceeded();
      }
    }
  }
}

void JointTrajectoryAction::goalCB(GoalHandle gh)
{
  if (!goal_active)
  {
    std::vector<int> remapTable;
    remapTable.resize(numberOfJoints);

    Goal g = gh.getGoal();

    RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contain " << g->trajectory.points.size() << " points" << RTT::endlog();

    // fill remap table
    for (unsigned int i = 0; i < numberOfJoints; i++)
    {
      int jointId = -1;
      for (unsigned int j = 0; j < g->trajectory.joint_names.size(); j++)
      {
        if (g->trajectory.joint_names[j] == jointNames[i])
        {
          jointId = j;
          break;
        }
      }
      if (jointId < 0)
      {
        RTT::Logger::log(RTT::Logger::Error) << "Trajectory contains invalid joint" << RTT::endlog();
        gh.setRejected();
        return;
      }
      else
      {
        remapTable[i] = jointId;
      }

    }

    //remap joints

    trajectory.resize(g->trajectory.points.size());

    for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
    {
      trajectory[i].positions.resize(g->trajectory.points[i].positions.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].positions.size(); j++)
      {
        trajectory[i].positions[j]
        = g->trajectory.points[i].positions[remapTable[j]];
      }

      trajectory[i].velocities.resize(g->trajectory.points[i].velocities.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].velocities.size(); j++)
      {
        trajectory[i].velocities[j]
        = g->trajectory.points[i].velocities[remapTable[j]];
      }

      trajectory[i].accelerations.resize(g->trajectory.points[i].accelerations.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].accelerations.size(); j++)
      {
        trajectory[i].accelerations[j]
        = g->trajectory.points[i].accelerations[remapTable[j]];
      }

      if(i == 0)
      {
        trajectory[i].time_from_start = g->trajectory.points[i].time_from_start;
      }
      else
      {
        trajectory[i].time_from_start = g->trajectory.points[i].time_from_start - g->trajectory.points[i-1].time_from_start;
      }
    }

    endPoint = g->trajectory.points.size();
    currentPoint = 0;

    activeGoal = gh;
    goal_active = true;
    gh.setAccepted();

  }
  else
  {
    gh.setRejected();
  }
}

void JointTrajectoryAction::cancelCB(GoalHandle gh)
{
  goal_active = false;
}

ORO_CREATE_COMPONENT( JointTrajectoryAction )
