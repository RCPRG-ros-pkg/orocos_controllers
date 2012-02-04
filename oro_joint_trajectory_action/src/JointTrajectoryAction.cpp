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
 * JointTrajectoryAction.cpp
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include "JointTrajectoryAction.h"

JointTrajectoryAction::JointTrajectoryAction(const std::string& name) :
  RTT::TaskContext(name, PreOperational), trajectoryPoint_port("trajectory_point"), bufferReady_port("buffer_ready"),
      numberOfJoints_prop("number_of_joints", "", 0), command_port_("command"),
      trajectoryCompleat_port("trajectory_compleat"), as(this, "JointTrajectoryAction",
                                                         boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                                                         boost::bind(&JointTrajectoryAction::cancelCB, this, _1), true)
{

  this->addPort(trajectoryPoint_port);
  this->addEventPort(bufferReady_port, boost::bind(&JointTrajectoryAction::bufferReadyCB, this));
  this->addEventPort(command_port_, boost::bind(&JointTrajectoryAction::commandCB, this));
  this->addEventPort(trajectoryCompleat_port, boost::bind(&JointTrajectoryAction::compleatCB, this));
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
    jointNames[i]
        = ((RTT::Property<std::string>*)this->getProperty(std::string("joint") + (char)(i + 48) + "_name"))->get();
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
  as.spinOnce();
}

void JointTrajectoryAction::goalCB(GoalHandle gh)
{
  if (!goal_active)
  {
    std::vector<int> remapTable;
    remapTable.resize(numberOfJoints);

    Goal g = gh.getGoal();

    RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contain " << g->trajectory.points.size() << " points"
        << RTT::endlog();

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
        trajectory[i].positions[j] = g->trajectory.points[i].positions[remapTable[j]];
      }

      trajectory[i].velocities.resize(g->trajectory.points[i].velocities.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].velocities.size(); j++)
      {
        trajectory[i].velocities[j] = g->trajectory.points[i].velocities[remapTable[j]];
      }

      trajectory[i].accelerations.resize(g->trajectory.points[i].accelerations.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].accelerations.size(); j++)
      {
        trajectory[i].accelerations[j] = g->trajectory.points[i].accelerations[remapTable[j]];
      }

      if (i == 0)
      {
        trajectory[i].time_from_start = g->trajectory.points[i].time_from_start;
      }
      else
      {
        trajectory[i].time_from_start = g->trajectory.points[i].time_from_start
            - g->trajectory.points[i - 1].time_from_start;
      }
    }

    endPoint = g->trajectory.points.size();
    currentPoint = 0;

    activeGoal = gh;
    goal_active = true;
    
    bool ok = true;
    
    RTT::TaskContext::PeerList peers = this->getPeerList();
    for(size_t i = 0; i < peers.size(); i++)
    {
      RTT::Logger::log(RTT::Logger::Debug) << "Starting peer : " << peers[i] << RTT::endlog();
      ok = ok && this->getPeer(peers[i])->start();
    }
    
    if(ok)
    {
      gh.setAccepted();
    } else
    {
      gh.setRejected();
      goal_active = false;
    }
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

void JointTrajectoryAction::commandCB()
{
  trajectory_msgs::JointTrajectory trj;
  if ((command_port_.read(trj) == RTT::NewData)&&(!goal_active))
  {
    std::vector<int> remapTable;
    remapTable.resize(numberOfJoints);

    RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contain " << trj.points.size() << " points"
        << RTT::endlog();

    // fill remap table
    for (unsigned int i = 0; i < numberOfJoints; i++)
    {
      int jointId = -1;
      for (unsigned int j = 0; j < trj.joint_names.size(); j++)
      {
        if (trj.joint_names[j] == jointNames[i])
        {
          jointId = j;
          break;
        }
      }
      if (jointId < 0)
      {
        RTT::Logger::log(RTT::Logger::Error) << "Trajectory contains invalid joint" << RTT::endlog();
        return;
      }
      else
      {
        remapTable[i] = jointId;
      }

    }

    //remap joints

    trajectory.resize(trj.points.size());

    for (unsigned int i = 0; i < trj.points.size(); i++)
    {
      trajectory[i].positions.resize(trj.points[i].positions.size());
      for (unsigned int j = 0; j < trj.points[i].positions.size(); j++)
      {
        trajectory[i].positions[j] = trj.points[i].positions[remapTable[j]];
      }

      trajectory[i].velocities.resize(trj.points[i].velocities.size());
      for (unsigned int j = 0; j < trj.points[i].velocities.size(); j++)
      {
        trajectory[i].velocities[j] = trj.points[i].velocities[remapTable[j]];
      }

      trajectory[i].accelerations.resize(trj.points[i].accelerations.size());
      for (unsigned int j = 0; j < trj.points[i].accelerations.size(); j++)
      {
        trajectory[i].accelerations[j] = trj.points[i].accelerations[remapTable[j]];
      }

      if (i == 0)
      {
        trajectory[i].time_from_start = trj.points[i].time_from_start;
      }
      else
      {
        trajectory[i].time_from_start = trj.points[i].time_from_start
            - trj.points[i - 1].time_from_start;
      }
    }

    endPoint = trj.points.size();
    currentPoint = 0;

    goal_active = true;
  }

}

void JointTrajectoryAction::compleatCB()
{
  if (goal_active)
  {
    activeGoal.setSucceeded();
    goal_active = false;
    
    RTT::TaskContext::PeerList peers = this->getPeerList();
    for(size_t i = 0; i < peers.size(); i++)
    {
      RTT::Logger::log(RTT::Logger::Debug) << "Stoping peer : " << peers[i] << RTT::endlog();
      this->getPeer(peers[i])->stop();
    }
    RTT::Logger::log(RTT::Logger::Debug) << "Trajectory complete" << RTT::endlog();
  }
}

void JointTrajectoryAction::bufferReadyCB()
{
  bool tmp;
  if (bufferReady_port.read(tmp) == RTT::NewData)
  {
    if (tmp && goal_active)
    {
      if (currentPoint < endPoint)
      {
        while(trajectory[currentPoint].time_from_start.toSec() < 0.01)
          ++currentPoint;
        RTT::Logger::log(RTT::Logger::Debug) << "Sending new point tmp = " << tmp << " goal_active = " << goal_active << " currentPoint = " << currentPoint << RTT::endlog();
        trajectoryPoint_port.write(trajectory[currentPoint]);
        ++currentPoint;
      }
    }
  }
}

ORO_CREATE_COMPONENT( JointTrajectoryAction )
