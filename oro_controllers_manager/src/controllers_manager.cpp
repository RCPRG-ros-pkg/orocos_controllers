/*
 * Copyright (c) 2012, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * ControllersManager.cpp
 *
 *  Created on: 04-02-2012
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include "controllers_manager.h"

ControllersManager::ControllersManager(const std::string& name) :
  RTT::TaskContext(name, PreOperational), number_of_controllers_prop_("NumberOfControllers"), default_controller_prop_("DefaultController"), as(this, "ControllersManager",
                                                         boost::bind(&ControllersManager::goalCB, this, _1),
                                                         boost::bind(&ControllersManager::cancelCB, this, _1), true)
{
  this->addProperty(number_of_controllers_prop_);
  this->addProperty(default_controller_prop_);
}

ControllersManager::~ControllersManager()
{

}

bool ControllersManager::configureHook()
{
  
  
  if ((number_of_controllers_ = number_of_controllers_prop_.get()) == 0)
  {
    return false;
  }

   RTT::Logger::log(RTT::Logger::Error) << "number of controllers " << number_of_controllers_
        << RTT::endlog();

  for (unsigned int i = 0; i < number_of_controllers_; i++)
  {
    controllers_names_.push_back(((RTT::Property<std::string>*)this->getProperty(std::string("controller_") + (char)(i + 48) + "_name"))->get());
    controllers_enable_port_.push_back(boost::shared_ptr<RTT::OutputPort<bool> > (new RTT::OutputPort<bool>));
    controllers_busy_port_.push_back(boost::shared_ptr<RTT::InputPort<bool> > (new RTT::InputPort<bool>));
    
    this->addPort(controllers_names_[i] + "Enable" , *controllers_enable_port_[i]);
    this->addPort(controllers_names_[i] + "Busy" , *controllers_busy_port_[i]);
  }

  return true;
}

bool ControllersManager::startHook()
{
  std::string default_controller = default_controller_prop_.get();
  
  active_controller_ = -1;
  
  for(size_t i = 0; i < controllers_names_.size(); i++)
  {
    if(controllers_names_[i] == default_controller)
    {
      active_controller_ = i;
    }
  }
  
  if(active_controller_ == -1)
    return false;
    
  for(size_t i = 0; i < number_of_controllers_; i++)
  {
    if(i == active_controller_)
    {
      controllers_enable_port_[i]->write(true);
    } else 
    {
      controllers_enable_port_[i]->write(false);
    }
  }
  
  return true;
}

void ControllersManager::updateHook()
{
  as.spinOnce();
  
  if(active_controller_ == next_controller_)
  {
    for(size_t i = 0; i < number_of_controllers_; i++)
    {
      if(i == active_controller_)
      {
        controllers_enable_port_[i]->write(true);
      } else 
      {
        controllers_enable_port_[i]->write(false);
      }
    }
  } else
  {
    for(size_t i = 0; i < number_of_controllers_; i++)
    { 
      controllers_enable_port_[i]->write(false);
    }
    
    bool busy = true;
    controllers_busy_port_[active_controller_]->read(busy);
    
    if(!busy)
    {
      active_controller_ = next_controller_;
      activeGoal.setSucceeded();
    }
  }
  
}

void ControllersManager::goalCB(GoalHandle gh)
{
    int controller = -1;
    
    Goal g = gh.getGoal();

    activeGoal = gh;
 
    for(size_t i = 0; i < controllers_names_.size(); i++)
    {
      if(controllers_names_[i] == g->controller_id)
      {
        controller = i;
      }
    }
 
    if(controller == -1)
    {
      gh.setRejected();
    } else 
    {
      gh.setAccepted();
      next_controller_ = controller;
    }
}

void ControllersManager::cancelCB(GoalHandle gh)
{
  goal_active = false;
}

ORO_CREATE_COMPONENT( ControllersManager )
