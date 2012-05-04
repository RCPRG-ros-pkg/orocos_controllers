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
 * ControllersManager.h
 *
 *  Created on: 04-02-2012
 *      Author: Konrad Banachowicz
 */

#ifndef ControllersManager_H_
#define ControllersManager_H_

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <oro_action_server.h>

#include <oro_controllers_manager/SwitchControllerAction.h>

class ControllersManager : public RTT::TaskContext
{
private:
    typedef actionlib::ActionServer<oro_controllers_manager::SwitchControllerAction> JTAS;
    typedef JTAS::GoalHandle GoalHandle;
    typedef boost::shared_ptr<const oro_controllers_manager::SwitchControllerGoal> Goal;
public:
    ControllersManager(const std::string& name);
    virtual ~ControllersManager();

    bool configureHook();
    bool startHook();
    void updateHook();
protected:
    RTT::Property<int> number_of_controllers_prop_;
    RTT::Property<std::string> default_controller_prop_;
private:

    void goalCB(GoalHandle gh);
    void cancelCB(GoalHandle gh);

    actionlib::ActionServer<oro_controllers_manager::SwitchControllerAction> as;
    bool goal_active;
    GoalHandle activeGoal;
    
    unsigned int number_of_controllers_;
    int active_controller_;
    int next_controller_;
    std::vector<std::string> controllers_names_;
    std::vector<boost::shared_ptr<RTT::OutputPort<bool> > > controllers_enable_port_;
    std::vector<boost::shared_ptr<RTT::InputPort<bool> > >controllers_busy_port_;
};

#endif /* ControllersManager_H_ */
