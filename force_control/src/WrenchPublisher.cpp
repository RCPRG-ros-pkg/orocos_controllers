// Copyright WUT 2015
/*
 * WrenchPublisher.cpp
 *
 *  Created on: 5 March 2015
 *      Author: twiniars
 */

#include <rtt/Component.hpp>

#include "WrenchPublisher.h"
#include "rtt_rosclock/rtt_rosclock.h"

WrenchPublisher::WrenchPublisher(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("InputWrench", port_input_wrench_);
  this->ports()->addPort("OutputWrenchStamped", port_output_wrench_stamped_);

  this->addProperty("frame_id", frame_id_).doc("");
}

WrenchPublisher::~WrenchPublisher() {
}

bool WrenchPublisher::configureHook() {
  output_wrench_stamped_.header.frame_id = frame_id_;

  return true;
}

bool WrenchPublisher::startHook() {

  return true;
}

void WrenchPublisher::updateHook() {

  geometry_msgs::Wrench current_wrench;
  port_input_wrench_.readNewest(current_wrench);

  output_wrench_stamped_.header.stamp = rtt_rosclock::host_now();

  output_wrench_stamped_.wrench = current_wrench;

  port_output_wrench_stamped_.write(output_wrench_stamped_);

}

ORO_CREATE_COMPONENT(WrenchPublisher)

