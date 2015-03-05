// Copyright WUT 2015

#include <rtt/Component.hpp>

#include "WrenchPublisher.h"
#include "eigen_conversions/eigen_msg.h"

WrenchPublisher::WrenchPublisher(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("InputWristWrench", port_input_wrist_wrench_);
  this->ports()->addPort("OutputWristWrench", port_output_wrist_wrench_);
}

WrenchPublisher::~WrenchPublisher() {
}

bool WrenchPublisher::configureHook() {

  return true;
}

bool WrenchPublisher::startHook() {

  return true;
}

void WrenchPublisher::updateHook() {




}

ORO_CREATE_COMPONENT(WrenchPublisher)

