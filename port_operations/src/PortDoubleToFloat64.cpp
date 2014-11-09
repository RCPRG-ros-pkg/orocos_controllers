// Copyright WUT 2014
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PortDoubleToFloat64.h"

PortDoubleToFloat64::PortDoubleToFloat64(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("InputPort", input_port_);
  this->ports()->addPort("OutputPort", output_port_);

}

PortDoubleToFloat64::~PortDoubleToFloat64() {

}

bool PortDoubleToFloat64::configureHook() {

  return true;
}

void PortDoubleToFloat64::updateHook() {

  double input_data;

  if (RTT::NewData == input_port_.read(input_data)) {
    output_data_.data = input_data;
    output_port_.write(output_data_);

  }

}

ORO_CREATE_COMPONENT(PortDoubleToFloat64)

