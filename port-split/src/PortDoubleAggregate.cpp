// Copyright WUT 2014
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PortDoubleAggregate.h"

PortDoubleAggregate::PortDoubleAggregate(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("OutputPort", output_port_);

  this->addProperty("number_of_ports", number_of_ports_).doc("");
}

PortDoubleAggregate::~PortDoubleAggregate() {

}

bool PortDoubleAggregate::configureHook() {
  port_input_list_.resize(number_of_ports_);

  for (size_t i = 0; i < number_of_ports_; i++) {
    char port_name[16];
    snprintf(port_name, sizeof(port_name), "InputPort_%zu", i);
    port_input_list_[i] = new typeof(*port_input_list_[i]);
    this->ports()->addPort(port_name, *port_input_list_[i]);
  }

  data_.resize(number_of_ports_);

  return true;
}

void PortDoubleAggregate::updateHook() {
  bool new_data = true;
  for (int i = 0; i < number_of_ports_; i++) {
    if (RTT::NewData != port_input_list_[i]->read(data_[i])) {
      new_data = false;
    }
  }

  if (new_data) {
    output_port_.write(data_);
  }
}

ORO_CREATE_COMPONENT(PortDoubleAggregate)

