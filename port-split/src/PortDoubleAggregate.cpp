// Copyright WUT 2014
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PortDoubleAggregate.h"

PortDoubleAggregate::PortDoubleAggregate(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("InputPort", input_port_);

  this->addProperty("number_of_ports", number_of_ports_).doc("");
}

PortDoubleAggregate::~PortDoubleAggregate() {

}

bool PortDoubleAggregate::configureHook() {
  port_output_list_.resize(number_of_ports_);

  for (size_t i = 0; i < number_of_ports_; i++) {
    char port_name[16];
    snprintf(port_name, sizeof(port_name), "OutputPort_%zu", i);
    port_output_list_[i] = new typeof(*port_output_list_[i]);
    this->ports()->addPort(port_name, *port_output_list_[i]);
  }

  data_.resize(number_of_ports_);

  return true;
}

void PortDoubleAggregate::updateHook() {

  if (RTT::NewData == input_port_.read(data_)) {
    for (int i = 0; i < number_of_ports_; i++) {
      port_output_list_[i]->write(data_[i]);
    }
  }

}

ORO_CREATE_COMPONENT(PortDoubleAggregate)

