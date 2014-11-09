// Copyright WUT 2014
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PortDoubleSum.h"

PortDoubleSum::PortDoubleSum(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("OutputPort", output_port_);

  this->addProperty("number_of_ports", number_of_ports_).doc("");
}

PortDoubleSum::~PortDoubleSum() {

}

bool PortDoubleSum::configureHook() {
  port_input_list_.resize(number_of_ports_);

  for (size_t i = 0; i < number_of_ports_; i++) {
    char port_name[16];
    snprintf(port_name, sizeof(port_name), "InputPort_%zu", i);
    port_input_list_[i] = new typeof(*port_input_list_[i]);
    this->ports()->addPort(port_name, *port_input_list_[i]);
  }

  return true;
}


// if any of the inputs is new the new output value is generated
void PortDoubleSum::updateHook() {

  double sum = 0.0;
  bool new_data = false;

  for (int i = 0; i < number_of_ports_; i++) {
    double sum_component;

    if (port_input_list_[i]->read(sum_component) == RTT::NewData) {
      new_data = true;
      sum += sum_component;
    }
  }

  if (new_data) {
    output_port_.write(sum);
  }
}

ORO_CREATE_COMPONENT(PortDoubleSum)

