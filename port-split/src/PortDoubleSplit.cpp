// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "PortDoubleSplit.h"

PortDoubleSplit::PortDoubleSplit(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {


}

PortDoubleSplit::~PortDoubleSplit() {

}

bool PortDoubleSplit::configureHook() {


  return true;
}

void PortDoubleSplit::updateHook() {

}


ORO_CREATE_COMPONENT(PortDoubleSplit)

