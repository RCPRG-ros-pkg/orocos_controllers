// Copyright WUT 2014
#ifndef PortDoubleAggregate_H_
#define PortDoubleAggregate_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <string>
#include <vector>
#include <Eigen/Dense>

class PortDoubleAggregate : public RTT::TaskContext {
 public:
  PortDoubleAggregate(const std::string& name);
  virtual ~PortDoubleAggregate();

  bool configureHook();
  void updateHook();
 private:

  // ports
  RTT::OutputPort<Eigen::VectorXd> output_port_;
  std::vector<RTT::InputPort<double>*> port_input_list_;

  Eigen::VectorXd data_;

  // properties
  int number_of_ports_;


};

#endif /* PortDoubleAggregate */
