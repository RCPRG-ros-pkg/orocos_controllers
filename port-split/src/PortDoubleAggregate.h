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
  RTT::InputPort<Eigen::VectorXd> input_port_;
  std::vector<RTT::OutputPort<double>*> port_output_list_;

  Eigen::VectorXd data_;

  // properties
  int number_of_ports_;


};

#endif /* PortDoubleAggregate */
