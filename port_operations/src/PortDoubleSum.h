// Copyright WUT 2014
#ifndef PortDoubleSum_H_
#define PortDoubleSum_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <string>
#include <vector>
#include <Eigen/Dense>

class PortDoubleSum : public RTT::TaskContext {
 public:
  PortDoubleSum(const std::string& name);
  virtual ~PortDoubleSum();

  bool configureHook();
  void updateHook();
 private:

  // ports
  RTT::OutputPort<double> output_port_;
  std::vector<RTT::InputPort<double>*> port_input_list_;

  // properties
  int number_of_ports_;

};

#endif /* PortDoubleSum_H_ */
