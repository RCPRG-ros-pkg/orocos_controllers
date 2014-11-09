// Copyright WUT 2014
#ifndef PortDoubleSplit_H_
#define PortDoubleSplit_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <string>
#include <vector>
#include <Eigen/Dense>

class PortDoubleSplit : public RTT::TaskContext {
 public:
  PortDoubleSplit(const std::string& name);
  virtual ~PortDoubleSplit();

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

#endif /* PortDoubleSplit */
