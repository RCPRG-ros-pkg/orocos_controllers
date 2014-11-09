// Copyright WUT 2014
#ifndef PortDoubleToFloat64_H_
#define PortDoubleToFloat64_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>

#include <string>
#include <vector>
#include <Eigen/Dense>

class PortDoubleToFloat64 : public RTT::TaskContext {
 public:
  PortDoubleToFloat64(const std::string& name);
  virtual ~PortDoubleToFloat64();

  bool configureHook();
  void updateHook();
 private:

  // ports
  RTT::InputPort<double> input_port_;
  RTT::OutputPort<std_msgs::Float64> output_port_;

  std_msgs::Float64 output_data_;

};

#endif /* PortDoubleToFloat64 */
