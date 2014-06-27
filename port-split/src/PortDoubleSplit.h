// Copyright WUT 2014
#ifndef PortDoubleSplit_H_
#define PortDoubleSplit_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>

class PortDoubleSplit : public RTT::TaskContext {
 public:
  PortDoubleSplit(const std::string& name);
  virtual ~PortDoubleSplit();

  bool configureHook();
  void updateHook();
 private:

};

#endif /* PortDoubleSplit */
