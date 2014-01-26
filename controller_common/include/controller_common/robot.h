/*
 * robot.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <rtt/RTT.hpp>
#include <Eigen/Dense>

namespace controller_common
{
template<int N, int K>
class Robot : public RTT::ServiceRequester
{
public:
  Robot(RTT::TaskContext *owner) :
      RTT::ServiceRequester("robot", owner), inertia("inertia"), jacobian("jacobian"), fkin("fkin")
  {
    this->addOperationCaller(inertia);
    this->addOperationCaller(jacobian);
    this->addOperationCaller(fkin);
  }

  typedef Eigen::Matrix<double, N, K * 6> Jacobian;
  typedef Eigen::Matrix<double, N, N> Inertia;
  typedef Eigen::Matrix<double, N, 1> Joints;
  typedef Eigen::Matrix<double, 4, K> ToolMass;
  typedef Eigen::Matrix<double, 7, K> Tool;

  RTT::OperationCaller<void(Inertia &, const Joints &, const ToolMass[K])> inertia;
  RTT::OperationCaller<void(Jacobian &, const Joints &, const Tool[K])> jacobian;
  RTT::OperationCaller<void(Eigen::Matrix<double, 3, 4>[N], const Joints &)> fkin;
};
}

#endif /* ROBOT_H_ */
