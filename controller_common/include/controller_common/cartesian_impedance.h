/*
 * cartesian_impedance.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef CARTESIAN_IMPEDANCE_H_
#define CARTESIAN_IMPEDANCE_H_

#include <rtt/RTT.hpp>
#include <Eigen/Dense>

#include <controller_common/robot.h>

#include <geometry_msgs/Pose.h>
#include <controller_common/CartesianImpedance.h>

template<int N, int K>
class CartesianImpedance : public RTT::TaskContext
{
public:
  CartesianImpedance(const std::string &name) :
      RTT::TaskContext(name), joint_position_(N), joint_velocity_(N), joint_torque_command_(N), nullspace_torque_command_(
          N)
  {
    this->ports()->addPort("JointPosition", port_joint_position_);
    this->ports()->addPort("JointVelocity", port_joint_velocity_);

    this->ports()->addPort("JointTorqueCommand", port_joint_torque_command_);
    this->ports()->addPort("NullSpaceTorqueCommand", port_nullspace_torque_command_);

    for (size_t i = 0; i < K; i++)
    {
      char name[30];
      sprintf(name, "CartesianPositionCommand%zu", i);
      this->ports()->addPort(name, port_cartesian_position_command_[i]);

      sprintf(name, "ToolPositionCommand%zu", i);
      this->ports()->addPort(name, port_tool_position_command_[i]);
    }
  }

  bool configureHook()
  {
    return true;
  }

  bool startHook()
  {
    return true;
  }

  void upadteHook()
  {
    Jacobian J;
    JacobianT JT, Ji;
    Inertia M, Mi, P;
    Stiffness Kc, Dxi;
    Spring p;
    Force F;
    Tool tools[K];
    ToolMass toolsM[K];
    Eigen::Affine3d r[K];
    Eigen::Affine3d r_cmd[K];
    controller_common::CartesinImpedance impedance[K];
    // read inputs
    port_joint_position_.read(joint_position_);
    port_joint_velocity_.read(joint_velocity_);
    port_nullspace_torque_command_.read(nullspace_torque_command_);

    for (size_t i = 0; i < K; i++)
    {
      geometry_msgs::Pose pos;
      port_cartesian_position_command_[i].read(pos);

      r_cmd[i].translation().x() = pos.position.x;
      r_cmd[i].translation().y() = pos.position.y;
      r_cmd[i].translation().z() = pos.position.z;
      r_cmd[i].rotation() = Eigen::Quaternion<double>(pos.orientation.w, pos.orientation.x, pos.orientation.y,
                                                      pos.orientation.z);

      port_tool_position_command_[i].read(pos);

      tools[i].translation().x() = pos.position.x;
      tools[i].translation().y() = pos.position.y;
      tools[i].translation().z() = pos.position.z;
      tools[i].rotation() = Eigen::Quaternion<double>(pos.orientation.w, pos.orientation.x, pos.orientation.y,
                                                      pos.orientation.z);

      port_cartesian_impedance_command_[i].read(impedance[i]);
      Kc(i*6+0) = impedance[i].stiffness.force.x;
      Kc(i*6+1) = impedance[i].stiffness.force.y;
      Kc(i*6+2) = impedance[i].stiffness.force.z;
      Kc(i*6+3) = impedance[i].stiffness.torque.x;
      Kc(i*6+4) = impedance[i].stiffness.torque.y;
      Kc(i*6+5) = impedance[i].stiffness.torque.z;

      Dxi(i*6+0) = impedance[i].damping.force.x;
      Dxi(i*6+1) = impedance[i].damping.force.y;
      Dxi(i*6+2) = impedance[i].damping.force.z;
      Dxi(i*6+3) = impedance[i].damping.torque.x;
      Dxi(i*6+4) = impedance[i].damping.torque.y;
      Dxi(i*6+5) = impedance[i].damping.torque.z;
    }

    // calculate robot data
    robot_->inertia(M, joint_position_, toolsM);
    robot_->jacobian(J, joint_position_, tools);
    robot_->fkin(r, joint_position_, tools);

    JT = J.transpose();
    Mi = M.inverse();

    // calculate stiffness component
    for (size_t i = 0; i < K; i++)
    {
      Eigen::Affine3d tmp;
      tmp = r[i].inverse() * r_cmd[i];

      p(i * 6) = tmp.translation().x();
      p(i * 6 + 1) = tmp.translation().y();
      p(i * 6 + 2) = tmp.translation().z();

      Eigen::Quaternion<double> quat = tmp.rotation();
      p(i * 6 + 3) = quat.x();
      p(i * 6 + 4) = quat.y();
      p(i * 6 + 5) = quat.z();
    }

    F.noalias() = Kc * p;

    // calculate damping component
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix<double, K * 6, K * 6> > es;
    Eigen::Matrix<double, K * 6, K * 6> A, Q, Dc;
    Eigen::Matrix<double, K * 6, 1> K0;
    A.noalias() = (J * Mi * JT).inverse();
    es.compute(Kc.asDiagonal(), A);
    K0 = es.eigenvalues();
    Q = es.eigenvectors().inverse();

    Dc.noalias() = Q.transpose() * Dxi.asDiagonal() * K0.cwiseSqrt().asDiagonal() * Q;
    F.noalias() -= Dc * J * joint_velocity_;

    // calculate null-space component
    Ji.noalias() = Mi * JT * (J * Mi * JT).inverse();
    P.noalias() = Eigen::Matrix<double, N, N>::Identity() - Ji * J;
    joint_torque_command_.noalias() = P * nullspace_torque_command_;

    // write outputs
    joint_torque_command_.noalias() += JT * F;
    port_joint_torque_command_.write(joint_torque_command_);
  }

  typedef controller_common::Robot<N, K> Robot;
  typedef Eigen::Matrix<double, N, K * 6> Jacobian;
  typedef Eigen::Matrix<double, K * 6, N> JacobianT;
  typedef Eigen::Matrix<double, N, N> Inertia;
  typedef Eigen::Matrix<double, N, 1> Joints;
  typedef Eigen::Matrix<double, 4, 1> ToolMass;
  typedef Eigen::Matrix<double, 7, 1> Tool;
  typedef Eigen::Matrix<double, K * 6, 1> Stiffness;
  typedef Eigen::Matrix<double, K * 6, 1> Spring;
  typedef Eigen::Matrix<double, K * 6, 1> Force;

private:
  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_joint_velocity_;

  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_command_[K];
  RTT::InputPort<geometry_msgs::Pose> port_tool_position_command_[K];
  RTT::InputPort<controller_common::CartesinImpedance> port_cartesian_impedance_command_[K];

  RTT::InputPort<Eigen::VectorXd> port_nullspace_torque_command_;

  RTT::OutputPort<Eigen::VectorXd> port_joint_torque_command_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_velocity_;

  Eigen::VectorXd joint_torque_command_;
  Eigen::VectorXd nullspace_torque_command_;

  boost::shared_ptr<Robot> robot_;
};

#endif /* CARTESIAN_IMPEDANCE_H_ */
