// Copyright WUT 2014
#include <rtt/Component.hpp>

#include "ForceTransformation.h"
#include "eigen_conversions/eigen_msg.h"

ForceTransformation::ForceTransformation(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);

  this->ports()->addPort("CurrentSensorSlowFilteredWrench",
                         port_current_sensor_slow_filtered_wrench_);
  this->ports()->addPort("CurrentSensorFastFilteredWrench",
                         port_current_sensor_fast_filtered_wrench_);
  this->ports()->addPort("OutputWristWrench", port_output_wrist_wrench_);
  this->ports()->addPort("OutputEndEffectorWrench",
                         port_output_end_effector_wrench_);
  this->ports()->addPort("Tool", port_tool_);
  this->ports()->addPort("ToolGravityParam", port_current_tool_gravity_param_);

  this->addProperty("sensor_frame", sensor_frame_property_);
  this->addProperty("is_right_turn_frame", is_right_turn_frame_property_);

  this->addProperty("tool_weight", tool_weight_property_);
  this->addProperty("gravity_arm_in_wrist", gravity_arm_in_wrist_property_);
}

ForceTransformation::~ForceTransformation() {
}

bool ForceTransformation::configureHook() {
  tf::poseMsgToKDL(sensor_frame_property_, sensor_frame_kdl_);
  tf::vectorMsgToKDL(gravity_arm_in_wrist_property_, gravity_arm_in_wrist_kdl_);

  return true;
}

bool ForceTransformation::startHook() {
  // read current force ad set as an offset force
  geometry_msgs::Wrench current_wrench;
  if (port_current_sensor_slow_filtered_wrench_.read(current_wrench)
      == RTT::NoData) {
    return false;
  }

  tf::wrenchMsgToKDL(current_wrench, force_offset_);

  // read current wrist pose
  geometry_msgs::Pose current_wrist_pose;
  if (port_current_wrist_pose_.read(current_wrist_pose) == RTT::NoData) {
    return false;
  }

  //  set tool weight and center point from input ports
  force_control_msgs::ToolGravityParam tg_param;
  if (port_current_tool_gravity_param_.read(tg_param) == RTT::NewData) {
    tool_weight_property_ = tg_param.weight;
    geometry_msgs::Vector3 gravity_arm_in_wrist_property_ = tg_param.mass_center;
    tf::vectorMsgToKDL(gravity_arm_in_wrist_property_,
                       gravity_arm_in_wrist_kdl_);
  }
  // compute reaction force

  gravity_force_torque_in_base_ = KDL::Wrench(
      KDL::Vector(0.0, 0.0, -tool_weight_property_),
      KDL::Vector(0.0, 0.0, 0.0));

  KDL::Frame current_frame;

  tf::poseMsgToKDL(current_wrist_pose, current_frame);

// sila reakcji w ukladzie czujnika z orientacja bazy
  KDL::Wrench gravity_force_torque_in_sensor = current_frame.M.Inverse()
      * gravity_force_torque_in_base_;

// macierz narzedzia wzgledem nadgarstka
  tool_mass_center_translation_ = KDL::Frame(KDL::Rotation(),
                                             gravity_arm_in_wrist_kdl_);

// sila reakcji w ukladzie nadgarstka z orientacja bazy
  reaction_force_torque_in_wrist_ = -(tool_mass_center_translation_
      * gravity_force_torque_in_sensor);

  return true;
}

void ForceTransformation::updateHook() {
  geometry_msgs::Pose current_wrist_pose;
  port_current_wrist_pose_.read(current_wrist_pose);
  KDL::Frame current_wrist_pose_kdl;
  tf::poseMsgToKDL(current_wrist_pose, current_wrist_pose_kdl);

  // odczyt sily
  geometry_msgs::Wrench current_wrench;
  port_current_sensor_fast_filtered_wrench_.read(current_wrench);
  KDL::Wrench input_force;
  tf::wrenchMsgToKDL(current_wrench, input_force);

  // offset level removal
  KDL::Wrench biased_force = input_force - force_offset_;

  if (!is_right_turn_frame_property_) {
    biased_force[2] = -biased_force[2];
    biased_force[5] = -biased_force[5];
  }

  // sprowadzenie wejsciowych, zmierzonych sil i momentow sil z ukladu czujnika do ukladu nadgarstka
  biased_force = sensor_frame_kdl_ * biased_force;

  // sprowadzenie odczytow sil do ukladu czujnika przy zalozeniu ze uklad chwytaka ma te sama orientacje
  // co uklad narzedzia
  KDL::Wrench gravity_force_torque_in_sensor =
      (current_wrist_pose_kdl.Inverse()).M * gravity_force_torque_in_base_;

  // finalne przeksztalcenie (3.30 z doktoratu TW)
  KDL::Wrench computed_force = biased_force
      - tool_mass_center_translation_ * gravity_force_torque_in_sensor
      - reaction_force_torque_in_wrist_;

  // sprowadzenie sily w ukladzie nadgarstka do orientacji ukladu bazowego
  computed_force = current_wrist_pose_kdl.M * (-computed_force);

  geometry_msgs::Wrench output_wrist_wrench;
  tf::wrenchKDLToMsg(computed_force, output_wrist_wrench);

  port_output_wrist_wrench_.write(output_wrist_wrench);

  // tool determination
  geometry_msgs::Pose tool_msgs;
  port_tool_.read(tool_msgs);
  KDL::Frame tool_kdl;
  tf::poseMsgToKDL(tool_msgs, tool_kdl);

  KDL::Wrench computed_ef_force = tool_kdl.Inverse()
      * (current_wrist_pose_kdl.M.Inverse() * computed_force);

  geometry_msgs::Wrench output_end_effector_wrench;
  tf::wrenchKDLToMsg(computed_ef_force, output_end_effector_wrench);

  port_output_end_effector_wrench_.write(output_end_effector_wrench);
}

ORO_CREATE_COMPONENT(ForceTransformation)

