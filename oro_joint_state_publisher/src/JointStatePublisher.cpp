#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <ocl/Component.hpp>

#include "JointStatePublisher.hpp"

JointStatePublisher::JointStatePublisher(const std::string& name) :
    RTT::TaskContext(name, PreOperational), servo_state_port("servo_states"),
    joint_state_port("joints_state"), number_of_joints_prop("number_of_joints",
        "number of joints")
{
  ports()->addPort(servo_state_port);
  ports()->addPort(joint_state_port);

  this->addProperty(number_of_joints_prop);
}

JointStatePublisher::~JointStatePublisher()
{
}

bool JointStatePublisher::configureHook()
{
  number_of_joints_ = number_of_joints_prop.get();

  names_.resize(number_of_joints_);

  for (unsigned int i = 0; i < number_of_joints_; i++)
  {
    names_[i] = ((RTT::Property<std::string>*) this->getProperty(
                  std::string("joint") + (char) (i + 48) + "_name"))->get();
  }
  if (number_of_joints_ != names_.size())
  {
    return false;
  }

  joint_state_.name.resize(number_of_joints_);
  joint_state_.position.resize(number_of_joints_);
  joint_state_.velocity.resize(number_of_joints_);
  joint_state_.effort.resize(number_of_joints_);

  for (unsigned int i = 0; i < number_of_joints_; i++)
  {
    joint_state_.name[i] = names_[i].c_str();
  }

  return true;
}

void JointStatePublisher::updateHook()
{
  if (servo_state_port.read(servo_state_) == RTT::NewData)
  {
    if (servo_state_.states.size() == number_of_joints_)
    {
      joint_state_.header.stamp = ros::Time::now();
      for (unsigned int i = 0; i < number_of_joints_; i++)
      {
        joint_state_.position[i] = servo_state_.states[i].position;
        joint_state_.velocity[i] = servo_state_.states[i].velocity;
        joint_state_.effort[i] = servo_state_.states[i].effort;
      }
      joint_state_port.write(joint_state_);
    }
    else
    {
      RTT::Logger::log(RTT::Logger::Error)
      << "Received servo state have invalid size (received : "
      << servo_state_.states.size() << " expected : " << number_of_joints_ << " )"
      << RTT::endlog();
    }
  }
}

ORO_CREATE_COMPONENT( JointStatePublisher )
