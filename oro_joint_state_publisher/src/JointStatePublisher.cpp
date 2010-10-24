#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <ocl/Component.hpp>

#include "JointStatePublisher.hpp"

JointStatePublisher::JointStatePublisher(const std::string& name) :
    RTT::TaskContext(name, PreOperational), msrJnt_port("servo_states"),
    jointState_port("joints_state"), nJoints_prop("number_of_joints",
        "number of joints")
{
  ports()->addPort(msrJnt_port);
  ports()->addPort(jointState_port);

  this->addProperty(nJoints_prop);
}

JointStatePublisher::~JointStatePublisher()
{
}

bool JointStatePublisher::configureHook()
{
  nJoints = nJoints_prop.get();

  names.resize(nJoints);

  for (unsigned int i = 0; i < nJoints; i++)
  {
    names[i] = ((RTT::Property<std::string>*) this->getProperty(
                  std::string("joint") + (char) (i + 48) + "_name"))->get();
  }
  if (nJoints != names.size())
  {
    return false;
  }

  jState.name.resize(nJoints);
  jState.position.resize(nJoints);
  jState.velocity.resize(nJoints);
  jState.effort.resize(nJoints);

  for (unsigned int i = 0; i < nJoints; i++)
  {
    jState.name[i] = names[i].c_str();
  }

  return true;
}

void JointStatePublisher::updateHook()
{
  if (msrJnt_port.read(msrJnt) == RTT::NewData)
  {
    if (msrJnt.states.size() == nJoints)
    {
      jState.header.stamp = ros::Time::now();
      for (unsigned int i = 0; i < nJoints; i++)
      {
        jState.position[i] = msrJnt.states[i].position;
        jState.velocity[i] = msrJnt.states[i].velocity;
        jState.effort[i] = msrJnt.states[i].effort;
      }
      jointState_port.write(jState);
    }
    else
    {
      RTT::Logger::log(RTT::Logger::Error)
      << "Received servo state have invalid size (received : "
      << msrJnt.states.size() << " expected : " << nJoints << " )"
      << RTT::endlog();
    }
  }
}

ORO_CREATE_COMPONENT( JointStatePublisher )
