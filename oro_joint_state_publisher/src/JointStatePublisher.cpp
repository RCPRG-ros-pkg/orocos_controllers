#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <ocl/Component.hpp>

#include "JointStatePublisher.hpp"

JointStatePublisher::JointStatePublisher(const std::string& name) :
    RTT::TaskContext(name, PreOperational), msrJnt_port("msrJnt"),
    jointState_port("jointState"), nJoints_prop("numberOfJoints",
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

  for (int i = 0; i < nJoints; i++)
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

  for (int i = 0; i < nJoints; i++)
  {
    jState.name[i] = names[i].c_str();
  }

  return true;
}

void JointStatePublisher::updateHook()
{
  if (msrJnt_port.read(msrJnt) == RTT::NewData)
  {
    if (msrJnt.size() == nJoints)
    {
      jState.header.stamp = ros::Time::now();
      for (int i = 0; i < nJoints; i++)
      {
        jState.position[i] = msrJnt[i].position;
        jState.velocity[i] = msrJnt[i].velocity;
      }
      jointState_port.write(jState);
    }
    else
    {
      RTT::Logger::log(RTT::Logger::Error)
      << "Received joint state have invalid size (received : "
      << msrJnt.size() << " expected : " << nJoints << " )"
      << RTT::endlog();
    }
  }
}

ORO_CREATE_COMPONENT( JointStatePublisher )
