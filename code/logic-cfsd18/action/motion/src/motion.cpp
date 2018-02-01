/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <odvdvehicle/GeneratedHeaders_ODVDVehicle.h>

#include "motion.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {


Motion::Motion(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-motion"),
  m_torque(),
  m_steeringAngle(),
  m_brakeEnabled(),
  m_deceleration()
{
}

Motion::~Motion()
{
}



void Motion::nextContainer(odcore::data::Container &a_container)
{

  if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    auto accelerationRequest = a_container.getData<opendlv::proxy::GroundAccelerationRequest>();
    double acceleration = accelerationRequest.getGroundAcceleration();

    m_torque = calcTorque(acceleration);

    if (m_brakeEnabled)
    {
      m_brakeEnabled = false;
      sendBrakeContainer();
    }

    sendActuationContainer();
  }

  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    auto decelerationRequest = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    m_deceleration = decelerationRequest.getGroundDeceleration();

    m_torque = calcTorque(m_deceleration);

    


    if (m_deceleration < 0)
    {
      m_brakeEnabled = true;
    }
    else
    {
      m_brakeEnabled = false;
    }

    sendActuationContainer();
    sendBrakeContainer();

  }

  if (a_container.getDataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
    float Kp = 2;
    auto steeringRequest = a_container.getData<opendlv::proxy::GroundSteeringRequest>();
    double steering = steeringRequest.getGroundSteering();

    m_steeringAngle = static_cast<float>(steering)*Kp;

    sendActuationContainer();
  }
}

void Motion::setUp()
{
  // std::string const exampleConfig =
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-motion.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Motion::tearDown()
{
}

float Motion::calcTorque(double a_arg)
{
  float mass = 200;
  float wheelRadius = 0.3;
  float acceleration = static_cast<float>(a_arg); // convert to float
  float torque = acceleration*mass*wheelRadius;

  return torque;
}

void Motion::sendActuationContainer()
{
  /*opendlv::proxy::ActuationRequest ar(m_torque,m_steeringAngle,true);

  odcore::data::Container c(ar);
  getConference().send(c);*/
}

void Motion::sendBrakeContainer()
{
 /* opendlv::proxy::rhino::BrakeRequest brakeRequest(m_brakeEnbaled,m_deceleration);
  odcore::data::Container c(brakeRequest);

  getConference.send(c);*/
}

}
}
}
}
