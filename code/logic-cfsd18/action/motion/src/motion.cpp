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
  m_steeringAngle(),
  m_brakeEnabled(),
  m_deceleration(),
  m_speed(),
  m_vehicleModelParameters(),
  m_leftMotorID(),
  m_rightMotorID()
{
}

Motion::~Motion()
{
}



void Motion::nextContainer(odcore::data::Container &a_container)
{

  if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    auto accelerationRequest = a_container.getData<opendlv::proxy::GroundAccelerationRequest>();
    float acceleration = accelerationRequest.getGroundAcceleration();

    calcTorque(acceleration);

    if (m_brakeEnabled)
    {
      m_brakeEnabled = false;
    }
  }

  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    auto decelerationRequest = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    m_deceleration = decelerationRequest.getGroundDeceleration();

    if (m_speed > float(5/3.6)){
      calcTorque(m_deceleration);
    }


    if (m_deceleration < 0)
    {
      m_brakeEnabled = true;
    }
    else
    {
      m_brakeEnabled = false;
    }
  }

  if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) { // change this to whatever container marcus sends out
    auto vehicleSpeed = a_container.getData<opendlv::proxy::GroundSpeedReading>();
    m_speed = vehicleSpeed.getGroundSpeed();
  }
}

void Motion::setUp()
{
  // std::string const exampleConfig =
  auto kv = getKeyValueConfiguration();

  double const vM = kv.getValue<double>("logic-sensation-geolocator.vehicle-parameter.m");
	double const vIz = kv.getValue<double>("logic-sensation-geolocator.vehicle-parameter.Iz");
	double const vG = kv.getValue<double>("logic-sensation-geolocator.vehicle-parameter.g");
	double const vL = kv.getValue<double>("logic-sensation-geolocator.vehicle-parameter.l");
	double const vLf = kv.getValue<double>("logic-sensation-geolocator.vehicle-parameter.lf");
	double const vLr = kv.getValue<double>("logic-sensation-geolocator.vehicle-parameter.lr");
	double const vMu = kv.getValue<double>("logic-sensation-geolocator.vehicle-parameter.mu");
	float const vWr = kv.getValue<float>("logic-sensation-geolocator.vehicle-parameter.wr");

  m_vehicleModelParameters << vM,vIz,vG,vL,vLf,vLr,vMu,vWr;

  m_leftMotorID = kv.getValue<int32_t>("logic-action-motion.sender-stamp.LeftMotor");
  m_rightMotorID = kv.getValue<int32_t>("logic-action-motion.sender-stamp.LeftMotor");
}

void Motion::tearDown()
{
}

void Motion::calcTorque(float a_arg)
{
  float mass = (float) m_vehicleModelParameters(1);
  float wheelRadius = m_vehicleModelParameters(8);
  float torque = a_arg*mass*wheelRadius;

  // Torque distribution
  float torqueLeft = torque*0.5f;
  float torqueRight = torque-torqueLeft;

  sendActuationContainer(m_leftMotorID,torqueLeft);
  sendActuationContainer(m_rightMotorID,torqueRight);
}

void Motion::sendActuationContainer(int32_t a_arg, float torque)
{
  opendlv::proxy::TorqueRequest tr(torque);
  odcore::data::Container c(tr);
  c.setSenderStamp(a_arg);
  getConference().send(c);
}


}
}
}
}
