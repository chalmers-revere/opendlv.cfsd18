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
  m_headingRequest(),
  m_brakeEnabled(),
  m_deceleration(),
  m_speed(),
  m_vehicleModelParameters(),
  m_leftMotorID(),
  m_rightMotorID(),
  m_PI(),
  m_aimTime(),
  m_dt()
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

  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    auto headingRequest = a_container.getData<opendlv::logic::action::AimPoint>();
    m_headingRequest = headingRequest.getAzimuthAngle();
  }
}

void Motion::setUp()
{
  // std::string const exampleConfig =
  auto kv = getKeyValueConfiguration();

  float const vM = kv.getValue<float>("global.vehicle-parameter.m");
  float const vIz = kv.getValue<float>("global.vehicle-parameter.Iz");
  float const vG = kv.getValue<float>("global.vehicle-parameter.g");
  float const vL = kv.getValue<float>("global.vehicle-parameter.l");
  float const vLf = kv.getValue<float>("global.vehicle-parameter.lf");
  float const vLr = kv.getValue<float>("global.vehicle-parameter.lr");
  float const vMu = kv.getValue<float>("global.vehicle-parameter.mu");
  float const vWr = kv.getValue<float>("global.vehicle-parameter.wr");

  m_vehicleModelParameters << vM,vIz,vG,vL,vLf,vLr,vMu,vWr;


  m_leftMotorID = kv.getValue<int32_t>("global.sender-stamp.left-motor");
  m_rightMotorID = kv.getValue<int32_t>("global.sender-stamp.right-motor ");

  m_aimTime = kv.getValue<float>("global.sender-stamp.aim-point-time");
  m_dt = kv.getValue<float>("opendlv-logic-cfsd18-action-motion.torque-parameter");

  m_PI = 3.14159265359f;
}

void Motion::tearDown()
{
}

void Motion::calcTorque(float a_arg)
{
  float mass = m_vehicleModelParameters(1);
  float wheelRadius = m_vehicleModelParameters(8);
  float torque = a_arg*mass*wheelRadius;
  float Iz = m_vehicleModelParameters(2);

  float yawRateRef = calcYawRateRef(m_headingRequest);

  float e_yawRate = -yawRateRef; // Add yaw rate here when Marcus is done with message

  float dT = 0.5f/m_dt*e_yawRate*Iz;
  // Torque distribution
  float torqueLeft = torque*0.5f - dT;
  float torqueRight = torque-torqueLeft;

  sendActuationContainer(m_leftMotorID,torqueLeft);
  sendActuationContainer(m_rightMotorID,torqueRight);
}

float Motion::calcYawRateRef(float a_arg){
  float t = m_aimTime*m_PI/a_arg;
  return 2.0f*m_PI/t;
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
