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

#define PI 3.14159265359f

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {


Motion::Motion(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-motion"),
  m_headingRequest(),
  m_speed(),
  m_vehicleModelParameters(),
  {
}

Motion::~Motion()
{
}



void Motion::nextContainer(odcore::data::Container &a_container)
{
  std::cout << "next container" << std::endl;
  (void) a_container;
  if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    auto accelerationRequest = a_container.getData<opendlv::proxy::GroundAccelerationRequest>();
    float acceleration = accelerationRequest.getGroundAcceleration();

    calcTorque(acceleration);
  }

  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    auto decelerationRequest = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    float deceleration = decelerationRequest.getGroundDeceleration();

    if (m_speed > float(5/3.6)){
      calcTorque(deceleration);
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
  std::cout << "Setting up motion" << std::endl;

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
  }

void Motion::tearDown()
{
}

void Motion::calcTorque(float a_arg)
{
  uint32_t leftMotorID = kv.getValue<uint32_t>("global.sender-stamp.left-motor");
  uint32_t rightMotorID = kv.getValue<uint32_t>("global.sender-stamp.right-motor ");
  float dT = kv.getValue<float>("opendlv-logic-cfsd18-action-motion.torque-parameter");


  float mass = m_vehicleModelParameters(1);
  float wheelRadius = m_vehicleModelParameters(8);
  float torque = a_arg*mass*wheelRadius;
  float Iz = m_vehicleModelParameters(2);

  float yawRateRef = calcYawRateRef(m_headingRequest);

  float e_yawRate = -yawRateRef; // Add yaw rate here when Marcus is done with message

  float dT = 0.5f/dT*e_yawRate*Iz;
  // Torque distribution
  float torqueLeft = torque*0.5f - dT;
  float torqueRight = torque-torqueLeft;

  sendActuationContainer(leftMotorID,torqueLeft);
  sendActuationContainer(rightMotorID,torqueRight);
}

float Motion::calcYawRateRef(opendlv::logic::action::AimPoint aimPoint){
  float headingReq = aimPoint.getAzimuthAngle();  // Angle to the aim point
  float dist  = aimPoint.getDistance();           // Distance to the aim point
  float u = m_speed;
  awrt(2.0f*(1.0f-(float)cos(2.0f*headingReq))));
  // Calculate the average yaw rate to turn for that specific curve
  float r = std::copysign(u/R,headingReq);

return r;
}

void Motion::sendActuationContainer(int32_t a_arg, float torque)
{(void) a_arg;
  (void) torque;
  opendlv::proxy::TorqueRequest tr(torque);
  odcore::data::Container c(tr);
  c.setSenderStamp(a_arg);
  getConference().send(c);
  std::cout << "Sent torque request" << std::endl;
}


}
}
}
}
