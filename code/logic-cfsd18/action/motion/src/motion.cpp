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
  m_aimPoint(),
  m_speed()
  {
}

Motion::~Motion()
{
}



void Motion::nextContainer(odcore::data::Container &a_container)
{
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

  if (a_container.getDataType() == opendlv::sim::KinematicState::ID()) { // change this to whatever container marcus sends out
    auto vehicleSpeed = a_container.getData<opendlv::sim::KinematicState>();
    m_speed = vehicleSpeed.getVx();
  }

  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    m_aimPoint = a_container.getData<opendlv::logic::action::AimPoint>();
  }
}

void Motion::setUp()
{
  // std::string const exampleConfig =
  std::cout << "Setting up motion" << std::endl;
}

void Motion::tearDown()
{
}

void Motion::calcTorque(float a_arg)
{
  auto kv = getKeyValueConfiguration();
  uint32_t leftMotorID = kv.getValue<uint32_t>("global.sender-stamp.left-motor");
  uint32_t rightMotorID = kv.getValue<uint32_t>("global.sender-stamp.right-motor ");
  float dT = kv.getValue<float>("opendlv-logic-cfsd18-action-motion.torque-parameter");


  float mass = kv.getValue<float>("global.vehicle-parameter.m");
  float wheelRadius = kv.getValue<float>("global.vehicle-parameter.wR");
  float torque = a_arg*mass*wheelRadius;
  float Iz = kv.getValue<float>("global.vehicle-parameter.Iz");

  float yawRateRef = calcYawRateRef(m_aimPoint);

  float e_yawRate = -yawRateRef; // Add yaw rate here when Marcus is done with message

  float dTorque = 0.5f/dT*e_yawRate*Iz;
  // Torque distribution
  float torqueLeft = torque*0.5f - dTorque;
  float torqueRight = torque-torqueLeft;

  sendActuationContainer(leftMotorID,torqueLeft);
  sendActuationContainer(rightMotorID,torqueRight);
}

float Motion::calcYawRateRef(opendlv::logic::action::AimPoint aimPoint)
{
  float headingReq = aimPoint.getAzimuthAngle();  // Angle to the aim point
  float dist  = aimPoint.getDistance();           // Distance to the aim point
  float u = m_speed;
  float R = dist/(float)(sqrt(2.0f*(1.0f-(float)cos(2.0f*headingReq))));
  // Calculate the average yaw rate to turn for that specific curve
  float r = std::copysign(u/R,headingReq);

  return r;
}

void Motion::sendActuationContainer(int32_t a_arg, float torque)
{
  (void) a_arg;
  (void) torque;
  opendlv::proxy::TorqueRequest tr(torque);
  odcore::data::Container c(tr);
  c.setSenderStamp(a_arg);
  getConference().send(c);
}


}
}
}
}
