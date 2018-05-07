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

#include "steering.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

// using namespace std;
// using namespace odcore::base;

Steering::Steering(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-steering")
{
}

Steering::~Steering()
{
}
void Steering::nextContainer(odcore::data::Container &a_container)
{
// checking for the message id and calcualting pwm request and checking left or right steering
  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    auto steering = a_container.getData<opendlv::logic::action::AimPoint>();
    float azimuth = steering.getAzimuthAngle();
    float delta = calcSteering(azimuth);
    float rackPosition = calcRackPosition(delta);
    (void) rackPosition; // Ask Dan if we send rack pos or delta
    opendlv::proxy::GroundSteeringRequest out1(delta);
    odcore::data::Container c1(out1);
    getConference().send(c1);
  }
}

float Steering::calcRackPosition(float delta) {
  const float a = 1;
  const float offset = 1;

  float drt = std::asin(delta)*a;
  float rackPosition = offset+drt;

  return rackPosition;
}

float Steering::calcSteering(float azimuth) {
  float Kp = 2.0f;
  float delta = Kp*azimuth;
  float deltaMax = 3.14/180*22;

  delta = std::min(std::max(delta,-deltaMax),deltaMax);

  return delta;
}

void Steering::setUp()
{
  std::cout << "steering setup" << std::endl;
}

void Steering::tearDown()
{
}

}
}
}
}
