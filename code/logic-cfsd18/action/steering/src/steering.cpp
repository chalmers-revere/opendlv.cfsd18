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
    float distance = steering.getDistance();
    float delta = calcSteering(azimuth, distance);
    float rackPosition = calcRackPosition(delta);
    opendlv::proxy::GroundSteeringRequest out1(rackPosition);
    odcore::data::Container c1(out1);
    getConference().send(c1);
  }
}

float Steering::calcRackPosition(float delta) {
  Eigen::ArrayXf rackTravel;
  Eigen::ArrayXf deltaLeft;
  Eigen::ArrayXf deltaRight;

  rackTravel << -20.188,-18.169,-16.15,-14.131,-12.113,-10.094,-8.075,
  -6.056,-4.038,-2.019,1.51e-15,2.019,4.038,6.056,8.075,10.094,12.113,
  14.131,16.15,18.169,20.188;

  deltaLeft << 26.383,23.404,20.539,17.769,15.078,12.452,9.883,7.36,4.876,
  2.424,9.12e-06,-2.403,-4.789,-7.163,-9.528,-11.89,-14.251,-16.615,-18.987,
  -21.37,-23.767;

  deltaRight << 23.767,21.37,18.987,16.615,14.251,11.89,9.528,7.163,4.789,
  2.403,8.31e-06,-2.424,-4.876,-7.36,-9.883,-12.452,-15.078,-17.769,-20.539,
  -23.404,-26.383;

  Eigen::ArrayXf deltaAvg = (deltaRight + deltaLeft)/2;
  float rackPosition = 0;

  for (int i=1;i < deltaAvg.size();i++){
    if (delta > deltaAvg(i)){
      // Calc dRackTravel/dDelta and do rackTravel(i-1) + dRT/dD*(delta-deltaAvg(i-1))
      float dRTdD = (rackTravel(i)-rackTravel(i-1))/(deltaAvg(i)-deltaAvg(i-1));
      rackPosition = rackTravel(i-1) + dRTdD*(delta-deltaAvg(i-1));
      break;
    }
  }
  return rackPosition;
}

float Steering::calcSteering(float azimuth, float distance) {
  float Kp = 2.0f;
  float delta = Kp*azimuth*distance;

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
