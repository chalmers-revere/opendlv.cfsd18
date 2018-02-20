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

using namespace std;
using namespace odcore::base;

Steering::Steering(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-steering"),
  m_pwmId(),
  m_stateId1(),
  m_stateId2(),
  m_stateId3()
{
}

Steering::~Steering()
{
}



void Steering::nextContainer(odcore::data::Container &a_container)
{
// checking for the message id and calcualting pwm request and checking left or right steering
  if (a_container.getDataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
    auto steering = a_container.getData<opendlv::proxy::GroundSteeringRequest>();
    float groundSteering = steering.getGroundSteering();
    uint32_t pwmrequest = calcSteering(groundSteering);
    const int bit = 1;
// sending pwm request
    opendlv::proxy::PulseWidthModulationRequest pr(pwmrequest);


// For the gpio module, we need to send three containers. One for left steering , second for right steering and thrid for current measurement pin

     if (pwmrequest > 0){

     opendlv::proxy::SwitchStateRequest leftbit;
     opendlv::proxy::SwitchStateRequest rightbit;
     opendlv::proxy::SwitchStateRequest thirdbit;

     if (bit > 0) {
        leftbit.setState(1);
        rightbit.setState(0);
        thirdbit.setState(1);
      } else {
        leftbit.setState(0);
        rightbit.setState(1);
        thirdbit.setState(0);
      }


      odcore::data::Container c1(pr);
      c1.setSenderStamp(m_pwmId);
      getConference().send(c1);


      odcore::data::Container c2(leftbit);
      c2.setSenderStamp(m_stateId1);          // Set some ID for left turn
      getConference().send(c2);


      odcore::data::Container c3(rightbit);
      c3.setSenderStamp(m_stateId2);        // Id for right turn
      getConference().send(c3);

      odcore::data::Container c4(thirdbit);
      c4.setSenderStamp(m_stateId3);        // Third bit
      getConference().send(c4);

    }
  }
}

uint32_t Steering::calcSteering(float a_arg) {

  float Kp = 2.0f;
  float pwm = Kp*a_arg;

  return static_cast<uint32_t>(pwm);
}


void Steering::setUp()
{
  auto kv = getKeyValueConfiguration();

  m_pwmId = kv.getValue<uint32_t>("logic-action-steering.sender-stamp.steeringID");
  m_stateId1 = kv.getValue<uint32_t>("logic-action-steering.sender-stamp.steeringIDLeft");
  m_stateId2 = kv.getValue<uint32_t>("logic-action-steering.sender-stamp.steeringIDRight");
  m_stateId3 = kv.getValue<uint32_t>("logic-action-steering.sender-stamp.steeringIDThird");
}

void Steering::tearDown()
{
}

}
}
}
}
