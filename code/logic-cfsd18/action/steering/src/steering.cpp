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
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-steering")
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
    float pwm = 3.5f * steering.getGroundSteering();
    uint32_t pwmrequest = static_cast<uint32_t>(pwm);
    uint16_t pinid = 1;
    const int bit = 1;
// sending pwm request
    opendlv::proxy::PulseWidthModulationRequest pr(pwmrequest);
    odcore::data::Container c1(pr);
    c1.setSenderStamp(pinid);
    getConference().send(c1);

// For the gpio module, we need to send three containers. One for left steering , second for right steering and thrid for current measurement pin

     if (pwm > 0){

     opendlv::proxy::SwitchStateRequest leftbit;
     opendlv::proxy::SwitchStateRequest rightbit;
     if (bit > 0) {
        leftbit.setState(1);
        rightbit.setState(0);
      } else {
        leftbit.setState(0);
        rightbit.setState(1);
      }
     odcore::data::Container c2(leftbit);
     c2.setSenderStamp(1);          // Set some ID for left turn
     getConference().send(c2);


     odcore::data::Container c3(rightbit);
     c3.setSenderStamp(2);        // Id for right turn
     getConference().send(c3);

/*        These are old messages
     opendlv::proxy::ToggleRequest::ToggleState selectbit;
     if (leftbit == opendlv::proxy::ToggleRequest::On) {
        selectbit = opendlv::proxy::ToggleRequest::On;
        selectbit = opendlv::proxy::ToggleRequest::Off;
      }
     opendlv::proxy::ToggleRequest requestselect(pinid, selectbit);
     odcore::data::Container c4(requestselect);
     getConference().send(c4);
*/


    }
  }
}

void Steering::setUp()
{
  // std::string const exampleConfig =
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-steering.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Steering::tearDown()
{
}

}
}
}
}
