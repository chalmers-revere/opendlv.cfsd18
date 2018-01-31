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
    double pwm= 3.5 * steering.getGroundSteering();
    uint32_t pwmrequest = static_cast<uint32_t>(pwm);
    uint16_t pinid = 1;
    const int bit = 1;
// sending pwm request 
    opendlv::proxy::PwmRequest pr(pinid,pwmrequest);
    odcore::data::Container c1(pr);
    getConference().send(c1);
// selecting toggle state for on or off state for the steering actuator
    opendlv::proxy::ToggleRequest::ToggleState state;
     if (pwm > 0) {
        state = opendlv::proxy::ToggleRequest::On;
      } else {
        state = opendlv::proxy::ToggleRequest::Off;
      }
     opendlv::proxy::ToggleRequest request(pinid, state);     
     odcore::data::Container c2(request);
     getConference().send(c2);

// For the gpio module, we need to send three containers. One for left steering , second for right steering and thrid for current measurement pin
    
     if (state == opendlv::proxy::ToggleRequest::On){

     opendlv::proxy::ToggleRequest::ToggleState leftbit;
     if (bit > 0) {
        leftbit = opendlv::proxy::ToggleRequest::On;
      } else {
        leftbit = opendlv::proxy::ToggleRequest::Off;
      }
     opendlv::proxy::ToggleRequest requestleft(pinid, leftbit);     
     odcore::data::Container c3(requestleft);
     getConference().send(c3);

     opendlv::proxy::ToggleRequest::ToggleState rightbit;
     if (bit > 0) {
        rightbit = opendlv::proxy::ToggleRequest::Off;
      } else {
        rightbit = opendlv::proxy::ToggleRequest::On;
      }
     opendlv::proxy::ToggleRequest requestright(pinid, rightbit);     
     odcore::data::Container c4(requestright);
     getConference().send(c4);


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
