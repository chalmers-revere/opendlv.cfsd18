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
#include <cstdlib>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>


#include "brakes.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

using namespace std;
using namespace odcore::base;

Brakes::Brakes(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-brakes")
{
}

Brakes::~Brakes()
{
}




void Brakes::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {

     auto deceleration = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
     double pwmrequest = 3.5 * deceleration.getGroundDeceleration();
     uint32_t pwmrequestt = static_cast<uint32_t>(pwmrequest);
     uint16_t pinid = 1;

     opendlv::proxy::PwmRequest pr(pinid,pwmrequestt);
     odcore::data::Container c1(pr);
     getConference().send(c1);

      opendlv::proxy::ToggleRequest::ToggleState state;
      if (pwmrequest < 0) {
        state = opendlv::proxy::ToggleRequest::On;
      } else {
        state = opendlv::proxy::ToggleRequest::Off;
      }
      opendlv::proxy::ToggleRequest request(pinid, state);     
      odcore::data::Container c2(request);
      getConference().send(c2);


  }
}

void Brakes::setUp()
{
  // std::string const exampleConfig = 
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-brakes.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Brakes::tearDown()
{
}

}
}
}
}
