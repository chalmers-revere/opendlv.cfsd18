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
  if (a_container.getDataType() == opendlv::logic::cognition::GroundSpeedLimit::ID()) {
     //auto acc = 200 * getKeyValueConfiguration().getValue<double>("opendlv.logic.cognition.GroundSpeedLimit");
     auto acc = a_container.getData<opendlv::logic::cognition::GroundSpeedLimit>();
     double acc_req = acc.getSpeedLimit();

     opendlv::proxy::GroundAccelerationRequest gac(acc_req);
     odcore::data::Container c1(gac);
     //Container c1(gac);
     getConference().send(c1);

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
