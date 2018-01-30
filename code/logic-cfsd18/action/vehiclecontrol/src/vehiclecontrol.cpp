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

#include "vehiclecontrol.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

Vehiclecontrol::Vehiclecontrol(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-vehiclecontrol"),
  m_accelerationRequest()
{
}

Vehiclecontrol::~Vehiclecontrol()
{
}



void Vehiclecontrol::nextContainer(odcore::data::Container &a_container)
{

  if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    auto accelerationRequest = a_container.getData<opendlv::proxy::GroundAccelerationRequest>();

    m_accelerationRequest = accelerationRequest.getGroundAcceleration();
  }
}

void Vehiclecontrol::setUp()
{
  // std::string const exampleConfig =
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-vehiclecontrol.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Vehiclecontrol::tearDown()
{
}

}
}
}
}