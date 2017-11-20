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

#include "lateral.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

Lateral::Lateral(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-lateral")
{
}

Lateral::~Lateral()
{
}




void Lateral::setUp()
{
  // std::string const exampleConfig = 
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-lateral.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Lateral::tearDown()
{
}

void Lateral::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::proxy::GroundSteeringReading::ID()) {
    auto groundSteeringReading = a_container.getData<opendlv::proxy::GroundSteeringReading>();
    if (isVerbose()) {
      std::cout << "[" << getName() << "] Received: " << groundSteeringReading.toString() << std::endl;
    }
  }
  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Lateral::body()
{
  // Todo: actual steering reading
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    double steeringRequest = 1;
    sendGroundSteeringRequest(steeringRequest);
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Lateral::sendGroundSteeringRequest(double a_steeringRequest)
{
  opendlv::proxy::GroundSteeringRequest gsr(a_steeringRequest);
  odcore::data::Container c(gsr);
  getConference().send(c);

  std::cout << "[" << getName() << "] Sending: " << gsr.toString() << std::endl;
}


}
}
}
}
