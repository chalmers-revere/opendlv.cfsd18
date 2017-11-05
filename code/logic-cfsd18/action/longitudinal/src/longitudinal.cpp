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

#include "longitudinal.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

Longitudinal::Longitudinal(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-longitudinal")
{
}

Longitudinal::~Longitudinal()
{
}



void Longitudinal::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::logic::cognition::GroundSpeedLimit::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::logic::action::PreviewPoint::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();

    opendlv::proxy::GroundAccelerationRequest o1;
    odcore::data::Container c1(o1);
    getConference().send(c1);

    opendlv::proxy::GroundDecelerationRequest o2;
    odcore::data::Container c2(o2);
    getConference().send(c2);
  }
}

void Longitudinal::setUp()
{
  // std::string const exampleConfig = 
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-longitudinal.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Longitudinal::tearDown()
{
}

}
}
}
}
