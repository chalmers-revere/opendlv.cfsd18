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

#include "detectconelane.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

DetectConeLane::DetectConeLane(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-perception-detectconelane")
{
}

DetectConeLane::~DetectConeLane()
{
}



void DetectConeLane::nextContainer(odcore::data::Container &a_container)
{
  std::cout << "I am in DetectConeLane!" << std::endl;
  if (a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()) {
  	std::cout << "I am in DetectConeLane!I received ObjectDistance!!!" << std::endl;
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();

    //std::cout << "I am in DetectConeLane!" << std::endl;
    //opendlv::logic::perception::Surface o1;
    //odcore::data::Container c1(o1);
    //getConference().send(c1);
  }
}

void DetectConeLane::setUp()
{
  // std::string const exampleConfig = 
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-perception-detectconelane.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void DetectConeLane::tearDown()
{
}

}
}
}
}
