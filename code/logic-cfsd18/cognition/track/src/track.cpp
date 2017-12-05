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

#include "track.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace cognition {

Track::Track(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-cognition-track")
{
}

Track::~Track()
{
}



void Track::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::logic::perception::Surface::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();

    opendlv::logic::action::AimPoint o1;
    odcore::data::Container c1(o1);
    getConference().send(c1);

    opendlv::logic::action::PreviewPoint o2;
    odcore::data::Container c2(o2);
    getConference().send(c2);

    opendlv::logic::cognition::GroundSpeedLimit o3;
    odcore::data::Container c3(o3);
    getConference().send(c3);
  }
  if (a_container.getDataType() == opendlv::system::SignalStatusMessage::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::system::SystemOperationState::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::system::NetworkStatusMessage::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
}

void Track::setUp()
{
  // std::string const exampleConfig = 
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-cognition-track.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Track::tearDown()
{
}

ArrayXXf Track::findSafeLocalPath(ArrayXXf sidePoints1, ArrayXXf sidePoints2, int nMidPoints)
{
  ArrayXXf newSidePoints1 = Track::placeEquidistantPoints(sidePoints1,nMidPoints);
  ArrayXXf newSidePoints2 = Track::placeEquidistantPoints(sidePoints2,nMidPoints);

  ArrayXXf midX = (newSidePoints1.col(1)+newSidePoints2.col(1))/2;
  ArrayXXf midY = (newSidePoints1.col(2)+newSidePoints2.col(2))/2;

  ArrayXXf localPath(nMidPoints,2);
  localPath.col(1) = midX;
  localPath.col(2) = midY;
  return localPath;

//  std::cout << "One side:  " << sidePoints1 << std::endl;
//  std::cout << "Other side:  " << sidePoints2 << std::endl;
//  ArrayXXf localPath(nMidPoints,2);
//  return localPath;
}

ArrayXXf Track::placeEquidistantPoints(ArrayXXf sidePoints, int nEqPoints)
{
  std::cout << "nPoints:  " << nEqPoints << std::endl;
  return sidePoints;
}

}
}
}
}
