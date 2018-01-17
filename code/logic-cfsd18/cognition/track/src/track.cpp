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
  int nCones = sidePoints.rows();

  // Full path length, and save lengths of individual segments
  float pathLength = 0;
  ArrayXXf segLength(nCones-1,1);
  for(int i = 0; i < nCones-1; i = i+1)
  {
    segLength(i) = ((sidePoints.row(i+1)-sidePoints.row(i)).matrix()).norm();
    pathLength = pathLength + segLength(i);
  }
  // Calculate equal subdistances
  int eqDistance = pathLength/(nEqPoints-1);

  // The latest point that you placed
  ArrayXXf latestPointCoords = sidePoints.row(0);
  // The latest cone that you passed
  int latestConeIndex = 0;
  // How long is left of the current segment
  float remainderOfSeg = segLength(0);
  // The new list of lineraly equidistant points
  ArrayXXf newSidePoints(nEqPoints,2);
  // The first point should be at the same place as the first cone
  newSidePoints.row(0) = latestPointCoords;

  // A temporary vector
  ArrayXXf vec(1,2);
  // Temporary distances
  float distanceToGoFromLatestPassedPoint, lengthOfNextSeg;
  // Start stepping through the given path
  for(int i = 1; i < nEqPoints-1; i = i+1)
  {
    // If the next point should be in the segment you are currently in, simply place it.
    if(remainderOfSeg > eqDistance)
    {
      vec = sidePoints.row(latestConeIndex+1)-latestPointCoords;
      latestPointCoords = latestPointCoords + (eqDistance/remainderOfSeg)*vec;
    } 
    else // If you need to go to the next segment, keep in mind which cones you pass and how long distance you have left to go.
    {
      latestConeIndex = latestConeIndex+1;
      distanceToGoFromLatestPassedPoint = eqDistance-remainderOfSeg;
      lengthOfNextSeg = segLength(latestConeIndex);

      while(distanceToGoFromLatestPassedPoint > lengthOfNextSeg)
      {
        latestConeIndex = latestConeIndex+1;
        distanceToGoFromLatestPassedPoint = distanceToGoFromLatestPassedPoint - lengthOfNextSeg;
        lengthOfNextSeg = segLength(latestConeIndex);
      } // End of while

      latestPointCoords = sidePoints.row(latestConeIndex);
      vec = sidePoints.row(latestConeIndex+1)-latestPointCoords;
      latestPointCoords = latestPointCoords + (distanceToGoFromLatestPassedPoint/segLength(latestConeIndex))*vec;
    } // End of else

    // In every case, save the point you just placed and check how much of that segment is left.
    newSidePoints.row(i) = latestPointCoords;
    remainderOfSeg = ((sidePoints.row(latestConeIndex+1)-latestPointCoords).matrix()).norm();
  } // End of for

  // The last point should be at the same place as the last cone.
  newSidePoints.row(nEqPoints-1) = sidePoints.row(nCones-1);

  //std::cout << "eqDistance:  " << eqDistance << std::endl;
  //std::cout << "latestConeIndex:  " << latestConeIndex << std::endl;
  //std::cout << "remainderOfSeg:  " << remainderOfSeg << std::endl;
  return newSidePoints;
}

ArrayXXf Track::traceBackToClosestPoint(ArrayXXf p1, ArrayXXf p2)
{
   // Input: The coordinates of the first two points. (column vectors)
   // Output: the point along the line that is closest to the origin.

   ArrayXXf v = p1-p2;	// The line to trace
   ArrayXXf n(2,1);	// The normal vector
   n(0,0) = -v(1,0); n(1,0) = v(0,0);
   int d = (p1(0,0)*v(1,0)-v(0,0)*p1(1,0))/(n(0,0)*v(1,0)-v(0,0)*n(1,0)); // Shortest distance between [0,0] and the vector
   return n*d;		// Follow the normal vector for that distance
}

}
}
}
}
