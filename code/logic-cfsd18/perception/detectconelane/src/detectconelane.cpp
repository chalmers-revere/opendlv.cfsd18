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
//// Map 4
//ArrayXXf side1(3,2); ArrayXXf side2(3,2);
//side1 << -1,2,
//	 0,2.5,
//	 0,3;
//side2 << 1,1.5,
//	 2,2,
//	 2,2.5;

//// Map 3
//ArrayXXf side1(9,2); ArrayXXf side2(8,2);
//side1 << -0.8, 0,
//	 -0.85, 1,
//	 -0.85,2,
//	 -0.8, 3,
//	 -0.72, 4.3,
//	 -0.5, 4.8,
//	 -0.4, 4.8,
//	 -0.1, 4,
//	 0, 2;
//side2 << -0.5, 0,
//	 -0.6, 1,
//	 -0.6, 2,
//	 -0.53, 3,
//	 -0.43, 3.6,
//	 -0.35, 3,
//	 -0.35, 2,
//	 -0.35, 1;

//// Actual test
//std::cout << "Side1: " << side1 << std::endl;
//std::cout << "Side2: " << side2 << std::endl;
//ArrayXXf localPath = DetectConeLane::findSafeLocalPath(side1, side2, 0.5);
//std::cout << "localPath: " << localPath << std::endl;


  if (a_container.getDataType() == opendlv::logic::perception::Object::ID()) {
    auto object = a_container.getData<opendlv::logic::perception::Object>();
    auto objectId = object.getObjectId();
    std::cout << "[cognition] DETECTCONELANE IS RECIEVING OBJECT " << objectId << std::endl;
    opendlv::logic::perception::Surface o1;
    o1.setSurfaceId(12345);
    odcore::data::Container c1(o1);
    std::cout << "[cognition] DETECTCONELANE IS SENDING SURFACE 12345 " << std::endl;
    getConference().send(c1);
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

ArrayXXf DetectConeLane::findSafeLocalPath(ArrayXXf sidePoints1, ArrayXXf sidePoints2, float distanceBetweenPoints)
{
  const int nMidPoints = 30; // There might be an alternative to "hard-coding" this.
  ArrayXXf newSidePoints1 = DetectConeLane::placeEquidistantPoints(sidePoints1,true,nMidPoints,-1);
  ArrayXXf newSidePoints2 = DetectConeLane::placeEquidistantPoints(sidePoints2,true,nMidPoints,-1);

  ArrayXXf midX = (newSidePoints1.col(0)+newSidePoints2.col(0))/2;
  ArrayXXf midY = (newSidePoints1.col(1)+newSidePoints2.col(1))/2;
  ArrayXXf tmpLocalPath(nMidPoints,2);
  tmpLocalPath.col(0) = midX;
  tmpLocalPath.col(1) = midY;

  ArrayXXf firstPoint = DetectConeLane::traceBackToClosestPoint(tmpLocalPath.row(0), tmpLocalPath.row(1));
  ArrayXXf localPath(nMidPoints+1,2);
  localPath.row(0) = firstPoint;
  localPath.block(1,0,nMidPoints,2) = tmpLocalPath;
  localPath = DetectConeLane::placeEquidistantPoints(localPath,false,-1,distanceBetweenPoints);
  return localPath;

//  std::cout << "One side:  " << sidePoints1 << std::endl;
//  std::cout << "Other side:  " << sidePoints2 << std::endl;
//  ArrayXXf localPath(nMidPoints,2);
//  return localPath;
}

ArrayXXf DetectConeLane::placeEquidistantPoints(ArrayXXf sidePoints, bool nEqPointsIsKnown, int nEqPoints, float eqDistance)
{
// Places linearly equidistant points along a sequence of points.
// If nEqPoints is known it will not use the input value for eqDistance, and instead calculate a suitable value.
// If nEqPoints is not known it will not use the input value for nEqPoints, and instead calculate a suitable value.

  int nCones = sidePoints.rows();

  // Full path length, and save lengths of individual segments
  float pathLength = 0;
  ArrayXXf segLength(nCones-1,1);
  for(int i = 0; i < nCones-1; i = i+1)
  {
    segLength(i) = ((sidePoints.row(i+1)-sidePoints.row(i)).matrix()).norm();
    pathLength = pathLength + segLength(i);
  }

  if(nEqPointsIsKnown)
  {
    // Calculate equal subdistances
    eqDistance = pathLength/(nEqPoints-1);
  }
  else
  {
    //Calculate how many points will fit
    nEqPoints = std::ceil(pathLength/eqDistance)+1;
  }

  // The latest point that you placed
  ArrayXXf latestPointCoords = sidePoints.row(0);
  // The latest cone that you passed
  int latestConeIndex = 0;
  // How long is left of the current segment
  float remainderOfSeg = segLength(0);
  // The new list of linearly equidistant points
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

ArrayXXf DetectConeLane::traceBackToClosestPoint(ArrayXXf p1, ArrayXXf p2)
{
   // Input: The coordinates of the first two points. (row vectors)
   // Output: the point along the line that is closest to the origin.

   ArrayXXf v = p1-p2;	// The line to trace
   ArrayXXf n(1,2);	// The normal vector
   n(0,0) = -v(0,1); n(0,1) = v(0,0);
   float d = (p1(0,0)*v(0,1)-v(0,0)*p1(0,1))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between [0,0] and the vector
   return n*d;		// Follow the normal vector for that distance
}

}
}
}
}
