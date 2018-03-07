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

//std::cout << "DETECTCONELANE IS SENDING SURFACE" << std::endl;
//opendlv::logic::perception::Surface o9;
//    o9.setSurfaceId(12345);
//    odcore::data::Container c9(o9);
//    getConference().send(c9);


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

ArrayXXf DetectConeLane::findSafeLocalPath(ArrayXXf sidePointsLeft, ArrayXXf sidePointsRight, float distanceBetweenPoints)
{

ArrayXXf longSide;
ArrayXXf shortSide;

// Identify the longest side
float pathLengthLeft = findTotalPathLength(sidePointsLeft);
float pathLengthRight = findTotalPathLength(sidePointsRight);
if(pathLengthLeft > pathLengthRight)
{
  longSide = sidePointsLeft;
  shortSide = sidePointsRight;
}
else
{
  longSide = sidePointsRight;
  shortSide = sidePointsLeft;
} // End of else

int nMidPoints = longSide.rows()*3; // We might have to choose this more carefully
int nConesShort = shortSide.rows();

// Divide the longest side into segments of equal length
ArrayXXf virtualPointsLong = placeEquidistantPoints(longSide,true,nMidPoints,-1);
ArrayXXf virtualPointsShort(nMidPoints,2);

float shortestDist;
float tmpDist;
int closestConeIndex;
float factor;

// Every virtual point on the long side should get one corresponding point on the short side
for(int i = 0; i < nMidPoints; i = i+1)
{
  // Find short side cone that is closest to the current long side point
  shortestDist = std::numeric_limits<float>::infinity();
  for(int j = 0; j < nConesShort; i = i+1)
  {
    tmpDist = ((shortSide.row(j)-virtualPointsLong.row(i)).matrix()).norm();
    if(tmpDist < shortestDist)
    {
      shortestDist = tmpDist;
      closestConeIndex = j;
    } // End of if
  } // End of for

  // Check if one of the two segments next to the cone has a perpendicular line to the long side point. If so, place the short side point
  // on that place of the segment. If not, place the point on the cone. If it's the first or last cone there is only one segment to check
  if(closestConeIndex == 0)
  {
    factor = findFactorToClosestPoint(shortSide.row(0),shortSide.row(1),virtualPointsLong.row(i));
    if(factor > 0 && factor <= 1)
    {
      virtualPointsShort.row(i) = shortSide.row(0)+factor*(shortSide.row(1)-shortSide.row(0));
    }
    else
    {
      virtualPointsShort.row(i) = shortSide.row(0);
    } // End of else
  }
  else if(closestConeIndex == nConesShort-1)
  {
    factor = findFactorToClosestPoint(shortSide.row(nConesShort-2),shortSide.row(nConesShort-1),virtualPointsLong.row(i));
    if(factor > 0 && factor <= 1)
    {
      virtualPointsShort.row(i) = shortSide.row(nConesShort-2)+factor*(shortSide.row(nConesShort-1)-shortSide.row(nConesShort-2));
    }
    else
    {
      virtualPointsShort.row(i) = shortSide.row(nConesShort-1);
    } // End of else
  }
  else
  {
    factor = findFactorToClosestPoint(shortSide.row(closestConeIndex-1),shortSide.row(closestConeIndex),virtualPointsLong.row(i));
    if(factor > 0 && factor <= 1)
    {
      virtualPointsShort.row(i) = shortSide.row(closestConeIndex-1)+factor*(shortSide.row(closestConeIndex)-shortSide.row(closestConeIndex-1));
    }
    else
    {
      factor = findFactorToClosestPoint(shortSide.row(closestConeIndex),shortSide.row(closestConeIndex+1),virtualPointsLong.row(i));
      if(factor > 0 && factor <= 1)
      {
        virtualPointsShort.row(i) = shortSide.row(closestConeIndex)+factor*(shortSide.row(closestConeIndex+1)-shortSide.row(closestConeIndex));
      }
      else
      {
        virtualPointsShort.row(i) = shortSide.row(closestConeIndex);
      } // End of else
    } // End of else
  } // End of else
} // End of for

// Match the virtual points from each side into pairs, and find the center of every pair
ArrayXXf midX = (virtualPointsLong.col(0)+virtualPointsShort.col(0))/2;
ArrayXXf midY = (virtualPointsLong.col(1)+virtualPointsShort.col(1))/2;
ArrayXXf tmpLocalPath(nMidPoints,2);
tmpLocalPath.col(0) = midX;
tmpLocalPath.col(1) = midY;

// Do the traceback to the vehicle and then go through the midpoint path again to place path points with equal distances between them.
ArrayXXf firstPoint = DetectConeLane::traceBackToClosestPoint(tmpLocalPath.row(0), tmpLocalPath.row(1));
ArrayXXf localPath(nMidPoints+1,2);
localPath.row(0) = firstPoint;
localPath.block(1,0,nMidPoints,2) = tmpLocalPath;
localPath = DetectConeLane::placeEquidistantPoints(localPath,false,-1,distanceBetweenPoints);

return localPath;
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

ArrayXXf DetectConeLane::orderCones(ArrayXXf cones, ArrayXXf vehicleLocation)
{
  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The cones in order

int nCones = cones.rows();
ArrayXXf current = vehicleLocation;
ArrayXXf found(nCones,1);
ArrayXXf orderedCones(nCones,2);

float shortestDist;
float tmpDist;
int closestConeIndex;

// The first chosen cone is the one closest to the vehicle. After that it continues with the closest neighbour
for(int i = 0; i < nCones; i = i+1)
{
  shortestDist = std::numeric_limits<float>::infinity();
  // Find closest cone to the last chosen cone
  for(int j = 0; i < nCones; i = i+1)
  {
    if(!((found==j).any()))
    {
      tmpDist = ((current-cones.row(j)).matrix()).norm();
      if(tmpDist < shortestDist)
      {
        shortestDist = tmpDist;
        closestConeIndex = j;
      } // End of if
    } // End of if
  } // End of for

  found(i) = closestConeIndex;
  current = cones.row(closestConeIndex);
} // End of for

// Rearrange cones to have the order of found
for(int i = 0; i < nCones; i = i+1)
{
  orderedCones.row(i) = cones.row(found(i));
}

//std::cout << "cones: " << cones << std::endl;
//std::cout << "vehicleLocation: " << vehicleLocation << std::endl;

return orderedCones;
}

ArrayXXf DetectConeLane::orderAndFilterCones(ArrayXXf cones, ArrayXXf vehicleLocation)
{
  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The cones that satisfy some requirements, in order

int nCones = cones.rows();
ArrayXXf current = vehicleLocation;
ArrayXXf found(nCones,1);

float shortestDist;
float tmpDist;
int closestConeIndex;
float line1;
float line2;
float line3;
float angle;
int nAcceptedCones = 0;

float orderingDistanceThreshold = 5.5; // There might be an alternative to hard coding this...
const float PI = 3.14159265;

for(int i = 0; i < nCones; i = i+1)
{
  shortestDist = std::numeric_limits<float>::infinity();
  closestConeIndex = -1;
  // Find closest cone to the last chosen cone
  for(int j = 0; i < nCones; i = i+1)
  {
    if(!((found==j).any()))
    {
      tmpDist = ((current-cones.row(j)).matrix()).norm();
      if(tmpDist < shortestDist && tmpDist < orderingDistanceThreshold)
      {
        // If it's one of the first two cones, the nearest neighbour is accepted
        if(i < 2)
        {
          shortestDist = tmpDist;
          closestConeIndex = j;
        }
        // Otherwise the nearest neighbour needs to be considered to be placed in roughly the same direction as the two previous cones
        // i.e the angle between the previous line and the next must be larger than pi/2 (it has a forward going component)
        else
        {
          // The angle is found with the cosine rule
          line1 = ((cones.row(found(i-2))-cones.row(found(i-1))).matrix()).norm();
          line2 = ((cones.row(found(i-1))-cones.row(j)).matrix()).norm();
          line3 = ((cones.row(j)-cones.row(found(i-2))).matrix()).norm();
          angle = std::acos((float)(-std::pow(line3,2)+std::pow(line2,2)+std::pow(line1,2))/(2*line2*line1));

          if(std::abs(angle) > PI/2)
          {
            shortestDist = tmpDist;
            closestConeIndex = j;
          } // End of if
        } // End of else
      } // End of if
    } // End of if
  } // End of for
  
  // If no remaining cone was accepted, the algorithm finishes early
  if(closestConeIndex == -1)
  {
    break;
  } // End of if

  nAcceptedCones = nAcceptedCones+1;
  found(i) = closestConeIndex;
  current = cones.row(closestConeIndex);
} // End of for

// Rearrange cones to have the order of found
ArrayXXf orderedCones(nAcceptedCones,2);
for(int i = 0; i < nAcceptedCones; i = i+1)
{
  orderedCones.row(i) = cones.row(found(i));
}

//std::cout << "cones: " << cones << std::endl;
//std::cout << "vehicleLocation: " << vehicleLocation << std::endl;

return orderedCones;
}

ArrayXXf DetectConeLane::insertNeededGuessedCones(ArrayXXf longSide, ArrayXXf shortSide, ArrayXXf vehicleLocation, float distanceThreshold, float guessDistance, bool guessToTheLeft)
{
  // Input: Both cone sides, vehicle position, two distance values and if the guesses should be on the left side
  // Output: The new ordered short side with mixed real and guessed cones

int nConesLong = longSide.rows();
int nConesShort = shortSide.rows();
ArrayXXf guessedCones(2*nConesLong-2,2); // 2n-2 is the number of guesses if all known cones need guessed matches

float shortestDist;
float tmpDist;
ArrayXXf guess(1,2);
int nGuessedCones = 0;

// Every long side cone should search for a possible match on the other side
for(int i = 0; i < nConesLong; i = i+1)
{
  shortestDist = std::numeric_limits<float>::infinity();
  // Find closest cone on the other side
  for(int j = 0; j < nConesShort; i = i+1)
  {
    tmpDist = ((longSide.row(i)-shortSide.row(j)).matrix()).norm();
    if(tmpDist < shortestDist)
    {
      shortestDist = tmpDist;
    } // End of if
  } // End of for
  
  // If the closest cone is not valid, create cone guesses perpendicular to both segments connected to the current cone.
  // If it's the first or last cone, there is only on segment available.
  if(shortestDist > distanceThreshold)
  {
    if(i == 0)
    {
      guess = guessCones(longSide.row(0),longSide.row(1),guessDistance,guessToTheLeft,true,false);
      nGuessedCones = nGuessedCones+1;
    }
    else if(i == nConesLong-1)
    {
      guess = guessCones(longSide.row(nConesLong-2),longSide.row(nConesLong-1),guessDistance,guessToTheLeft,false,true);
      nGuessedCones = nGuessedCones+1;
    }
    else
    {
      guess = guessCones(longSide.row(i-1),longSide.row(i),guessDistance,guessToTheLeft,false,true);
      nGuessedCones = nGuessedCones+1;
      guessedCones.row(nGuessedCones-1) = guess;
      guess = guessCones(longSide.row(i),longSide.row(i+1),guessDistance,guessToTheLeft,true,false);
      nGuessedCones = nGuessedCones+1;
    } // End of else

    guessedCones.row(nGuessedCones-1) = guess;
  } // End of if
} // End of for

// Collect real and guessed cones in the same array, and order them
ArrayXXf guessedConesFinal = guessedCones.topRows(nGuessedCones);
ArrayXXf realAndGuessedCones(nConesShort+nGuessedCones,2);
realAndGuessedCones.topRows(nConesShort) = shortSide;
realAndGuessedCones.bottomRows(nGuessedCones) = guessedConesFinal;
ArrayXXf newShortSide = orderCones(realAndGuessedCones,vehicleLocation);

//std::cout << "longSide:  " << longSide << std::endl;
//std::cout << "shortSide:  " << shortSide << std::endl;
//std::cout << "vehicleLocation:  " << vehicleLocation << std::endl;
//std::cout << "distanceThreshold:  " << distanceThreshold << std::endl;
//std::cout << "guessDistance:  " << guessDistance << std::endl;
//std::cout << "guessToTheLeft:  " << guessToTheLeft << std::endl;

return newShortSide;
}

ArrayXXf DetectConeLane::guessCones(ArrayXXf firstCone, ArrayXXf secondCone, float guessDistance, bool guessToTheLeft, bool guessForFirstCone, bool guessForSecondCone)
{
  // Input: Two neighbouring cones, the guessing distance, if guesses should go to the left or not, and which known cones should
  // get matching guesses
  // Output: Guessed cone positions

ArrayXXf vector = secondCone-firstCone;
float direction;
if(guessToTheLeft)
{
  direction = 1.0;
}
else
{
  direction = -1.0;
} // End of else

ArrayXXf normal(1,2);
normal << -vector(1),vector(0);
normal = normal/((normal.matrix()).norm());
ArrayXXf guessVector = direction*guessDistance*normal;

// The guess is placed a guessVector away from the cone that should get a mathing guess
ArrayXXf guessedCones(1,2);
if(guessForFirstCone && !guessForSecondCone)
{
  guessedCones << firstCone(0)+guessVector(0),firstCone(1)+guessVector(1);
}
else if(!guessForFirstCone && guessForSecondCone)
{
  guessedCones << secondCone(0)+guessVector(0),secondCone(1)+guessVector(1);
}
else
{
  // Should not be in use. Works in matlab but not here were array sizes have to be defined in advance? Throw exception if this is reached?
  guessedCones << -1000,-1000;
} // End of else

//std::cout << "firstCone: " << firstCone << std::endl;
//std::cout << "secondCone: " << secondCone << std::endl;
//std::cout << "guessDistance: " << guessDistance << std::endl;
//std::cout << "guessToTheLeft: " << guessToTheLeft << std::endl;
//std::cout << "guessForFirstCone: " << guessForFirstCone << std::endl;
//std::cout << "guessForSecondCone: " << guessForSecondCone << std::endl;

return guessedCones;
}

float DetectConeLane::findTotalPathLength(ArrayXXf sidePoints)
{
  // Input: Cone positions
  // Output: Total length of cone sequence

int nCones = sidePoints.rows();
float pathLength = 0;

float segLength;
for(int i = 0; i < nCones-1; i = i+1)
{
  segLength = ((sidePoints.row(i+1)-sidePoints(i)).matrix()).norm();
  pathLength = pathLength + segLength;
}

// std::cout << "sidePoints: " << sidePoints << std::endl;

return pathLength;
}

float DetectConeLane::findFactorToClosestPoint(ArrayXXf p1, ArrayXXf p2, ArrayXXf q)
{
  // Input: The two cones of a cone segment and a reference point
  // Output: The factor to multiply with the vector between the cones in order to reach the point on the segment that has a
  // perpendicular line to the reference point

ArrayXXf v = p1-p2; // The line to follow
ArrayXXf n(1,2);    // The normal
n << -v(1),v(0);

float factor = (q(0)-p1(0)+(p1(1)-q(1))*n(0)/n(1))/(v(0)-v(1)*n(0)/n(1));

//std::cout << "p1: " << p1 << std::endl;
//std::cout << "p2: " << p2 << std::endl;
//std::cout << "q: " << q << std::endl;

return factor;
}

}
}
}
}
