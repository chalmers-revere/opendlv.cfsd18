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
, m_coneCollector()
, coneNum()
{
    m_coneCollector = Eigen::MatrixXd::Zero(4,20);
    coneNum = 0;
}

DetectConeLane::~DetectConeLane()
{
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


void DetectConeLane::nextContainer(odcore::data::Container &a_container)
{
    if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID()) {
        auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
	    uint32_t objectId = coneDirection.getObjectId();

        CheckContainer(objectId);
        m_coneCollector(0,objectId) = coneDirection.getAzimuthAngle();
	    m_coneCollector(1,objectId) = coneDirection.getZenithAngle();
	    coneNum++;
	    
	}

	if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()){
        auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
        uint32_t objectId = coneDistance.getObjectId();

        //CheckContainer(objectId);
        m_coneCollector(2,objectId) = coneDistance.getDistance();
	    m_coneCollector(3,objectId) = 0;
	}

}

void DetectConeLane::CheckContainer(uint32_t objectId)
{
	if (objectId == 0){
		rebuildLocalMap();
		m_coneCollector = Eigen::MatrixXd::Zero(4,20);
	    coneNum = 0;
	}
}

// copy from perception-detectcone
Eigen::MatrixXd DetectConeLane::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = MatrixXd::Zero(4,1);
  recievedPoint << xData,
                   yData,
                   zData,
                    0;
  return recievedPoint;
}

ArrayXXf DetectConeLane::findSafeLocalPath(ArrayXXf sidePointsLeft, ArrayXXf sidePointsRight, ArrayXXf vehicleLocation, float distanceBetweenPoints)
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
  for(int j = 0; j < nConesShort; j = j+1)
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
    
    if(shortSide.rows() > 1)
    {
      factor = findFactorToClosestPoint(shortSide.row(0),shortSide.row(1),virtualPointsLong.row(i));
    }



    if(shortSide.rows() > 1 && factor > 0 && factor <= 1)
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
//std::cout << "factor: " << factor << std::endl;
} // End of for

// Match the virtual points from each side into pairs, and find the center of every pair
ArrayXXf midX = (virtualPointsLong.col(0)+virtualPointsShort.col(0))/2;
ArrayXXf midY = (virtualPointsLong.col(1)+virtualPointsShort.col(1))/2;
ArrayXXf tmpLocalPath(nMidPoints,2);
tmpLocalPath.col(0) = midX;
tmpLocalPath.col(1) = midY;

// Do the traceback to the vehicle and then go through the midpoint path again to place path points with equal distances between them.
ArrayXXf firstPoint = DetectConeLane::traceBackToClosestPoint(tmpLocalPath.row(0), tmpLocalPath.row(1), vehicleLocation);
ArrayXXf localPath(nMidPoints+1,2);
localPath.row(0) = firstPoint;
localPath.block(1,0,nMidPoints,2) = tmpLocalPath;
localPath = DetectConeLane::placeEquidistantPoints(localPath,false,-1,distanceBetweenPoints);

//ArrayXXf localPath = tmpLocalPath;
//std::cout << "unused vl:  " << vehicleLocation << std::endl;
//std::cout << "unused dbp:  " << distanceBetweenPoints << std::endl;


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

ArrayXXf DetectConeLane::traceBackToClosestPoint(ArrayXXf p1, ArrayXXf p2, ArrayXXf q)
{
   // Input: The coordinates of the first two points. (row vectors)
   //        A reference point q (vehicle location)
   // Output: the point along the line that is closest to the reference point.

   ArrayXXf v = p1-p2;	// The line to trace
   ArrayXXf n(1,2);	// The normal vector
   n(0,0) = -v(0,1); n(0,1) = v(0,0);
   //float d = (p1(0,0)*v(0,1)-v(0,0)*p1(0,1))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between [0,0] and the vector
   float d = (v(0,1)*(p1(0,0)-q(0,0))+v(0,0)*(q(0,1)-p1(0,1)))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between q and the vector
   return q+n*d;       // Follow the normal vector for that distance
}

ArrayXXf DetectConeLane::orderCones(ArrayXXf cones, ArrayXXf vehicleLocation)
{
  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The cones in order

int nCones = cones.rows();
ArrayXXf current = vehicleLocation;
ArrayXXf found(nCones,1);
found.fill(-1);
ArrayXXf orderedCones(nCones,2);

float shortestDist;
float tmpDist;
int closestConeIndex;

// The first chosen cone is the one closest to the vehicle. After that it continues with the closest neighbour
for(int i = 0; i < nCones; i = i+1)
{
  shortestDist = std::numeric_limits<float>::infinity();
  // Find closest cone to the last chosen cone
  for(int j = 0; j < nCones; j = j+1)
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
found.fill(-1);

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
  for(int j = 0; j < nCones; j = j+1)
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
//std::cout << "Accepted " << j << " at i = " << i << std::endl;
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
//std::cout << "Accepted " << j << " at i = " << i << std::endl;
          } // End of if
        } // End of else
      } // End of if
    } // End of if
  } // End of for

  // If no remaining cone was accepted, the algorithm finishes early
  if(closestConeIndex == -1)
  {
std::cout << "I BREAK NOW" << std::endl;
    break;
  } // End of if

  nAcceptedCones = nAcceptedCones+1;
  found(i) = closestConeIndex;
//std::cout << "found: " << found << std::endl;
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
  for(int j = 0; j < nConesShort; j = j+1)
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

std::cout << "Entered guessCones" << std::endl;

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
  std::cout << "WARNING, ENTERED DANGEROUS AREA IN GUESSCONES " << std::endl;
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
  segLength = ((sidePoints.row(i+1)-sidePoints.row(i)).matrix()).norm();
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

ArrayXXf v = p2-p1; // The line to follow
ArrayXXf n(1,2);    // The normal
n << -v(1),v(0);

float factor = (q(0)-p1(0)+(p1(1)-q(1))*n(0)/n(1))/(v(0)-v(1)*n(0)/n(1));

//std::cout << "p1: " << p1 << std::endl;
//std::cout << "p2: " << p2 << std::endl;
//std::cout << "q: " << q << std::endl;

return factor;
}


void DetectConeLane::rebuildLocalMap()
{
	//Convert to cartesian
	Eigen::MatrixXd cone;
	Eigen::MatrixXd coneLocal = Eigen::MatrixXd::Zero(2,coneNum);
    for(int p = 0; p < coneNum; p++){
        cone = Spherical2Cartesian(m_coneCollector(0,p), m_coneCollector(1,p), m_coneCollector(2,p));
        //m_coneCollector.col(p) = cone;
        coneLocal.col(p) = cone.topRows(2);
    }

    //std::cout << "Cones " << std::endl;
    //std::cout << coneLocal << std::endl;
    
    // the following code only for test
    // manually set left cones and right cones
    int n_left = 0;
    int n_right = 0;
    for(int q = 0; q < coneNum; q++){
        if(coneLocal(0,q) < 0){
            n_left++;
        }else if(coneLocal(0,q) > 0){
        	n_right++;
        }
    }

    if(n_left > 1 || n_right > 1 ){

	    Eigen::MatrixXd coneLeft = Eigen::MatrixXd::Zero(2,n_left);
	    Eigen::MatrixXd coneRight = Eigen::MatrixXd::Zero(2,n_right);
	    int i = 0;
	    int j = 0;

        for(int k = 0; k < coneNum; k++){
            if(coneLocal(0,k) < 0){
                coneLeft.col(i) = coneLocal.col(k);
                i++;
            }else if(coneLocal(0,k) > 0){
                    coneRight.col(j) = coneLocal.col(k);
                    j++;
                }
            }
            /*
            std::cout << "Left " << std::endl;
            std::cout << coneLeft << std::endl;
            std::cout << "Right " << std::endl;
            std::cout << coneRight << std::endl;
            */
// Map with no guesses or filtering
//ArrayXXf sideLeft(4,2); ArrayXXf sideRight(5,2); ArrayXXf location(1,2);
//sideLeft << -68.0726,51.3005,
//            -70.1104,46.7798,
//            -69.1145,48.9532,
//            -66.9452,53.4154;
//sideRight << -65.4468,49.8283,
//             -64.5145,51.6441,
//             -63.5884,52.9191,
//             -66.4618,47.5305,
//             -67.5248,45.2408;
//location << -70,44;

// Map with no filtering but some guesses
//ArrayXXf sideLeft(8,2); ArrayXXf sideRight(4,2); ArrayXXf location(1,2);
//sideLeft << -65.7229,55.0356,
//            -58.4039,60.0614,
//            -63.4316,57.4711,
//            -64.5641,56.3201,
//            -62.3371,58.4472,
//            -57.2672,59.6403,
//            -61.2345,59.2276,
//            -60.1803,59.7912;
//sideRight << -62.4745,54.1608,
//             -63.5884,52.9191,
//             -61.4630,55.2024,
//             -60.5469,56.0359;
//location << -66,54;

// Map with only one detected side
//ArrayXXf sideLeft(0,2); ArrayXXf sideRight(5,2); ArrayXXf location(1,2);
//sideRight << -71.3926,38.6152,
//             -68.7047,42.9798,
//             -67.5248,45.2408,
//             -66.4618,47.5305,
//             -69.9846,40.6632;
//location << -72,38;

// Map with both filtering and guesses (seeing later part of track)
//ArrayXXf sideLeft(4,2); ArrayXXf sideRight(8,2); ArrayXXf location(1,2);
//sideLeft << -77.1638,-19.5275,
//            -74.6719,-22.8084,
//            -76.1332,-20.4146,
//            -75.3298,-21.4609;
//sideRight << -75.3114,-17.1634,
//             -76.8202,-16.2099,
//             -77.9561,-12.1569,
//             -78.1355,-15.3939,
//             -73.8542,-18.4595,
//             -76.8529,-12.6378,
//             -79.4938,-14.6250,
//             -72.6859,-20.0369;
//location << -73,-22;

// Map with both filtering and guesses (seeing earlier part of track)
//ArrayXXf sideLeft(4,2); ArrayXXf sideRight(9,2); ArrayXXf location(1,2);
//sideLeft << -79.0031,-7.1247,
//            -83.4068,-15.5843,
//            -79.2091,-5.7584,
//            -82.4479,-16.2767;
//sideRight << -79.8760,-11.2204,
//             -81.7276,-13.0964,
//             -80.5293,-13.9657,
//             -81.8028,-8.2125,
//             -82.2223,-5.6942,
//             -79.4938,-14.6250,
//             -78.8625,-11.7738,
//             -82.6931,-12.5203,
//             -81.1177,-9.6574;
//location << -81,-5;

// Map with a guess where cones are placed poorly
/*
ArrayXXf sideLeft(6,2); ArrayXXf sideRight(7,2); ArrayXXf location(1,2);
sideLeft << -55.3944,58.0601,
            -60.1803,59.7912,
            -57.2672,59.6403,
            -58.4039,60.0614,
            -61.2345,59.2276,
            -56.4540,59.3048;
sideRight << -58.1339,56.8301,
             -57.8483,56.0774,
             -59.8076,56.5859,
             -58.1000,56.7947,
             -59.0203,57.0216,
             -59.0262,57.1244,
             -58.7363,57.0246;
location << -61,57;
*/

ArrayXXf location(1,2);
location << 0,0;


MatrixXf coneLeft_f = coneLeft.cast <float> ();
MatrixXf coneRight_f = coneRight.cast <float> (); 

ArrayXXf sideLeft = coneLeft_f.transpose().array();
ArrayXXf sideRight = coneRight_f.transpose().array();



float distanceThreshold = 3.5;
float guessDistance = 3.0;


// Actual test
std::cout << "SideLeft: " << sideLeft << std::endl;
std::cout << "SideRight: " << sideRight << std::endl;
std::cout << "Location: " << location << std::endl;

ArrayXXf orderedConesLeft = DetectConeLane::orderAndFilterCones(sideLeft,location);
ArrayXXf orderedConesRight = DetectConeLane::orderAndFilterCones(sideRight,location);
std::cout << "orderedConesLeft: " << orderedConesLeft << std::endl;
std::cout << "orderedConesRight: " << orderedConesRight << std::endl;

float pathLengthLeft = DetectConeLane::findTotalPathLength(orderedConesLeft);
float pathLengthRight = DetectConeLane::findTotalPathLength(orderedConesRight);
std::cout << "pathLengthLeft: " << pathLengthLeft << std::endl;
std::cout << "pathLengthRight: " << pathLengthRight << std::endl;

ArrayXXf longSide;
ArrayXXf shortSide;
if(pathLengthLeft > pathLengthRight)
{
std::cout << "first if: " << std::endl;
  ArrayXXf tmpLongSide = orderedConesLeft;
  ArrayXXf tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesLeft, orderedConesRight, location, distanceThreshold,  guessDistance, false);

  //if(tmpLongSide.rows() != longSide.rows())
  //{
std::cout << "Long size before: " << longSide.rows() << " " << longSide.cols() << std::endl;
    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
std::cout << "Long size after: " << longSide.rows() << " " << longSide.cols() << std::endl;
  //} // End of if

  longSide = tmpLongSide;

  //if(tmpShortSide.rows() != shortSide.rows())
  //{
std::cout << "Short size before: " << shortSide.rows() << " " << shortSide.cols() << std::endl;
    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
std::cout << "Short size after: " << shortSide.rows() << " " << shortSide.cols() << std::endl;
  //} // End of if

  shortSide = tmpShortSide;
}
else
{
std::cout << "second if: " << std::endl;
  ArrayXXf tmpLongSide = orderedConesRight;
  ArrayXXf tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesRight, orderedConesLeft, location, distanceThreshold,  guessDistance, true);
std::cout << "got through first bit " << std::endl;

  //if(tmpLongSide.rows() != longSide.rows())
  //{
std::cout << "Long size before: " << longSide.rows() << " " << longSide.cols() << std::endl;
    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
std::cout << "Long size after: " << longSide.rows() << " " << longSide.cols() << std::endl;
  //} // End of if

  longSide = tmpLongSide;

  //if(tmpShortSide.rows() != shortSide.rows())
  //{
std::cout << "Short size before: " << shortSide.rows() << " " << shortSide.cols() << std::endl;
    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
std::cout << "Short size after: " << shortSide.rows() << " " << shortSide.cols() << std::endl;
  //} // End of if

  shortSide = tmpShortSide;
} // End of else
std::cout << "longSide: " << longSide << std::endl;
std::cout << "shortSide: " << shortSide << std::endl;


ArrayXXf localPath;


if(longSide.rows() > 1)
{
    localPath = DetectConeLane::findSafeLocalPath(longSide, shortSide, location, 0.5);
    std::cout << "localPath: " << localPath << std::endl;
}
else
{
	// only for dummy test
    localPath.resize(0,2);
}


//std::cout << "DETECTCONELANE IS SENDING SURFACE" << std::endl;
//opendlv::logic::perception::Surface o9;
//    o9.setSurfaceId(12345);
//    odcore::data::Container c9(o9);
//    getConference().send(c9);

/*

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
 */




    }
    
}    

    





}
}
}
}

