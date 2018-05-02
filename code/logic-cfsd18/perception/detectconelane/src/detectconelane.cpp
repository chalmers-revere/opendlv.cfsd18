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
, m_lastTimeStamp()
, m_coneCollector()
, m_coneMutex()
, m_newFrame()
, m_timeDiffMilliseconds()
, m_lastTypeId()
{
    m_coneCollector = Eigen::MatrixXd::Zero(4,20);
    m_newFrame = true;
    m_timeDiffMilliseconds = 150;
    m_lastTypeId = -1;
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
/*
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
*/

if(a_container.getDataType() == opendlv::logic::perception::Object::ID()){
    //std::cout << "RECIEVED AN OBJECT!" << std::endl;
    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneObject = a_container.getData<opendlv::logic::perception::Object>();
    int conesInFrame = coneObject.getObjectId();

    //std::cout << "FRAME: " << m_newFrame << std::endl;
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (m_newFrame){
       m_newFrame = false;
       std::thread coneCollector(&DetectConeLane::initializeCollection, this, conesInFrame);
       coneCollector.detach();
       //initializeCollection();
    }

  }


if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID()) {
    //std::cout << "Recieved Direction" << std::endl;
    //Retrive data and timestamp
    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
    uint32_t objectId = coneDirection.getObjectId();
  //bool newFrameDir = false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      //std::cout << "Message Recieved " << std::endl;
      m_coneCollector(0,objectId) = coneDirection.getAzimuthAngle();
      m_coneCollector(1,objectId) = coneDirection.getZenithAngle();

	//std::cout << "FRAME BEFORE LOCAL: " << m_newFrame << std::endl;
    //newFrameDir = m_newFrame;
    }

	//std::cout << "FRAME: " << m_newFrame << std::endl;
   /* if (newFrameDir){

      std::thread coneCollector (&DetectConeLane::initializeCollection,this);
      coneCollector.detach();

    }
   */
}

else if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()){

    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
    uint32_t objectId = coneDistance.getObjectId();
  //bool newFrameDist = false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      m_coneCollector(2,objectId) = coneDistance.getDistance();

	//std::cout << "FRAME BEFORE LOCAL: " << m_newFrame << std::endl;
    //newFrameDist = m_newFrame;
    }

    //std::cout << "FRAME: " << m_newFrame << std::endl;
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
   /* if (newFrameDist){
       std::thread coneCollector(&DetectConeLane::initializeCollection, this);
       coneCollector.detach();
       //initializeCollection();
    }
   */
  }

  else if(a_container.getDataType() == opendlv::logic::perception::ObjectType::ID()){

    //std::cout << "Recieved Type" << std::endl;
    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneType = a_container.getData<opendlv::logic::perception::ObjectType>();
    int objectId = coneType.getObjectId();
  //bool newFrameType =false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      m_lastTypeId = (m_lastTypeId<objectId)?(objectId):(m_lastTypeId);
      auto type = coneType.getType();
      m_coneCollector(3,objectId) = type;

    //newFrameType = m_newFrame;
    }

    //std::cout << "FRAME: " << m_newFrame << std::endl;
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
   /* if (newFrameType){
      std::thread coneCollector (&DetectConeLane::initializeCollection,this); //just sleep instead maybe since this is unclear how it works
      coneCollector.detach();
      //initializeCollection();

    }
   */
}

}

void DetectConeLane::initializeCollection(int conesInFrame){
  //std::this_thread::sleep_for(std::chrono::duration 1s); //std::chrono::milliseconds(m_timeDiffMilliseconds)
  bool sleep = true;
  //auto start = std::chrono::system_clock::now();
  //std::cout << "m_lastTypeId1: " << m_lastTypeId<< std::endl;
  //std::cout << "conesInFrame1: " << conesInFrame<< std::endl;
  while(sleep) // Can probably be rewritten nicer
  {
  /*  auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
    if ( elapsed.count() > m_timeDiffMilliseconds*1000 )
        sleep = false;
  */
    if (m_lastTypeId >= conesInFrame-1)
        sleep = false;
  }
  //std::cout << "m_lastTypeId2: " << m_lastTypeId<< std::endl;
  //std::cout << "conesInFrame2: " << conesInFrame<< std::endl;

  Eigen::MatrixXd extractedCones;
  int nLeft = 0;
  int nRight = 0;
  int nSmall = 0;
  int nBig = 0;

  {
    odcore::base::Lock lockCone(m_coneMutex);

    extractedCones = m_coneCollector.leftCols(m_lastTypeId+1);
    for (int i = 0; i < extractedCones.cols(); i++) {
      int type = extractedCones(3,i);
      if(type == 1){ nLeft++; }
      else if(type == 2){ nRight++; }
      else if(type == 3){ nSmall++; }
      else if(type == 4){ nBig++; }
      else
      {
        std::cout << "WARNING! Object " << i << " has invalid cone type: " << type << std::endl;
      }
    }
    //std::cout << "members: " << nLeft << " " << nRight << " " << nSmall << " " << nBig << std::endl;
    m_coneCollector = Eigen::MatrixXd::Zero(4,20);
    m_lastTypeId = -1;
    m_newFrame = true;
  }
  //Initialize for next collection
  //std::cout << "Collection done " << extractedCones.rows() << " " << extractedCones.cols() << std::endl;
//std::cout << "extractedCones: " << extractedCones.transpose() << std::endl;
  if(extractedCones.cols() > 0){
    //std::cout << "Extracted Cones " << std::endl;
    //std::cout << extractedCones << std::endl;

    DetectConeLane::sortIntoSideArrays(extractedCones, nLeft, nRight, nSmall, nBig);

  }

}

void DetectConeLane::generateSurfaces(ArrayXXf sideLeft, ArrayXXf sideRight, ArrayXXf location){
  float distanceThreshold = 3.5; // TODO: Set in configuration
  float guessDistance = 3.0;

  ArrayXXf orderedConesLeft = DetectConeLane::orderAndFilterCones(sideLeft,location);
  ArrayXXf orderedConesRight = DetectConeLane::orderAndFilterCones(sideRight,location);

  float pathLengthLeft = DetectConeLane::findTotalPathLength(orderedConesLeft);
  float pathLengthRight = DetectConeLane::findTotalPathLength(orderedConesRight);

  ArrayXXf longSide;
  ArrayXXf shortSide;
  bool leftIsLong;

  if (std::abs(pathLengthLeft-pathLengthRight) > 0.01f) {
    leftIsLong = pathLengthLeft > pathLengthRight;
  } else{
    leftIsLong = orderedConesLeft.rows() > orderedConesRight.rows();
  }

  if(leftIsLong)
  {
    ArrayXXf tmpLongSide = orderedConesLeft;
    ArrayXXf tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesLeft, orderedConesRight, location, distanceThreshold,  guessDistance, false);

    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
    longSide = tmpLongSide;

    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
    shortSide = tmpShortSide;
  }
  else
  {
    ArrayXXf tmpLongSide = orderedConesRight;
    ArrayXXf tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesRight, orderedConesLeft, location, distanceThreshold,  guessDistance, true);

    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
    longSide = tmpLongSide;

    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
    shortSide = tmpShortSide;
  } // End of else
  /*
  longSide.resize(2,2);
  shortSide.resize(2,2);
  longSide << 3.17316, 1.2082,
  6.611233, -0.812926;
  shortSide << 2.14202, -1.47425,
  5.09234, -3.39936;
    */
  // std::cout<<"longSide accepted cones: "<<longSide<<"\n";
  // std::cout<<"shortSide accepted cones: "<<shortSide<<"\n";


  if(longSide.rows() > 1)
  {
    // findSafeLocalPath ends with sending surfaces
    DetectConeLane::findSafeLocalPath(longSide, shortSide);
  }
  else
  {
    // TODO: In here a special case surface should be sent
    if(longSide.rows() == 0)
    { std::cout<<"No Cones"<<"\n";
      //No cones
      opendlv::logic::perception::GroundSurface surface;
      surface.setSurfaceId(1);
      odcore::data::Container cStop1(surface);
      getConference().send(cStop1);

      opendlv::logic::perception::GroundSurfaceArea surfaceArea;
      surfaceArea.setSurfaceId(0);
      surfaceArea.setX1(1.0f);
      surfaceArea.setY1(0.0f);
      surfaceArea.setX2(1.0f);
      surfaceArea.setY2(0.0f);
      surfaceArea.setX3(0.0f);
      surfaceArea.setY3(0.0f);
      surfaceArea.setX4(0.0f);
      surfaceArea.setY4(0.0f);
      odcore::data::Container cStop2(surfaceArea);
      getConference().send(cStop2);
    }
    else if(longSide.rows() == 1 && shortSide.rows() == 0)
    { //std::cout<<"1 Cone"<<"\n";
      // 1 cone
      int direction;
      if(leftIsLong)
      {
        direction = -1;
      }
      else
      {
        direction = 1;
      }

      opendlv::logic::perception::GroundSurface surface;
      surface.setSurfaceId(1);
      odcore::data::Container cGo1(surface);
      getConference().send(cGo1);

      opendlv::logic::perception::GroundSurfaceArea surfaceArea;
      surfaceArea.setSurfaceId(0);
      surfaceArea.setX1(0.0f);
      surfaceArea.setY1(0.0f);
      surfaceArea.setX2(0.0f);
      surfaceArea.setY2(0.0f);
      surfaceArea.setX3(longSide(0,0));
      surfaceArea.setY3(longSide(0,1)+1.5f*direction);
      surfaceArea.setX4(longSide(0,0));
      surfaceArea.setY4(longSide(0,1)+1.5f*direction);
      odcore::data::Container cGo2(surfaceArea);
      getConference().send(cGo2);

    }
    else
    { //std::cout<<"1 on each side"<<"\n";
      //1 on each side
      opendlv::logic::perception::GroundSurface surface;
      surface.setSurfaceId(1);
      odcore::data::Container cGo3(surface);
      getConference().send(cGo3);

      opendlv::logic::perception::GroundSurfaceArea surfaceArea;
      surfaceArea.setSurfaceId(0);
      surfaceArea.setX1(0.0f);
      surfaceArea.setY1(0.0f);
      surfaceArea.setX2(0.0f);
      surfaceArea.setY2(0.0f);
      surfaceArea.setX3(longSide(0,0));
      surfaceArea.setY3(longSide(0,1));
      surfaceArea.setX4(shortSide(0,0));
      surfaceArea.setY4(shortSide(0,1));
      odcore::data::Container cGo4(surfaceArea);
      getConference().send(cGo4);
    }

  } // End of else
} // End of generateSurfaces


// copy from perception-detectcone
Eigen::MatrixXd DetectConeLane::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = MatrixXd::Zero(2,1);
  recievedPoint << xData,
                   yData;
  return recievedPoint;
}


void DetectConeLane::findSafeLocalPath(ArrayXXf sidePointsLeft, ArrayXXf sidePointsRight)
{

  ArrayXXf longSide, shortSide;

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

  int nMidPoints = longSide.rows()*3; // TODO: We might have to choose this more carefully
  int nConesShort = shortSide.rows();

  // Divide the longest side into segments of equal length
  ArrayXXf virtualPointsLong = placeEquidistantPoints(longSide,true,nMidPoints,-1);
  ArrayXXf virtualPointsShort(nMidPoints,2);

  float shortestDist, tmpDist, factor;
  int closestConeIndex;

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
  } // End of for

  ArrayXXf virtualPointsLongFinal, virtualPointsShortFinal;
  if(virtualPointsLong.rows() % 2 == 0)
  {
    // Number of points is even. Accepted.
    virtualPointsLongFinal = virtualPointsLong;
    virtualPointsShortFinal = virtualPointsShort;
  }
  else
  {
    // Number of points is odd. Add another point with tiny extrapolation in the end.
    int nLong = virtualPointsLong.rows();
    int nShort = virtualPointsShort.rows();

    virtualPointsLongFinal.resize(nLong+1,2);
    virtualPointsShortFinal.resize(nShort+1,2);

    virtualPointsLongFinal.topRows(nLong) = virtualPointsLong;
    virtualPointsShortFinal.topRows(nShort) = virtualPointsShort;

    ArrayXXf lastVecLong = virtualPointsLong.row(nLong-1)-virtualPointsLong.row(nLong-2);
    lastVecLong = lastVecLong / ((lastVecLong.matrix()).norm());

    virtualPointsLongFinal.bottomRows(1) = virtualPointsLong.row(nLong-1) + 0.01*lastVecLong;
    virtualPointsShortFinal.bottomRows(1) = virtualPointsShort.row(nShort-1);
  } // End of else

  DetectConeLane::sendMatchedContainer(virtualPointsLongFinal, virtualPointsShortFinal);


/* // All of this should be taken care of in Linus' module

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

*/

} // End of findSafeLocalPath


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

  return newSidePoints;
} // End of placeEquidistantPoints


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
  } // End of for
  return orderedCones;
} // End of orderCones


ArrayXXf DetectConeLane::orderAndFilterCones(ArrayXXf cones, ArrayXXf vehicleLocation)
{
  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The cones that satisfy some requirements, in order

  int nCones = cones.rows();
  ArrayXXf current = vehicleLocation;
  ArrayXXf found(nCones,1);
  found.fill(-1);

  float shortestDist, tmpDist, line1, line2, line3, angle;
  int closestConeIndex;
  int nAcceptedCones = 0;

  float orderingDistanceThreshold = 5.5; // TODO: There might be an alternative to hard coding this...
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
// std::cout << "Remove invalid cones" << std::endl;
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

  return orderedCones;
} // End of orderAndFilterCones


ArrayXXf DetectConeLane::insertNeededGuessedCones(ArrayXXf longSide, ArrayXXf shortSide, ArrayXXf vehicleLocation, float distanceThreshold, float guessDistance, bool guessToTheLeft)
{
  // Input: Both cone sides, vehicle position, two distance values and if the guesses should be on the left side
  // Output: The new ordered short side with mixed real and guessed cones
  int nConesLong = longSide.rows();
  int nConesShort = shortSide.rows();

  ArrayXXf guessedCones(std::max(2*nConesLong-2,0),2); // 2n-2 is the number of guesses if all known cones need guessed matches
  float shortestDist, tmpDist;
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
    if(shortestDist > distanceThreshold && longSide.rows()>1)
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
  return newShortSide;
} // End of insertNeededGuessedCones


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
    std::cout << "WARNING, ENTERED DANGEROUS AREA IN GUESSCONES " << std::endl;
    guessedCones << -1000,-1000;
  } // End of else

  return guessedCones;
} // End of guessCones


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

  return pathLength;
} // End of findTotalPathLength


float DetectConeLane::findFactorToClosestPoint(ArrayXXf p1, ArrayXXf p2, ArrayXXf q)
{
  // Input: The two cones of a cone segment and a reference point
  // Output: The factor to multiply with the vector between the cones in order to reach the point on the segment that has a
  // perpendicular line to the reference point

  ArrayXXf v = p2-p1; // The line to follow
  ArrayXXf n(1,2);    // The normal
  n << -v(1),v(0);

  float factor = (q(0)-p1(0)+(p1(1)-q(1))*n(0)/n(1))/(v(0)-v(1)*n(0)/n(1));

  return factor;
} // End of findFactorToClosestPoint


void DetectConeLane::sortIntoSideArrays(MatrixXd extractedCones, int nLeft, int nRight, int nSmall, int nBig)
{

  int coneNum = extractedCones.cols();
  //Convert to cartesian
  Eigen::MatrixXd cone;
  Eigen::MatrixXd coneLocal = Eigen::MatrixXd::Zero(2,coneNum);
  for(int p = 0; p < coneNum; p++)
  {
    cone = Spherical2Cartesian(extractedCones(0,p), extractedCones(1,p), extractedCones(2,p));
    coneLocal.col(p) = cone;
  }
//std::cout << "ConeLocal: " << coneLocal.transpose() << std::endl;

  //if(nLeft > 1 || nRight > 1 )
  //{

    Eigen::MatrixXd coneLeft = Eigen::MatrixXd::Zero(2,nLeft);
    Eigen::MatrixXd coneRight = Eigen::MatrixXd::Zero(2,nRight);
    Eigen::MatrixXd coneSmall = Eigen::MatrixXd::Zero(2,nSmall);
    Eigen::MatrixXd coneBig = Eigen::MatrixXd::Zero(2,nBig);
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;
    int type;

    for(int k = 0; k < coneNum; k++){
      type = static_cast<int>(extractedCones(3,k));
      if(type == 1)
      {
        coneLeft.col(a) = coneLocal.col(k);
        a++;
      }
      else if(type == 2)
      {
        coneRight.col(b) = coneLocal.col(k);
        b++;
      }
      else if(type == 3)
      {
        coneSmall.col(c) = coneLocal.col(k);
        c++;
      }
      else if(type == 4)
      {
        coneBig.col(d) = coneLocal.col(k);
        d++;
      }
    } // End of for


    ArrayXXf location(1,2);
    location << 0,0;

    MatrixXf coneLeft_f = coneLeft.cast <float> ();
    MatrixXf coneRight_f = coneRight.cast <float> ();
    ArrayXXf sideLeft = coneLeft_f.transpose().array();
    ArrayXXf sideRight = coneRight_f.transpose().array();

// -- TODO: Add sending messages for orange cones --
    generateSurfaces(sideLeft, sideRight, location);

  //} // End of if
} // End of sortIntoSideArrays


void DetectConeLane::sendMatchedContainer(Eigen::ArrayXXf virtualPointsLong, Eigen::ArrayXXf virtualPointsShort)
{

  int nSurfaces = virtualPointsLong.rows()/2;
  //std::cout << "Sending " << nSurfaces << " surfaces" << std::endl;

  opendlv::logic::perception::GroundSurface surface;
  surface.setSurfaceId(nSurfaces);
  odcore::data::Container c0(surface);
  getConference().send(c0);
  //std::cout << "virtualPointsLong: " << virtualPointsLong<< std::endl;
  //std::cout << "virtualPointsShort: " << virtualPointsShort<< std::endl;
  for(int n = 0; n < nSurfaces; n++)
  {
    opendlv::logic::perception::GroundSurfaceArea surfaceArea;
    surfaceArea.setSurfaceId(n);
    surfaceArea.setX1(virtualPointsLong(2*n,0));
    surfaceArea.setY1(virtualPointsLong(2*n,1));
    surfaceArea.setX2(virtualPointsShort(2*n,0));
    surfaceArea.setY2(virtualPointsShort(2*n,1));
    surfaceArea.setX3(virtualPointsLong(2*n+1,0));
    surfaceArea.setY3(virtualPointsLong(2*n+1,1));
    surfaceArea.setX4(virtualPointsShort(2*n+1,0));
    surfaceArea.setY4(virtualPointsShort(2*n+1,1));
    odcore::data::Container c1(surfaceArea);
    getConference().send(c1);

  } // End of for

} // End of sendMatchedContainer





}
}
}
}
