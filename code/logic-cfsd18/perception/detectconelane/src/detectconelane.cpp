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
#include <cstdlib>
#include <mutex>
#include <condition_variable>


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
, m_newFrame{true}
, m_directionOK{false}
, m_distanceOK {false}
, m_runOK{true}
, m_directionFrame{}
, m_distanceFrame{}
, m_typeFrame{}
, m_directionFrameBuffer{}
, m_distanceFrameBuffer{}
, m_typeFrameBuffer{}
, m_lastDirectionId{}
, m_lastDistanceId{}
, m_lastTypeId{}
, m_newDirectionId{true}
, m_newDistanceId{true}
, m_newTypeId{true}
, m_directionTimeReceived{}
, m_distanceTimeReceived{}
, m_typeTimeReceived{}
, m_nConesInFrame{}
, m_objectPropertyId{}
, m_directionId{}
, m_distanceId{}
, m_typeId{}
, m_surfaceId{}
{
  m_surfaceId = rand();
}

DetectConeLane::~DetectConeLane()
{
}

void DetectConeLane::setUp()
{
}

void DetectConeLane::tearDown()
{
}


void DetectConeLane::nextContainer(odcore::data::Container &a_container)
{
if(a_container.getDataType() == opendlv::logic::perception::ObjectProperty::ID()){
    //std::cout << "RECIEVED AN OBJECTPROPERY!" << std::endl;
    auto object = a_container.getData<opendlv::logic::perception::ObjectProperty>();
    int objectId = object.getObjectId();
    auto nConesInFrame = object.getProperty();

    if (m_newFrame) { // If true, a frame has just been sent for processing
      m_newFrame = false;
      m_nConesInFrame = std::stoul(nConesInFrame);
      m_objectPropertyId = objectId; // Currently not used.
      //std::cout<<m_nConesInFrame<<" frames to run"<<"\n";
    }
}

if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID()) {
    int objectId;
    {
      odcore::base::Lock lockDirection(m_directionMutex);
      auto object = a_container.getData<opendlv::logic::perception::ObjectDirection>();
      objectId = object.getObjectId();
      odcore::data::TimeStamp containerStamp = a_container.getReceivedTimeStamp();
      double timeStamp = containerStamp.toMicroseconds(); // Save timeStamp for sorting purposes;

        if (m_newDirectionId) {
          m_directionId = (objectId!=m_lastDirectionId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
          m_newDirectionId=(m_directionId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
        }

        float angle = object.getAzimuthAngle(); //Unpack message

        if (objectId == m_directionId) {
          m_directionFrame[timeStamp] = angle;
          m_directionTimeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
        } else if (objectId != m_lastDirectionId){ // If message doesn't belong to current or previous frame.
          m_directionFrameBuffer[timeStamp] = angle; // Place message content coordinates in buffer
        }
    }
    auto wait = std::chrono::system_clock::now(); // Time point now
    std::chrono::duration<double> dur = wait-m_directionTimeReceived; // Duration since last message recieved to m_surfaceFrame
    double duration = (m_directionId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame
    double receiveTimeLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-perception-detectconelane.receiveTimeLimit");
    if ((duration>receiveTimeLimit) && (m_nConesInFrame>m_directionFrame.size())) { //Only for debug
      std::cout<<"DURATION TIME DIRECTION EXCEEDED: "<<duration<<std::endl;
      std::cout<<m_directionFrame.size()<<" directionFrames to run"<<"\n";
      std::cout<<m_nConesInFrame<<" frames to run"<<"\n";
    }
    // Run if frame is full or if we have waited to long for the remaining messages
    if ((m_directionFrame.size()==m_nConesInFrame || duration>receiveTimeLimit)) { //!m_newFrame && objectId==m_surfaceId &&
      m_directionOK=true;
      //std::cout<<m_directionFrame.size()<<" directionFrames to run"<<"\n";
      //std::cout<<m_directionFrameBuffer.size()<<" directionFrames in buffer"<<"\n";
      /*std::cout<<"m_directionOK"<<"\n";*/
    }
}

else if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()){
  int objectId;
  {
    odcore::base::Lock lockDistance(m_distanceMutex);
    auto object = a_container.getData<opendlv::logic::perception::ObjectDistance>();
    objectId = object.getObjectId();
    odcore::data::TimeStamp containerStamp = a_container.getReceivedTimeStamp();
    double timeStamp = containerStamp.toMicroseconds(); // Save timeStamp for sorting purposes;

    if (m_newDistanceId) {
      m_distanceId = (objectId!=m_lastDistanceId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
      m_newDistanceId=(m_distanceId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
    }

    float distance = object.getDistance(); //Unpack message

    if (objectId == m_distanceId) {
      m_distanceFrame[timeStamp] = distance;
      m_distanceTimeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
    } else if (objectId != m_lastDistanceId){ // If message doesn't belong to current or previous frame.
      m_distanceFrameBuffer[timeStamp] = distance; // Place message content coordinates in buffer
    }
  }
  auto wait = std::chrono::system_clock::now(); // Time point now
  std::chrono::duration<double> dur = wait-m_distanceTimeReceived; // Duration since last message recieved to m_surfaceFrame
  double duration = (m_distanceId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame
  double receiveTimeLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-perception-detectconelane.receiveTimeLimit");
  if ((duration>receiveTimeLimit)&& (m_nConesInFrame>m_distanceFrame.size())) { //Only for debug
    std::cout<<"DURATION TIME DISTANCE EXCEEDED: "<<duration<<std::endl;
    std::cout<<m_distanceFrame.size()<<" distanceFrames to run"<<"\n";
    std::cout<<m_nConesInFrame<<" frames to run"<<"\n";
  }
  // Run if frame is full or if we have waited to long for the remaining messages
  if ((m_distanceFrame.size()==m_nConesInFrame || duration>receiveTimeLimit)) { //!m_newFrame && objectId==m_surfaceId &&
    m_distanceOK=true;
    //std::cout<<m_distanceFrame.size()<<" distanceFrames to run"<<"\n";
    //std::cout<<m_distanceFrameBuffer.size()<<" distanceFrames in buffer"<<"\n";
    /*std::cout<<"m_distanceOK"<<"\n";*/
  }
}

  else if(a_container.getDataType() == opendlv::logic::perception::ObjectType::ID()){
    int objectId;
    {
      odcore::base::Lock lockType(m_typeMutex);
      auto object = a_container.getData<opendlv::logic::perception::ObjectType>();
      objectId = object.getObjectId();
      odcore::data::TimeStamp containerStamp = a_container.getReceivedTimeStamp();
      double timeStamp = containerStamp.toMicroseconds(); // Save timeStamp for sorting purposes;

      if (m_newTypeId) {
        m_typeId = (objectId!=m_lastTypeId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
        m_newTypeId=(m_typeId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
      }

      int type = object.getType(); //Unpack message

      if (objectId == m_typeId) {
        m_typeFrame[timeStamp] = type;
        m_typeTimeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
      } else if (objectId != m_lastTypeId){ // If message doesn't belong to current or previous frame.
        m_typeFrameBuffer[timeStamp] = type; // Place message content in buffer
      }
    }
    auto wait = std::chrono::system_clock::now(); // Time point now
    std::chrono::duration<double> dur = wait-m_typeTimeReceived; // Duration since last message recieved to m_surfaceFrame
    double duration = (m_typeId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame
    double receiveTimeLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-perception-detectconelane.receiveTimeLimit");
    if ((duration>receiveTimeLimit) && (m_nConesInFrame>m_typeFrame.size())) { //Only for debug
      std::cout<<"DURATION TIME TYPE EXCEEDED: "<<duration<<std::endl;
      std::cout<<m_typeFrame.size()<<" typeFrames to run"<<"\n";
      std::cout<<m_nConesInFrame<<" frames to run"<<"\n";
    }
    // Run if frame is full or if we have waited to long for the remaining messages
    if ((m_typeFrame.size()==m_nConesInFrame || duration>receiveTimeLimit) && m_runOK) { //!m_newFrame && objectId==m_surfaceId &&
      if (m_directionOK && m_distanceOK) {
        m_runOK = false;
        //std::cout<<"m_runOK"<<std::endl;
        std::thread coneCollector(&DetectConeLane::initializeCollection, this);
        coneCollector.detach();
      }
      //std::cout<<m_typeFrame.size()<<" typeFrames to run"<<"\n";
      //std::cout<<m_typeFrameBuffer.size()<<" typeFrames in buffer"<<"\n";
    }
}
}
void DetectConeLane::initializeCollection(){
  /*std::cout<< "directionFrames in initializeCollection2: "<<m_directionFrame.size()<<"\n";
  std::cout<< "distanceFrames in initializeCollection2: "<<m_distanceFrame.size()<<"\n";
  std::cout<< "typeFrames in initializeCollection2: "<<m_typeFrame.size()<<"\n";*/
  std::map< double, float > directionFrame;
  std::map< double, float > distanceFrame;
  std::map< double, int > typeFrame;

  if (m_directionFrame.size() == m_distanceFrame.size() && m_directionFrame.size() == m_typeFrame.size()) {
    {
      odcore::base::Lock lockDirection(m_directionMutex);
      odcore::base::Lock lockDistance(m_distanceMutex);
      odcore::base::Lock lockType(m_typeMutex);
      m_newFrame = true;
      m_directionOK = false;
      m_distanceOK = false;
      directionFrame = m_directionFrame;
      distanceFrame = m_distanceFrame;
      typeFrame = m_typeFrame;
      m_directionFrame = m_directionFrameBuffer;
      m_distanceFrame = m_distanceFrameBuffer;
      m_typeFrame = m_typeFrameBuffer;
      m_directionFrameBuffer.clear(); // Clear buffer
      m_distanceFrameBuffer.clear(); // Clear buffer
      m_typeFrameBuffer.clear(); // Clear buffer
      m_lastDirectionId = m_directionId; // Update last object id to ignore late messages
      m_lastDistanceId = m_distanceId; // Update last object id to ignore late messages
      m_lastTypeId = m_typeId; // Update last object id to ignore late messages
      m_newDirectionId = true;
      m_newDistanceId = true;
      m_newTypeId = true;
    }
  }
  else {
    {
      odcore::base::Lock lockDirection(m_directionMutex);
      odcore::base::Lock lockDistance(m_distanceMutex);
      odcore::base::Lock lockType(m_typeMutex);
      m_newFrame = true;
      m_directionOK = false;
      m_distanceOK = false;
      m_directionFrame = m_directionFrameBuffer;
      m_distanceFrame = m_distanceFrameBuffer;
      m_typeFrame = m_typeFrameBuffer;
      m_directionFrameBuffer.clear(); // Clear buffer
      m_distanceFrameBuffer.clear(); // Clear buffer
      m_typeFrameBuffer.clear(); // Clear buffer
      m_lastDirectionId = m_directionId; // Update last object id to ignore late messages
      m_lastDistanceId = m_distanceId; // Update last object id to ignore late messages
      m_lastTypeId = m_typeId; // Update last object id to ignore late messages
      m_newDirectionId = true;
      m_newDistanceId = true;
      m_newTypeId = true;
      m_runOK = true;
    }
    std::cout<<"return 0"<<std::endl;
    return;
  }
  // Unpack
  Eigen::MatrixXd extractedCones(3,directionFrame.size());
  float dir;
  float dis;
  int tpe;
  int I=0;
  for (std::map<double, float >::iterator it = directionFrame.begin();it !=directionFrame.end();it++){
    dir=it->second;
    extractedCones(0,I) = static_cast<double>(dir);
    I++;
  }
  I=0;
  for (std::map<double, float >::iterator it = distanceFrame.begin();it !=distanceFrame.end();it++){
    dis=it->second;
    extractedCones(1,I) = static_cast<double>(dis);
    I++;
  }
  I=0;
  for (std::map<double, int >::iterator it = typeFrame.begin();it !=typeFrame.end();it++){
    tpe=it->second;
    extractedCones(2,I) = static_cast<double>(tpe);
    I++;
  }
  //std::cout<<"extractedCones"<<extractedCones<<std::endl;
  int nLeft = 0;
  int nRight = 0;
  int nSmall = 0;
  int nBig = 0;

    for (int i = 0; i < extractedCones.cols(); i++) {
      int type = extractedCones(2,i);
      if(type == 1){ nLeft++; }
      else if(type == 2){ nRight++; }
      else if(type == 3){ nSmall++; }
      else if(type == 4){ nBig++; }
      else
      {
        std::cout << "WARNING! Object " << i << " has invalid cone type: " << type << std::endl;
      } // End of else
    } // End of for

    //std::cout << "members: " << nLeft << " " << nRight << " " << nSmall << " " << nBig << std::endl;


  //Initialize for next collection
  //std::cout << "Collection done " << extractedCones.rows() << " " << extractedCones.cols() << std::endl;
//std::cout << "extractedCones: " << extractedCones.transpose() << std::endl;
  if(extractedCones.cols() > 0){
    //std::cout << "Extracted Cones " << std::endl;
    //std::cout << extractedCones << std::endl;

    DetectConeLane::sortIntoSideArrays(extractedCones, nLeft, nRight, nSmall, nBig);
  } // End of if
  m_runOK = true;
} // End of initializeCollection

void DetectConeLane::generateSurfaces(ArrayXXf sideLeft, ArrayXXf sideRight, ArrayXXf location){
  auto kv = getKeyValueConfiguration();
  float const coneWidthSeparationThreshold = kv.getValue<float>("logic-cfsd18-perception-detectconelane.coneWidthSeparationThreshold");
  float const guessDistance = kv.getValue<float>("logic-cfsd18-perception-detectconelane.guessDistance");
  bool const fakeSlamActivated = kv.getValue<bool>("logic-cfsd18-perception-detectconelane.fakeSlamActivated");
  ArrayXXf orderedConesLeft;
  ArrayXXf orderedConesRight;
  if (!fakeSlamActivated) {
    orderedConesLeft = DetectConeLane::orderAndFilterCones(sideLeft,location);
    orderedConesRight = DetectConeLane::orderAndFilterCones(sideRight,location);
  }else{
    orderedConesLeft = DetectConeLane::orderCones(sideLeft,location);
    orderedConesRight = DetectConeLane::orderCones(sideRight,location);
  }


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
    ArrayXXf tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesLeft, orderedConesRight, location, coneWidthSeparationThreshold,  guessDistance, false);

    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
    longSide = tmpLongSide;

    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
    shortSide = tmpShortSide;
  }
  else
  {
    ArrayXXf tmpLongSide = orderedConesRight;
    ArrayXXf tmpShortSide = DetectConeLane::insertNeededGuessedCones(orderedConesRight, orderedConesLeft, location, coneWidthSeparationThreshold,  guessDistance, true);

    longSide.resize(tmpLongSide.rows(),tmpLongSide.cols());
    longSide = tmpLongSide;

    shortSide.resize(tmpShortSide.rows(),tmpShortSide.cols());
    shortSide = tmpShortSide;
  } // End of else
  //std::cout<<"longSide accepted cones: "<<longSide<<"\n";
  //std::cout<<"shortSide accepted cones: "<<shortSide<<"\n";


  if(longSide.rows() > 1)
  {
    // findSafeLocalPath ends with sending surfaces
    DetectConeLane::findSafeLocalPath(longSide, shortSide);
  }
  else
  {
    if(longSide.rows() == 0)
    { std::cout<<"No Cones"<<"\n";
      //No cones
      opendlv::logic::perception::GroundSurfaceProperty surface;
      surface.setSurfaceId(m_surfaceId);
      surface.setProperty("1");
      odcore::data::Container cStop1(surface);
      getConference().send(cStop1);

      opendlv::logic::perception::GroundSurfaceArea surfaceArea;
      surfaceArea.setSurfaceId(m_surfaceId);
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
      //std::cout<<"Sending with ID: "<<m_surfaceId<<"\n";
      int rndmId = rand();
      while (m_surfaceId == rndmId){rndmId = rand();}
      m_surfaceId = rndmId;
    }
    else if(longSide.rows() == 1 && shortSide.rows() == 0)
    { std::cout<<"1 Cone"<<"\n";
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

      opendlv::logic::perception::GroundSurfaceProperty surface;
      surface.setSurfaceId(m_surfaceId);
      surface.setProperty("1");
      odcore::data::Container cGo1(surface);
      getConference().send(cGo1);

      opendlv::logic::perception::GroundSurfaceArea surfaceArea;
      surfaceArea.setSurfaceId(m_surfaceId);
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
      //std::cout<<"Sending with ID: "<<m_surfaceId<<"\n";
      int rndmId = rand();
      while (m_surfaceId == rndmId){rndmId = rand();}
      m_surfaceId = rndmId;
    }
    else
    { std::cout<<"1 on each side"<<"\n";
      //1 on each side
      opendlv::logic::perception::GroundSurfaceProperty surface;
      surface.setSurfaceId(m_surfaceId);
      surface.setProperty("1");
      odcore::data::Container cGo3(surface);
      getConference().send(cGo3);

      opendlv::logic::perception::GroundSurfaceArea surfaceArea;
      surfaceArea.setSurfaceId(m_surfaceId);
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
      //std::cout<<"Sending with ID: "<<m_surfaceId<<"\n";
      int rndmId = rand();
      while (m_surfaceId == rndmId){rndmId = rand();}
      m_surfaceId = rndmId;
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
} // End of Spherical2Cartesian


void DetectConeLane::findSafeLocalPath(ArrayXXf sidePointsLeft, ArrayXXf sidePointsRight)
{
  ArrayXXf longSide, shortSide;

  // Identify the longest side
  float pathLengthLeft = DetectConeLane::findTotalPathLength(sidePointsLeft);
  float pathLengthRight = DetectConeLane::findTotalPathLength(sidePointsRight);
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

  int nMidPoints = longSide.rows()*3;
  int nConesShort = shortSide.rows();

  // Divide the longest side into segments of equal length
  ArrayXXf virtualPointsLong = DetectConeLane::placeEquidistantPoints(longSide,true,nMidPoints,-1);
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
        factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(0),shortSide.row(1),virtualPointsLong.row(i));
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
      factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(nConesShort-2),shortSide.row(nConesShort-1),virtualPointsLong.row(i));
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
      factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(closestConeIndex-1),shortSide.row(closestConeIndex),virtualPointsLong.row(i));
      if(factor > 0 && factor <= 1)
      {
        virtualPointsShort.row(i) = shortSide.row(closestConeIndex-1)+factor*(shortSide.row(closestConeIndex)-shortSide.row(closestConeIndex-1));
      }
      else
      {
        factor = DetectConeLane::findFactorToClosestPoint(shortSide.row(closestConeIndex),shortSide.row(closestConeIndex+1),virtualPointsLong.row(i));
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

  auto kv = getKeyValueConfiguration();
  float const coneLengthSeparationThreshold = kv.getValue<float>("logic-cfsd18-perception-detectconelane.coneLengthSeparationThreshold");
  float const maxConeAngle = kv.getValue<float>("logic-cfsd18-perception-detectconelane.maxConeAngle");

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
        if(tmpDist < shortestDist && ((tmpDist < coneLengthSeparationThreshold && i>0) || (tmpDist < coneLengthSeparationThreshold+3.5f && i<1)) )
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

            if(std::abs(angle) > maxConeAngle)
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
std::cout << "Remove invalid cones" << std::endl;
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
        guess = DetectConeLane::guessCones(longSide.row(0),longSide.row(1),guessDistance,guessToTheLeft,true,false);
        nGuessedCones = nGuessedCones+1;
      }
      else if(i == nConesLong-1)
      {
        guess = DetectConeLane::guessCones(longSide.row(nConesLong-2),longSide.row(nConesLong-1),guessDistance,guessToTheLeft,false,true);
        nGuessedCones = nGuessedCones+1;
      }
      else
      {
        guess = DetectConeLane::guessCones(longSide.row(i-1),longSide.row(i),guessDistance,guessToTheLeft,false,true);
        nGuessedCones = nGuessedCones+1;
        guessedCones.row(nGuessedCones-1) = guess;
        guess = DetectConeLane::guessCones(longSide.row(i),longSide.row(i+1),guessDistance,guessToTheLeft,true,false);
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

  ArrayXXf newShortSide = DetectConeLane::orderCones(realAndGuessedCones, vehicleLocation);
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
    cone = DetectConeLane::Spherical2Cartesian(extractedCones(0,p), 0.0, extractedCones(1,p));
    coneLocal.col(p) = cone;
  } // End of for
//std::cout << "ConeLocal: " << coneLocal.transpose() << std::endl;

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
    type = static_cast<int>(extractedCones(2,k));
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
    } // End of else
  } // End of for


  ArrayXXf location(1,2);
  location << -3,0;

  MatrixXf coneLeft_f = coneLeft.cast <float> ();
  MatrixXf coneRight_f = coneRight.cast <float> ();
  ArrayXXf sideLeft = coneLeft_f.transpose().array();
  ArrayXXf sideRight = coneRight_f.transpose().array();

  DetectConeLane::generateSurfaces(sideLeft, sideRight, location);
} // End of sortIntoSideArrays


void DetectConeLane::sendMatchedContainer(Eigen::ArrayXXf virtualPointsLong, Eigen::ArrayXXf virtualPointsShort)
{
  int nSurfaces = virtualPointsLong.rows()/2;
  //std::cout << "Sending " << nSurfaces << " surfaces" << std::endl;

  opendlv::logic::perception::GroundSurfaceProperty surface;
  surface.setSurfaceId(m_surfaceId);
  std::string property = std::to_string(nSurfaces);
  surface.setProperty(property);
  odcore::data::Container c0(surface);
  getConference().send(c0);

  for(int n = 0; n < nSurfaces; n++)
  {
    opendlv::logic::perception::GroundSurfaceArea surfaceArea;
    surfaceArea.setSurfaceId(m_surfaceId);
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
  //std::cout<<"Sending with ID: "<<m_surfaceId<<"\n";
  int rndmId = rand();
  while (m_surfaceId == rndmId){rndmId = rand();}
  m_surfaceId = rndmId;
} // End of sendMatchedContainer





}
}
}
}
