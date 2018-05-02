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
#include <fstream>
#include <sstream>
#include <thread>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include "detectcone.hpp"

namespace opendlv {
namespace sim {
namespace cfsd18 {
namespace perception {

DetectCone::DetectCone(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "sim-cfsd18-perception-detectcone")
, m_heading()
, m_location()
, m_leftCones()
, m_rightCones()
, m_smallCones()
, m_bigCones()
, m_orangeVisibleInSlam()
, m_locationMutex()
{
}

DetectCone::~DetectCone()
{
}



void DetectCone::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::sim::Frame::ID()) 
  {
    auto frame = a_container.getData<opendlv::sim::Frame>();
    float x = frame.getX();
    float y = frame.getY();
    float yaw = frame.getYaw();
    {
      odcore::base::Lock lockLocation(m_locationMutex);
      m_location << x,y;
      m_heading = yaw;
    }
  }
}


odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DetectCone::body()
{
  auto kv = getKeyValueConfiguration();
  float const detectRange = kv.getValue<float>("sim-cfsd18-perception-detectcone.detectRange");
  float const detectWidth = kv.getValue<float>("sim-cfsd18-perception-detectcone.detectWidth");
  bool const fakeSlamActivated = kv.getValue<bool>("sim-cfsd18-perception-detectcone.fakeSlamActivated");
  int const nConesFakeSlam = kv.getValue<int>("sim-cfsd18-perception-detectcone.nConesFakeSlam");

  while(getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING)
  {
    ArrayXXf locationCopy;
    float headingCopy;
    {
      odcore::base::Lock lockLocation(m_locationMutex);
      locationCopy = m_location;
      headingCopy = m_heading;
    }
    ArrayXXf detectedConesLeft, detectedConesRight, detectedConesSmall, detectedConesBig;
    
    if(fakeSlamActivated)
    {
      detectedConesLeft = DetectCone::simConeDetectorSlam(m_leftCones, locationCopy, headingCopy, nConesFakeSlam);
      detectedConesRight = DetectCone::simConeDetectorSlam(m_rightCones, locationCopy, headingCopy, nConesFakeSlam);

      if(m_orangeVisibleInSlam)
      {
        // If the orange cones are set to visible in the detection they will be transformed into local coordinates and stored
        m_orangeVisibleInSlam = false;
        MatrixXf rotationMatrix(2,2);
        rotationMatrix << std::cos(-headingCopy),-std::sin(-headingCopy),
                          std::sin(-headingCopy),std::cos(-headingCopy);

        ArrayXXf tmpLocationSmall(m_smallCones.rows(),2);
        (tmpLocationSmall.col(0)).fill(locationCopy(0));
        (tmpLocationSmall.col(1)).fill(locationCopy(1));
        ArrayXXf tmpLocationBig(m_bigCones.rows(),2);
        (tmpLocationBig.col(0)).fill(locationCopy(0));
        (tmpLocationBig.col(1)).fill(locationCopy(1));

        detectedConesSmall = ((rotationMatrix*(((m_smallCones-tmpLocationSmall).matrix()).transpose())).transpose()).array();
        detectedConesBig = ((rotationMatrix*(((m_bigCones-tmpLocationBig).matrix()).transpose())).transpose()).array();
      }
      else
      {
        // Otherwise no orange cones are stored
        detectedConesSmall.resize(0,2);
        detectedConesBig.resize(0,2);
      } // End of else
    }
    else
    {
      // If slam detection is deactivated the cones will be detected with normal vision
      detectedConesLeft = DetectCone::simConeDetectorBox(m_leftCones, locationCopy, headingCopy, detectRange, detectWidth);
      detectedConesRight = DetectCone::simConeDetectorBox(m_rightCones, locationCopy, headingCopy, detectRange, detectWidth);
      detectedConesSmall = DetectCone::simConeDetectorBox(m_smallCones, locationCopy, headingCopy, detectRange, detectWidth);
      detectedConesBig = DetectCone::simConeDetectorBox(m_bigCones, locationCopy, headingCopy, detectRange, detectWidth);
    } // End of else

    // This is where the messages are sent
    MatrixXd detectedConesLeftMat = ((detectedConesLeft.matrix()).transpose()).cast <double> ();
    MatrixXd detectedConesRightMat = ((detectedConesRight.matrix()).transpose()).cast <double> ();
    MatrixXd detectedConesSmallMat = ((detectedConesSmall.matrix()).transpose()).cast <double> ();
    MatrixXd detectedConesBigMat = ((detectedConesBig.matrix()).transpose()).cast <double> ();

    opendlv::logic::perception::Object numberOfCones;
    numberOfCones.setObjectId(detectedConesLeftMat.cols()+detectedConesRightMat.cols()+detectedConesSmallMat.cols()+detectedConesBigMat.cols());

    odcore::data::Container c0(numberOfCones);
    c0.setSenderStamp(m_senderStamp);
    getConference().send(c0);

    int type = 1;
    auto startLeft = std::chrono::system_clock::now();
    sendMatchedContainer(detectedConesLeftMat, type, 0);
    type = 2;
    sendMatchedContainer(detectedConesRightMat, type, detectedConesLeftMat.cols());
    type = 3;
    sendMatchedContainer(detectedConesSmallMat, type, detectedConesLeftMat.cols()+detectedConesRightMat.cols());
    type = 4;
    sendMatchedContainer(detectedConesBigMat, type, detectedConesLeftMat.cols()+detectedConesRightMat.cols()+detectedConesSmallMat.cols());

    auto finishRight = std::chrono::system_clock::now();
    auto timeSend = std::chrono::duration_cast<std::chrono::microseconds>(finishRight - startLeft);
    std::cout << "sendTime:" << timeSend.count() << std::endl;
  } // End of while

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
} // End of body


void DetectCone::setUp()
{
  // Starting position and heading are set in the configuration
  auto kv = getKeyValueConfiguration();

  float const startX = kv.getValue<float>("sim-cfsd18-perception-detectcone.startX");
  float const startY = kv.getValue<float>("sim-cfsd18-perception-detectcone.startY");
  float const startHeading = kv.getValue<float>("sim-cfsd18-perception-detectcone.startHeading");
  m_location.resize(1,2);
  m_location << startX,startY;
  m_heading = startHeading;

  std::string const filename = kv.getValue<std::string>("sim-cfsd18-perception-detectcone.mapFilename");
  DetectCone::readMap(filename);
} // End of setUp


void DetectCone::tearDown()
{
}

void DetectCone::readMap(std::string filename)
{
  int leftCounter = 0;
  int rightCounter = 0;
  int smallCounter = 0;
  int bigCounter = 0;

  std::string line, word;
  std::string const HOME = "/opt/opendlv.data/";
  std::string infile = HOME + filename;

  std::ifstream file(infile, std::ifstream::in);

  if(file.is_open())
  {
    while(getline(file,line))
    {
      std::stringstream strstr(line);

      getline(strstr,word,',');
      getline(strstr,word,',');
      getline(strstr,word,',');

      if(word.compare("1") == 0){leftCounter = leftCounter+1;}
      else if(word.compare("2") == 0){rightCounter = rightCounter+1;}
      else if(word.compare("3") == 0){smallCounter = smallCounter+1;}
      else if(word.compare("4") == 0){bigCounter = bigCounter+1;}
      else{std::cout << "ERROR in DetectCone::simDetectCone while counting types. Not a valid cone type." << std::endl;}
    } // End of while

    file.close();
  } // End of if

  ArrayXXf tmpLeftCones(leftCounter,2);
  ArrayXXf tmpRightCones(rightCounter,2);
  ArrayXXf tmpSmallCones(smallCounter,2);
  ArrayXXf tmpBigCones(bigCounter,2);
  float x, y;
  leftCounter = 0;
  rightCounter = 0;
  smallCounter = 0;
  bigCounter = 0;
  std::ifstream myFile(infile, std::ifstream::in);

  if(myFile.is_open())
  {
    while(getline(myFile,line))
    {
      std::stringstream strstr(line);

      getline(strstr,word,',');
      x = std::stof(word);
      getline(strstr,word,',');
      y = std::stof(word);

      getline(strstr,word,',');

      if(word.compare("1") == 0)
      {
        tmpLeftCones(leftCounter,0) = x;
        tmpLeftCones(leftCounter,1) = y;
        leftCounter = leftCounter+1;
      }
      else if(word.compare("2") == 0)
      {
        tmpRightCones(rightCounter,0) = x;
        tmpRightCones(rightCounter,1) = y;
        rightCounter = rightCounter+1;
      }
      else if(word.compare("3") == 0)
      {
        tmpSmallCones(smallCounter,0) = x;
        tmpSmallCones(smallCounter,1) = y;
        smallCounter = smallCounter+1;
      }
      else if(word.compare("4") == 0)
      {
        tmpBigCones(bigCounter,0) = x;
        tmpBigCones(bigCounter,1) = y;
        bigCounter = bigCounter+1;
      }
      else{std::cout << "ERROR in DetectCone::simDetectCone while storing cones. Not a valid cone type." << std::endl;}
    } // End of while

    myFile.close();
  } // End of if

  m_leftCones = tmpLeftCones;
  m_rightCones = tmpRightCones;
  m_smallCones = tmpSmallCones;
  m_bigCones = tmpBigCones;
} // End of readMap


ArrayXXf DetectCone::simConeDetectorBox(ArrayXXf globalMap, ArrayXXf location, float heading, float detectRange, float detectWidth)
{
  // Input: Positions of cones and vehicle, heading angle, detection ranges forward and to the side
  // Output: Local coordinates of the cones within the specified area

  int nCones = globalMap.rows();

  MatrixXf rotationMatrix(2,2);
  rotationMatrix << std::cos(-heading),-std::sin(-heading),
                    std::sin(-heading),std::cos(-heading);

  ArrayXXf tmpLocation(nCones,2);
  (tmpLocation.col(0)).fill(location(0));
  (tmpLocation.col(1)).fill(location(1));

  ArrayXXf localMap = ((rotationMatrix*(((globalMap-tmpLocation).matrix()).transpose())).transpose()).array();

  ArrayXXf detectedCones(nCones,2);
  bool inLongitudinalInterval, inLateralInterval;
  int nFound = 0;
  for(int i = 0; i < nCones; i = i+1)
  {
    inLongitudinalInterval = localMap(i,0) < detectRange && localMap(i,0) >= 0;
    inLateralInterval = localMap(i,1) >= -detectWidth/2 && localMap(i,1) <= detectWidth/2;

    if(inLongitudinalInterval && inLateralInterval)
    {
      detectedCones.row(nFound) = localMap.row(i);
      nFound = nFound+1;
    } // End of if
  } // End of for

ArrayXXf detectedConesFinal = detectedCones.topRows(nFound);

return detectedConesFinal;
} // End of simConeDetectorBox


ArrayXXf DetectCone::simConeDetectorSlam(ArrayXXf globalMap, ArrayXXf location, float heading, int nConesInFakeSlam)
{
  // Input: Positions of cones and vehicle, heading angle, detection ranges forward and to the side
  // Output: Local coordinates of the upcoming cones

  int nCones = globalMap.rows();

  MatrixXf rotationMatrix(2,2);
  rotationMatrix << std::cos(-heading),-std::sin(-heading),
                    std::sin(-heading),std::cos(-heading);

  ArrayXXf tmpLocation(nCones,2);
  (tmpLocation.col(0)).fill(location(0));
  (tmpLocation.col(1)).fill(location(1));

  // Convert to local coordinates
  ArrayXXf localMap = ((rotationMatrix*(((globalMap-tmpLocation).matrix()).transpose())).transpose()).array();

  float shortestDist = std::numeric_limits<float>::infinity();
  float tmpDist;
  int closestConeIndex = -1;

  // Find the closest cone. It will be the first in the returned sequence.
  for(int i = 0; i < nCones; i = i+1)
  {
    tmpDist = ((localMap.row(i)).matrix()).norm();
    if(tmpDist < shortestDist && tmpDist > 0)
    {
      shortestDist = tmpDist;
      closestConeIndex = i;
    } // End of if
  } // End of for

  if(closestConeIndex != -1)
  {
    VectorXi indices;

    // If more than the existing cones are requested, send all existing cones
    if(nConesInFakeSlam >= nCones)
    {
      // If the first cone is closest, no wrap-around is needed
      if(closestConeIndex == 0)
      {
        indices = VectorXi::LinSpaced(nCones,0,nCones-1);
      }
      else
      {
        VectorXi firstPart = VectorXi::LinSpaced(nCones-closestConeIndex,closestConeIndex,nCones-1);
        VectorXi secondPart = VectorXi::LinSpaced(closestConeIndex,0,closestConeIndex-1);
        indices << firstPart, secondPart;
      } // End of else
    }
    // If the sequence should contain both the end and beginning of the track, do wrap-around
    else if(closestConeIndex + nConesInFakeSlam > nCones)
    {
      VectorXi firstPart = VectorXi::LinSpaced(nCones-closestConeIndex,closestConeIndex,nCones-1);
      VectorXi secondPart = VectorXi::LinSpaced(nConesInFakeSlam-(nCones-closestConeIndex),0,nConesInFakeSlam-(nCones-closestConeIndex)-1);
      indices << firstPart, secondPart;
    }
    // Otherwise simply take the closest and the following cones
    else
    {
      indices = VectorXi::LinSpaced(nConesInFakeSlam,closestConeIndex,closestConeIndex+nConesInFakeSlam-1);
    }

    // Sort the cones according to the order set above
    ArrayXXf detectedCones(indices.size(),2);
    for(int i = 0; i < indices.size(); i = i+1)
    {
      detectedCones.row(i) = localMap.row(indices(i));
    }

    // If the first cones of the track is visible, the orange cones are set as visible as well
    if(indices.minCoeff() == 0)
    {
      m_orangeVisibleInSlam = true;
    }

    return detectedCones;

  }
  // If no closest cone was found, the returned array is empty
  else
  {
    std::cout << "Error: No cone found in fake slam detection" << std::endl;
    ArrayXXf detectedCones(0,2);
  
    return detectedCones;
  } // End of else
} // End of simConeDetectorSlam


void DetectCone::sendMatchedContainer(Eigen::MatrixXd cones, int type, int startID)
{
std::cout << "New location: " << m_location << " and heading: " << m_heading << std::endl;
std::cout << "Sending " << cones.cols() << " of type " << type << std::endl;

  opendlv::logic::sensation::Point conePoint;
  for(int n = 0; n < cones.cols(); n++){

    Cartesian2Spherical(cones(0,n), cones(1,n), 0, conePoint);

    opendlv::logic::perception::ObjectDirection coneDirection;
    coneDirection.setObjectId(n+startID);
    coneDirection.setAzimuthAngle(conePoint.getAzimuthAngle());
    coneDirection.setZenithAngle(conePoint.getZenithAngle());
    odcore::data::Container c2(coneDirection);
    c2.setSenderStamp(m_senderStamp);
    getConference().send(c2);

    opendlv::logic::perception::ObjectDistance coneDistance;
    coneDistance.setObjectId(n+startID);
    coneDistance.setDistance(conePoint.getDistance());
    odcore::data::Container c3(coneDistance);
    c3.setSenderStamp(m_senderStamp);
    getConference().send(c3);

    opendlv::logic::perception::ObjectType coneType;
    coneType.setObjectId(n+startID);
    coneType.setType(type);
    odcore::data::Container c4(coneType);
    c3.setSenderStamp(m_senderStamp);
    getConference().send(c4);
  } // End of for
} // End of sendMatchedContainer


void DetectCone::Cartesian2Spherical(double x, double y, double z, opendlv::logic::sensation::Point &pointInSpherical)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan2(x,y)*static_cast<double>(RAD2DEG);
  double zenithAngle = atan2(z,sqrt(x*x+y*y))*static_cast<double>(RAD2DEG);
  pointInSpherical.setDistance(distance);
  pointInSpherical.setAzimuthAngle(azimuthAngle);
  pointInSpherical.setZenithAngle(zenithAngle);
} // End of Cartesian2Spherical

}
}
}
}
