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
{
}

DetectCone::~DetectCone()
{
}



void DetectCone::nextContainer(odcore::data::Container &a_container)
{
/*
//---------JUST A TEST--------------
ArrayXXf sideLeft(16,2);
sideLeft << -71.8448,43.4762,
            -70.1104,46.7798,
            -68.5906,50.1646,
            -66.9452,53.4154,
            -65.1495,55.6763,
            -63.4316,57.4711,
            -61.7986,58.8658,
            -60.1803,59.7912,
            -57.7773,59.8813,
            -56.4540,59.3048,
            -55.1177,57.3924,
            -54.7247,55.1797,
            -54.9912,53.1888,
            -55.7781,51.2868,
            -57.0103,49.0158,
            -58.4917,46.9843;

ArrayXXf sideRight(16,2);
sideRight << -69.3358,41.8116,
             -67.5248,45.2408,
             -65.9594,48.7006,
             -64.5145,51.6441,
             -63.0292,53.5468,
             -61.4630,55.5468,
             -60.1246,56.3718,
             -59.0203,57.0216,
             -58.8937,57.0944,
             -58.1000,56.7947,
             -57.9840,56.4957,
             -57.7252,55.3076,
             -57.8749,54.0262,
             -58.3856,52.7769,
             -59.4402,50.7850,
             -60.8640,48.8286;

ArrayXXf location(1,2);
location << -61,57;
float heading = 3.14159/6;
float detectRange = 11.5;
float detectWidth = 5;
*/



//std::string filename = "trackdrive_cones_dense.csv";
//DetectCone::readMap(filename);

  if (a_container.getDataType() == opendlv::sim::Frame::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();

    opendlv::logic::perception::Object o1;
    odcore::data::Container c1(o1);
    getConference().send(c1);
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DetectCone::body()
{
std::cout << "DetectCone body" << std::endl;
  auto kv = getKeyValueConfiguration();
  float const detectRange = kv.getValue<float>("sim-cfsd18-perception-detectcone.detectRange");
  float const detectWidth = kv.getValue<float>("sim-cfsd18-perception-detectcone.detectWidth");

  while(getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING)
  {
    ArrayXXf detectedConesLeft = DetectCone::simConeDetectorBox(m_leftCones, m_location, m_heading, detectRange, detectWidth);
    ArrayXXf detectedConesRight = DetectCone::simConeDetectorBox(m_rightCones, m_location, m_heading, detectRange, detectWidth);

std::cout << "Exited detectors" << std::endl;

    // This is where the messages are (hopefully) sent
    MatrixXd detectedConesLeftMat = ((detectedConesLeft.matrix()).transpose()).cast <double> ();
    MatrixXd detectedConesRightMat = ((detectedConesRight.matrix()).transpose()).cast <double> (); 

    opendlv::logic::perception::Object numberOfCones;
    numberOfCones.setObjectId(detectedConesLeftMat.cols()+detectedConesRightMat.cols());
    odcore::data::Container c0(numberOfCones);
    c0.setSenderStamp(m_senderStamp);
    getConference().send(c0);    

    int type = 1;
    auto startLeft = std::chrono::system_clock::now();
    sendMatchedContainer(detectedConesLeftMat, type, 0);
    type = 2;
    sendMatchedContainer(detectedConesRightMat, type, detectedConesLeftMat.cols());
    auto finishRight = std::chrono::system_clock::now();
    auto timeSend = std::chrono::duration_cast<std::chrono::microseconds>(finishRight - startLeft);
    std::cout << "sendTIme:" << timeSend.count() << std::endl;

    //std::cout << "detectedConesLeftMat: " << detectedConesLeftMat.transpose() << std::endl;
    //std::cout << "detectedConesRightMat: " << detectedConesRightMat.transpose() << std::endl;
    //std::cout << "THIS IS SIM-DETECTCONE SPEAKING" << std::endl;
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

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
  
  //std::cout << "Left: " << m_leftCones << std::endl;
  //std::cout << "Right: " << m_rightCones << std::endl;
  std::cout << "Small: " << m_smallCones << std::endl;
  std::cout << "Big: " << m_bigCones << std::endl;

  // std::string const exampleConfig = 
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-perception-detectcone.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void DetectCone::tearDown()
{
}

void DetectCone::readMap(std::string filename)
{
  int leftCounter = 0;
  int rightCounter = 0;
  int smallCounter = 0;
  int bigCounter = 0;

  std::string line;
  std::string word;
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
      
    }
    file.close();
  }

  ArrayXXf tmpLeftCones(leftCounter,2);
  ArrayXXf tmpRightCones(rightCounter,2);
  ArrayXXf tmpSmallCones(smallCounter,2);
  ArrayXXf tmpBigCones(bigCounter,2);
  float x;
  float y;
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

      if(word.compare("1") == 0){
        tmpLeftCones(leftCounter,0) = x;
        tmpLeftCones(leftCounter,1) = y;
        leftCounter = leftCounter+1;
      }
      else if(word.compare("2") == 0){
        tmpRightCones(rightCounter,0) = x;
        tmpRightCones(rightCounter,1) = y;
        rightCounter = rightCounter+1;
      }
      else if(word.compare("3") == 0){
        tmpSmallCones(smallCounter,0) = x;
        tmpSmallCones(smallCounter,1) = y;
        smallCounter = smallCounter+1;
      }
      else if(word.compare("4") == 0){
        tmpBigCones(bigCounter,0) = x;
        tmpBigCones(bigCounter,1) = y;
        bigCounter = bigCounter+1;
      }
      else{std::cout << "ERROR in DetectCone::simDetectCone while storing cones. Not a valid cone type." << std::endl;}
    }
    myFile.close();
  }

  m_leftCones = tmpLeftCones;
  m_rightCones = tmpRightCones;
  m_smallCones = tmpSmallCones;
  m_bigCones = tmpBigCones;
}

ArrayXXf DetectCone::simConeDetectorBox(ArrayXXf globalMap, ArrayXXf location, float heading, float detectRange, float detectWidth)
{
  // Input: Positions of cones and vehicle, heading angle, detection ranges forward and to the side
  // Output: Local coordinates of the cones within the specified area

std::cout << "Entered detector " << std::endl;

int nCones = globalMap.rows();

MatrixXf rotationMatrix(2,2);
rotationMatrix << std::cos(-heading),-std::sin(-heading),
                  std::sin(-heading),std::cos(-heading);

ArrayXXf tmpLocation(nCones,2);
(tmpLocation.col(0)).fill(location(0));
(tmpLocation.col(1)).fill(location(1));

ArrayXXf localMap = ((rotationMatrix*(((globalMap-tmpLocation).matrix()).transpose())).transpose()).array();

ArrayXXf detectedCones(nCones,2);
bool inLongitudinalInterval;
bool inLateralInterval;
int nFound = 0;
for(int i = 0; i < nCones; i = i+1)
{
  inLongitudinalInterval = localMap(i,0) < detectRange && localMap(i,0) >= 0;
  inLateralInterval = localMap(i,1) >= -detectWidth/2 && localMap(i,1) <= detectWidth/2;
  if(inLongitudinalInterval && inLateralInterval)
  {
    detectedCones.row(nFound) = localMap.row(i);
    nFound = nFound+1;
  }
} // End of for

ArrayXXf detectedConesFinal = detectedCones.topRows(nFound);

//std::cout << "globalMap: " << globalMap << std::endl;
//std::cout << "location: " << location << std::endl;
//std::cout << "heading: " << heading << std::endl;
//std::cout << "detectRange: " << detectRange << std::endl;
//std::cout << "detectWidth: " << detectWidth << std::endl;

return detectedConesFinal;
}

void DetectCone::sendMatchedContainer(Eigen::MatrixXd cones, int type, int startID)
{
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
  }
}

void DetectCone::Cartesian2Spherical(double x, double y, double z, opendlv::logic::sensation::Point &pointInSpherical)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan2(x,y)*static_cast<double>(RAD2DEG);
  double zenithAngle = atan2(z,sqrt(x*x+y*y))*static_cast<double>(RAD2DEG);
  pointInSpherical.setDistance(distance);
  pointInSpherical.setAzimuthAngle(azimuthAngle);
  pointInSpherical.setZenithAngle(zenithAngle);
}

}
}
}
}
