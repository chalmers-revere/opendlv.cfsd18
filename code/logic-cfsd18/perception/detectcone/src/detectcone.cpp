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

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"
#include "opendavinci/odcore/wrapper/SharedMemory.h"
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <opendavinci/generated/odcore/data/CompactPointCloud.h>

#include <opendavinci/odcore/data/TimeStamp.h>

#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

//#include "odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h"

#include "detectcone.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

using namespace std;
using namespace Eigen;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::wrapper;

DetectCone::DetectCone(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-perception-detectcone")
 , m_lastLidarData()
 , m_lastCameraData()
 , m_pointMatched()
 , m_diffVec()
 , m_finalPointCloud()
 , m_threshold()
 , m_timeDiffMilliseconds()
 , m_lastTimeStamp()
 , m_coneCollector()
 , m_coneCollected()
{

  m_diffVec = 0;
  m_pointMatched = MatrixXd::Zero(4,1);
  m_lastCameraData = MatrixXd::Zero(4,1);
  m_lastLidarData = MatrixXd::Zero(4,1);
  m_coneCollector = MatrixXd::Zero(4,10);
  m_coneCollected = 0;
}

DetectCone::~DetectCone()
{
}



void DetectCone::nextContainer(odcore::data::Container &a_container)
{



  if (a_container.getDataType() == opendlv::logic::sensation::Point::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
    
    //Retrive data and timestamp
    odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    auto point = a_container.getData<opendlv::logic::sensation::Point>();

    //Check last timestamp if they are from same message

    cout << "Message Recieved " << endl;
    if (((timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds()) < m_timeDiffMilliseconds*1000)){
      cout << "Test 2 " << endl;
      m_coneCollector.col(m_coneCollected+1) << point.getAzimuthAngle(),
                                              point.getZenithAngle(),
                                              point.getDistance(),
                                              0;

      m_coneCollected++;
      m_lastTimeStamp = timeStamp;
      cout << "FoundCone: " << endl;
      cout << m_coneCollector << endl;

    }
    else{
      //All object candidates collected, to sensor fusion
      cout << "Extracted Cones " << endl;
      MatrixXd extractedCones;
      extractedCones = m_coneCollector.leftCols(m_coneCollected);
      cout << extractedCones << endl;
      SendCollectedCones(extractedCones);

      //Initialize for next collection
      m_lastTimeStamp = timeStamp;
      m_coneCollected = 0;
      m_coneCollector = MatrixXd::Zero(4,10);
      m_coneCollector.col(0) << point.getAzimuthAngle(),
                                point.getZenithAngle(),
                                point.getDistance(),
                                0;
    }

  }

}

void DetectCone::matchPoints(MatrixXd lidar, MatrixXd camera)
{
  //Index vars
  int colFinalPoints = 1;
  int index = 0; 
  //Initialize zero matricies
  MatrixXd tempPointLidar = MatrixXd::Zero(4,1);
  MatrixXd tempPointCamera = MatrixXd::Zero(4,1);
  m_finalPointCloud = MatrixXd::Zero(4,lidar.cols());
  MatrixXd diffVec = MatrixXd::Zero(1,lidar.cols());
  
  //Pick i:th found lidar object
  for (int i = 0; i < lidar.cols(); i++){
    //Reset match check for each lidar point
    bool matchFound = false;
    //Loop through all found camera objects  
    for (int j = 0; j < camera.cols(); j++){  
      //store in temporary variables as input in findMatch
      tempPointLidar.col(0) = lidar.col(i);
      tempPointCamera.col(0) = camera.col(j);
         
      findMatch(tempPointLidar, tempPointCamera);
      //Store range difference of i lidar point, j camera point
      diffVec(0,j) = m_diffVec;

    }
    //Reset
    m_diffVec=1000000;

    //Iterate through all points to find the closest camera object j to lidar object to current i
    for (int k = 0; k < diffVec.cols(); k++){

      if (m_diffVec > diffVec(0,k) && diffVec(0,k) > 0 ){
        m_diffVec = diffVec(0,k);
        index = k;
        matchFound = true;
      }

    } 

    //If no match is found, store object as a cone without classification, else use index found to classify cone
    if(!matchFound) {
      m_finalPointCloud(0,colFinalPoints-1) = lidar(0,i);
      m_finalPointCloud(1,colFinalPoints-1) = lidar(1,i);
      m_finalPointCloud(2,colFinalPoints-1) = lidar(2,i);
      m_finalPointCloud(3,colFinalPoints-1) = 0;
      colFinalPoints++;
    }
    else{    
      m_finalPointCloud(0,colFinalPoints-1) = lidar(0,i);
      m_finalPointCloud(1,colFinalPoints-1) = lidar(1,i);
      m_finalPointCloud(2,colFinalPoints-1) = lidar(2,i);
      m_finalPointCloud(3,colFinalPoints-1) = camera(3,index);
      colFinalPoints++;   
    }   
  }
}

void DetectCone::findMatch(MatrixXd lidarPoint, MatrixXd cameraPoint)
{
  //Calculate distance between lidar and camera points
  MatrixXd tempNorm = MatrixXd::Zero(2,1);
  tempNorm(0,0) = lidarPoint(0,0)-cameraPoint(0,0);
  tempNorm(1,0) = lidarPoint(1,0)-cameraPoint(1,0);
  m_diffVec = tempNorm.norm();

  //below threshold results in point added
  if(m_diffVec < m_threshold){
    
      m_pointMatched(0,0) = lidarPoint(0,0);    
      m_pointMatched(1,0) = lidarPoint(1,0);
      m_pointMatched(2,0) = lidarPoint(2,0);
      m_pointMatched(3,0) = cameraPoint(3,0);
  } 
  else{

    m_pointMatched = MatrixXd::Zero(4,1);
    m_diffVec = 0;
  }
      
}

MatrixXd DetectCone::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{

  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  double xData = distance * sin(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * sin(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * cos(zenimuth * static_cast<double>(DEG2RAD));
  MatrixXd recievedPoint(4,1);
  recievedPoint << xData,
                    yData,
                    zData,
                    0; 
  return recievedPoint;

}
opendlv::logic::sensation::Point DetectCone::Cartesian2Spherical(double x, double y, double z)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan(y/x)*static_cast<double>(RAD2DEG);
  double zenithAngle = atan(sqrt(x*x+y*y)/z)*static_cast<double>(RAD2DEG);
  logic::sensation::Point pointInSpherical;
  pointInSpherical.setDistance(distance);
  pointInSpherical.setAzimuthAngle(azimuthAngle);
  pointInSpherical.setZenithAngle(zenithAngle);
  return pointInSpherical;

}

void DetectCone::SendCollectedCones(MatrixXd lidarCones)
{

  //Convert to cartesian

  for(int p = 0; p < lidarCones.cols(); p++){

    lidarCones.col(p) = Spherical2Cartesian(lidarCones(0,p), lidarCones(1,p), lidarCones(2,p));

  }
  cout << "lidarCones " << endl;
  cout << lidarCones << endl;
  MatrixXd cameraCones = MatrixXd::Zero(4,2);
  cameraCones << 1.7,  5.6,
                 1.8,  2.4,
                 0.1, 0,
                 1, 2;

  cout << "CameraCones " << endl;
  cout << cameraCones << endl;
  matchPoints(lidarCones, cameraCones);
  cout << "matched: " << endl;
  cout << m_finalPointCloud << endl;

  SendMatchedContainer(m_finalPointCloud);
}

void DetectCone::SendMatchedContainer(MatrixXd cones)
{

   for(int n = 0; n < cones.cols(); n++){

     opendlv::logic::sensation::Point conePoint = Cartesian2Spherical(cones(0,n), cones(1,n), cones(2,n));
     odcore::data::Container c1(conePoint);
     getConference().send(c1);
      cout << "a point sent out with distance: " <<conePoint.getDistance() <<"; azimuthAngle: " << conePoint.getAzimuthAngle() << "; and zenithAngle: " << conePoint.getZenithAngle() << endl;
    
   }  
}

void DetectCone::setUp()
{

  auto kv = getKeyValueConfiguration();
  m_threshold = kv.getValue<double>("logic-cfsd18-perception-detectcone.threshold");
  m_timeDiffMilliseconds = kv.getValue<double>("logic-cfsd18-perception-detectcone.timeDiffMilliseconds");
  cout << "setup OK " << endl;

  /*MatrixXd tDataC = MatrixXd::Zero(4,15);
  tDataC << 156.070834564403,  184.601466470279,  12.89492281629822,  181.922585263508,  38.4423334159231,  95.2869772168372,  108.050576050148,  159.876418507748,  104.572187872226,  125.994541306663,  67.7155269296972,  121.351729955313,  95.5718038179233,  112.490754168257,  187.325804192157,
            34.6488968771459,  470.3156219435640,  4.70876340071653,  450.2937622172337,  70.91722137374080,  190.4989903743118,  250.1613172118744,  390.0775417218023,  25.8791158832663,  27.5842177337360,  130.9307211912096,  280.8746836179155,  22.4613982170025,  23.9103149023747,  47.6730787639961,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 2, 2, 2, 1, 1, 2, 2, 2, 1, 1, 2, 2, 1, 2;
  MatrixXd tDataL = MatrixXd::Zero(4,15);
  tDataL << 156.113102969901, 184.340612945136, 2.95886782990784,  181.607675867664,  38.1937859114006,  95.1211415834165,  108.589335322046,  159.749352541238,  104.376743419275,  125.372590173807,  67.3038043182645,  121.157040090890,  95.4920643647510,  112.990079345581,  187.174929372708,
            34.6906654901397,  47.3138030228940,  3.92112855089666,  45.9625755471868,  8.06192588819332,  20.3616390707633,  26.2846191262603,  39.3565285744002,  25.0452408905100,  26.9123104216864,  13.7414484368171,  28.9177366890745,  23.0568582810388,  24.0943009038242,  47.9602146952051,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  matchPoints(tDataL, tDataC);
  cout << "Points matched 1:" << endl;
  cout << m_finalPointCloud << endl;

  tDataC = MatrixXd::Zero(4,10);
  tDataC << 156.070834564403,  184.601466470279,  2.89492281629822,  181.922585263508,  38.4423334159231,  95.2869772168372,  108.050576050148,  159.876418507748,  104.572187872226,  125.994541306663,
            34.6488968771459,  47.3156219435640,  4.70876340071653,  45.2937622172337,  7.91722137374080,  19.4989903743118,  25.1613172118744,  39.0775417218023,  25.8791158832663,  27.5842177337360,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 2, 2, 2, 1, 1, 2, 2, 2, 1;
  tDataL = MatrixXd::Zero(4,10);
  tDataL << 156.113102969901, 184.340612945136, 2.95886782990784,  181.607675867664,  38.1937859114006,  95.1211415834165,  108.589335322046,  159.749352541238,  104.376743419275,  125.372590173807,
            34.6906654901397,  47.3138030228940,  3.92112855089666,  45.9625755471868,  8.06192588819332,  20.3616390707633,  26.2846191262603,  39.3565285744002,  25.0452408905100,  26.9123104216864,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  matchPoints(tDataL, tDataC);
  cout << "Points matched 2:" << endl;
  cout << m_finalPointCloud << endl;

    tDataC = MatrixXd::Zero(4,2);
  tDataC << 156.070834564403,  184.601466470279,
            34.6488968771459,  47.3156219435640,
            0, 0,
            1, 2;
  tDataL = MatrixXd::Zero(4,2);
  tDataL << 156.113102969901, 184.340612945136,
            34.6906654901397,  47.3138030228940,
            0, 0,
            0, 0;
  matchPoints(tDataL, tDataC);
  cout << "Points matched 3:" << endl;
  cout << m_finalPointCloud << endl;*/
  


}



void DetectCone::tearDown()
{
}

}
}
}
}
