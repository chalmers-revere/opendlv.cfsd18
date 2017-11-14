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

//#include "odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h"

#include "detectcone.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

using namespace std;
using namespace Eigen;

DetectCone::DetectCone(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-perception-detectcone")
 , m_lastLidarData()
 , m_lastCameraData()
 , m_pointMatched()
 , m_diffVec()
 , m_finalPointCloud()
 , m_threshold()
{

  m_diffVec = 0;
  m_pointMatched = MatrixXd::Zero(4,1);
  m_lastCameraData = MatrixXd::Zero(4,1);
  m_lastLidarData = MatrixXd::Zero(4,1);
}

DetectCone::~DetectCone()
{
}



void DetectCone::nextContainer(odcore::data::Container &a_container)
{
  /*if (a_container.getDataType() == opendlv::logic::sensation::Attention::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();

    opendlv::logic::perception::Object o1;
    odcore::data::Container c1(o1);
    getConference().send(c1);

  }*/

    /*if (data == opendlv::body::SensorInfo::ID()){
    When i get timestamp
    odcore::data::TimeStamp timeStampLidar = recievedPoints.getReceivedTimeStamp();

    auto lidarData = recievedPoints.getData<opendlv::proxy::MessageforLidar>();
    Extract signal ID
    const int signalID = static_cast<int>(lidarData.getSignalID());

    switch(signalID) {
      case 1:
            Get timestamp from sender
            odcore::data::TimeStamp timeStampLidar = recievedPoints.getSentTimeStamp();
            Eigen::MatrixXD<double> lidarPoints(3,3); Check size
            static_cast<double>(lidarData.getPointCloud());
            m_lastLidarData = lidarPoints;
        break;
      case 2:
            odcore::data::TimeStamp timeStampCamera = recievedPoints.getSentTimeStamp();
            Eigen::MatrixXD<double> cameraPoints(3,3); //Check Size
            static_cast<double>(lidarData.getPointCloud());
            m_lastCameraData = cameraPoints;
        break;
    }

    
  }

  Check format of timestamp (Class - Class = new class, only for timestamp class!!)
  if (((timeStampLidar - timeStampCamera).toMicroSeconds() < 10000)){ //Read parameter
  }*/

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

void DetectCone::setUp()
{

  auto kv = getKeyValueConfiguration();
  double const threshold = kv.getValue<double>("logic-cfsd18-perception-detectcone.threshold");
  m_threshold = threshold;

  /*------------TEST CASE ---------------
  MatrixXd tDataC = MatrixXd::Zero(4,15);
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
  cout << "Points matched:" << endl;
  cout << m_finalPointCloud << endl;
  ----------------------------------------
  */
}

void DetectCone::tearDown()
{
}

}
}
}
}
