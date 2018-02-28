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

#include "slam.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {

Slam::Slam(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-sensation-slam")
, m_timeDiffMilliseconds()
, m_lastTimeStamp()
, m_coneCollector()
, m_lastObjectId()
, m_coneMutex()
, m_sensorMutex()
, m_mapMutex()
, m_odometryData()
, m_gpsReference()
, m_map()
, m_newConeThreshold()
{
  m_coneCollector = Eigen::MatrixXd::Zero(4,20);
  m_lastObjectId = 0;
  m_odometryData << 0,0,0;
}

Slam::~Slam()
{
}



void Slam::nextContainer(odcore::data::Container &a_container)
{


  //#####################Recieve Landmarks###########################
  if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID()) {
    odcore::base::Lock lockCone(m_coneMutex);
    std::cout << "Recieved Direction" << std::endl;
    //Retrive data and timestamp
    odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
		uint32_t objectId = coneDirection.getObjectId();

    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (CheckContainer(objectId,timeStamp)){
      //std::cout << "Test 2 " << std::endl;
      m_coneCollector(0,objectId) = coneDirection.getAzimuthAngle();
			m_coneCollector(1,objectId) = coneDirection.getZenithAngle();
    }

  }
	if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()){
    odcore::base::Lock lockCone(m_coneMutex);
    std::cout << "Recieved Distance" << std::endl;
		odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
		uint32_t objectId = coneDistance.getObjectId();

    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (CheckContainer(objectId,timeStamp)){
      m_coneCollector(2,objectId) = coneDistance.getDistance();
			m_coneCollector(3,objectId) = 0;
    }
  }

  //#########################Recieve Odometry##################################
  if(a_container.getDataType() == opendlv::logic::sensation::Geolocation::ID()){
   
    odcore::base::Lock lockSensor(m_sensorMutex);
    auto odometry = a_container.getData<opendlv::logic::sensation::Geolocation>();

    double longitude = odometry.getLongitude();
    double latitude = odometry.getLatitude();
    opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
    opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);

    m_odometryData << gpsTransform.getX(),
                      gpsTransform.getY(),
                      odometry.getHeading();
  }



  //When new frame
     //If candidate do SLAM (euclidian distance for two closest landmarks)




  //Else keep collecting

  /*opendlv::data::environment::Point3 xyz;
  xyz.setX(x(0));
  xyz.setY(x(1));
  xyz.setZ(0);
  opendlv::data::environment::WGS84Coordinate gpsCurrent = m_gpsReference.transform(xyz);
  opendlv::logic::sensation::Geolocation geoState;
  geoState.setLatitude(gpsCurrent.getLatitude());
  geoState.setLongitude(gpsCurrent.getLongitude());
  geoState.setHeading(x(5));*/


}

bool Slam::CheckContainer(uint32_t objectId, odcore::data::TimeStamp timeStamp){
    if (((timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds()) < m_timeDiffMilliseconds*1000)){
      m_lastObjectId = (m_lastObjectId<objectId)?(objectId):(m_lastObjectId);
      m_lastTimeStamp = timeStamp;

    }
    else {
      //All object candidates collected, to sensor fusion
      Eigen::MatrixXd extractedCones;
      extractedCones = m_coneCollector.leftCols(m_lastObjectId+1);
      if(extractedCones.cols() > 1){
      std::cout << "Extracted Cones " << std::endl;
      std::cout << extractedCones << std::endl;
      if(isKeyframe(extractedCones)){
          performSLAM(extractedCones);//Thread?
        }
      }
      //Initialize for next collection
      m_lastTimeStamp = timeStamp;
      m_lastObjectId = 0;
      m_coneCollector = Eigen::MatrixXd::Zero(4,20);
    }
  return true;
}

bool Slam::isKeyframe(Eigen::MatrixXd Cones){
//Check if keyframe candidate
  Eigen::MatrixXd temp = Cones;
  return true;
}


void Slam::performSLAM(Eigen::MatrixXd cones){
  Eigen::Vector3d pose;
  {
    odcore::base::Lock lockSensor(m_sensorMutex);
    pose = m_odometryData;
  }
  Eigen::MatrixXd xyCones = conesToGlobal(pose, cones);
  addConesToMap(xyCones);
  //addKeyframeToGraph(
  
}

Eigen::MatrixXd Slam::conesToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd cones){
  Eigen::MatrixXd xyCones = Eigen::MatrixXd::Zero(3,20);
  for(int i = 0; i<cones.cols();i++){
    Eigen::Vector3d cone = Spherical2Cartesian(cones(0,i), cones(1,i), cones(2,i));
    cone(0) += pose(0);
    cone(1) += pose(1);
    xyCones(0,i) = cone(0);
    xyCones(1,i) = cone(1);
    xyCones(2,i) = cones(3,i);
  }
  return xyCones;
}

void Slam::addConesToMap(Eigen::MatrixXd cones){
  odcore::base::Lock lockMap(m_mapMutex);
  for(int i = 0; i<cones.cols(); i++){
    if(newCone(cones.col(i))){
       Cone cone = Cone(cones(0,i),cones(1,i),(int)cones(2,i),23); //Temp id, think of system later
       m_map.push_back(cone);
    }
  }
}    
      


bool Slam::newCone(Eigen::MatrixXd cone){
  for(uint32_t i = 0; i<m_map.size(); i++){
    if(fabs(m_map[i].getProperty() - cone(2))<0.0001){
      double distance = (m_map[i].getX()-cone(0))*(m_map[i].getX()-cone(0))+(m_map[i].getY()-cone(1))*(m_map[i].getY()-cone(1));
      distance = std::sqrt(distance);
      if(distance<m_newConeThreshold){
	return false;
      }
    }
  }
  return true;
}
  
//Get the new pose
//Convert cones from local to global coordinate system
//Add eventual new cones to the global map
//Convert the cone observation to a g2o node and a edge with the new pose
//Try to match cones to already observed cones 
//Add the pose and make a odometry measurement to the previous pose if applicable
//(Perform localization maybe)
//Loop closing Find orange cones
//Update the global map
//When map is updated send out cones in local spherical coordinates(along with color info)


Eigen::Vector3d Slam::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Vector3d::Zero();
  recievedPoint << xData,
                   yData,
                   zData;
  return recievedPoint;
}

void Slam::setUp()
{
  auto kv = getKeyValueConfiguration();
  m_timeDiffMilliseconds = kv.getValue<double>("logic-cfsd18-perception-detectcone.timeDiffMilliseconds");
  m_newConeThreshold = kv.getValue<double>("logic-cfsd18-sensation-slam.newConeLimit");

  double const latitude = getKeyValueConfiguration().getValue<double>("logic-sensation-geolocator.GPSreference.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>("logic-sensation-geolocator.GPSreference.longitude");
  m_gpsReference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);
  
}

void Slam::tearDown()
{
}

}
}
}
}
