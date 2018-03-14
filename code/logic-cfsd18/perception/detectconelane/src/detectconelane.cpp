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


    }
    
}    
    //std::ofstream outfile;
    //std::cout << "I am writing to txt!!!!!!!!!" << std::endl;
    //outfile.open("/opt/testdata/coneLocal.txt", std::ios::out | std::ios::app); // opens file named "filename" for output
    //outfile << coneLocal << std::endl;//saves "Hello" to the outfile with the insertion operator
    //outfile.close();// closes file; always do this when you are done using the file
    





}
}
}
}

