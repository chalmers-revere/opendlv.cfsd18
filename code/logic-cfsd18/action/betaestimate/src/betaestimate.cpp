#include <iostream>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include "betaestimate.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

Betaestimate::Betaestimate(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-perception-betaestimate")
, m_coneCollector()
, coneNum()
, m_pastMaps()

{
    m_coneCollector = Eigen::MatrixXd::Zero(4,20);
    coneNum = 0;
}

Betaestimate::~Betaestimate()
{
}



void Betaestimate::nextContainer(odcore::data::Container &a_container)
{/*
  //std::cout << "I am in Betaestimate!" << std::endl;
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
}
/*
void Betaestimate::CheckContainer(uint32_t objectId){
	if (objectId == 0){
		rebuildLocalMap();
		m_coneCollector = Eigen::MatrixXd::Zero(4,20);
	    coneNum = 0;
	}
}

// copy from perception-detectcone
Eigen::MatrixXd Betaestimate::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
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

void Betaestimate::rebuildLocalMap()
{
	//Convert to cartesian
	Eigen::MatrixXd cone;
	Eigen::MatrixXd coneLocal = Eigen::MatrixXd::Zero(2,coneNum);
    for(int p = 0; p < coneNum; p++){
        cone = Spherical2Cartesian(m_coneCollector(0,p), m_coneCollector(1,p), m_coneCollector(2,p));
        //m_coneCollector.col(p) = cone;
        coneLocal.col(p) = cone.topRows(2);
    }
}

void Betaestimate::newFrame(){
  int lookback = 5;
  if (m_pastMaps.size() <= lookback) {
    m_pastMaps.push_back(m_coneCollector);
  } else {
    for (int i = 1; i<lookback; i++){
      m_pastMaps[lookback-i] = m_pastMaps[lookback-i-1];
    }
  }

}
*/
void Betaestimate::setUp()
{
  // std::string const exampleConfig =
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-perception-betaestimate.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Betaestimate::tearDown()
{
}

}
}
}
}
