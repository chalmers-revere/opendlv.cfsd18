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
{
  m_coneCollector = Eigen::MatrixXd::Zero(4,20);
  m_lastObjectId = 0;
}

Slam::~Slam()
{
}



void Slam::nextContainer(odcore::data::Container &a_container)
{
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
      if(true){
          //SetConesToMap(extractedCones);
        }
      }
      //Initialize for next collection
      m_lastTimeStamp = timeStamp;
      m_lastObjectId = 0;
      m_coneCollector = Eigen::MatrixXd::Zero(4,20);
    }
		return true;
}

void Slam::setUp()
{
  auto kv = getKeyValueConfiguration();
  m_timeDiffMilliseconds = kv.getValue<double>("logic-cfsd18-perception-detectcone.timeDiffMilliseconds");
  
}

void Slam::tearDown()
{
}

}
}
}
}
