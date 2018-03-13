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

#include "detectcone.hpp"

namespace opendlv {
namespace sim {
namespace cfsd18 {
namespace perception {

DetectCone::DetectCone(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "sim-cfsd18-perception-detectcone")
{
}

DetectCone::~DetectCone()
{
}



void DetectCone::nextContainer(odcore::data::Container &a_container)
{

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

ArrayXXf detectedConesLeft = DetectCone::simConeDetectorBox(sideLeft, location, heading, detectRange, detectWidth);
ArrayXXf detectedConesRight = DetectCone::simConeDetectorBox(sideRight, location, heading, detectRange, detectWidth);
std::cout << "detectedConesLeft: " << detectedConesLeft << std::endl;
std::cout << "detectedConesRight: " << detectedConesRight << std::endl;

  std::cout << "THIS IS SIM-DETECTCONE SPEAKING" << std::endl;
  if (a_container.getDataType() == opendlv::logic::sensation::Attention::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();

    opendlv::logic::perception::Object o1;
    odcore::data::Container c1(o1);
    getConference().send(c1);
  }
}

void DetectCone::setUp()
{
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

}
}
}
}
