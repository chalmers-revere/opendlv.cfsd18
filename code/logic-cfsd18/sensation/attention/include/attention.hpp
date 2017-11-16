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

#ifndef OPENDLV_LOGIC_CFSD18_SENSATION_ATTENTION_HPP
#define OPENDLV_LOGIC_CFSD18_SENSATION_ATTENTION_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>

//#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

#include "opendavinci/generated/odcore/data/SharedPointCloud.h"
#include "opendavinci/generated/odcore/data/CompactPointCloud.h"

#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {


using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::wrapper;
using namespace Eigen;

class Attention : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Attention(int32_t const &, char **);
  Attention(Attention const &) = delete;
  Attention &operator=(Attention const &) = delete;
  virtual ~Attention();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
  
  void SaveOneCPCPointNoIntensity(const int &pointIndex,const uint16_t &distance_integer, const float &azimuth, const float &verticalAngle, const uint8_t &distanceEncoding);
  void SaveCPC32NoIntensity(const uint8_t &part, const uint8_t &entriesPerAzimuth, const float &startAzimuth, const float &endAzimuth, const uint8_t &distanceEncoding);
 // void SaveCPC32WithIntensity(const uint8_t &part, const uint8_t &entriesPerAzimuth, const float &startAzimuth, const float &endAzimuth, const uint8_t &distanceEncoding, const uint8_t &numberOfBitsForIntensity, const uint8_t &intensityPlacement, const uint16_t &mask, const float &intensityMaxValue);
  void SavePointCloud();

  const float START_V_ANGLE = -15.0; //For each azimuth there are 16 points with unique vertical angles from -15 to 15 degrees
  const float V_INCREMENT = 2.0; //The vertical angle increment for the 16 points with the same azimuth is 2 degrees
  const float START_V_ANGLE_32 = -30.67; //The starting angle for HDL-32E. Vertical angle ranges from -30.67 to 10.67 degress, with alternating increment 1.33 and 1.34
  const float V_INCREMENT_32_A = 1.33; //The first vertical angle increment for HDL-32E
  const float V_INCREMENT_32_B = 1.34; //The second vertical angle increment for HDL-32E

  const double DEG2RAD = 0.017453292522222; // PI/180.0

  uint8_t m_12_startingSensorID_32; //From which layer for the first part(12 layers) of CPC for HDL-32E
  uint8_t m_11_startingSensorID_32; //From which layer for the second part(11 layers) of CPC for HDL-32E
  uint8_t m_9_startingSensorID_32; //From which layer for the third part(9 layers) of CPC for HDL-32E
  array<float, 12>  m_12_verticalAngles; //Store the 12 vertical angles for the first part (including 12 layers) of CPC for HDL-32E
  array<float, 11> m_11_verticalAngles; //Store the 11 vertical angles for the second part (including 11 layers) of CPC for HDL-32E
  array<float, 9> m_9_verticalAngles; //Store the 9 vertical angles for the third part (including 9 layers) of CPC for HDL-32E
  string m_12_cpcDistance_32; //The distance string for the first part of CPC for HDL-32E    
  string m_11_cpcDistance_32; //The distance string for the second part of CPC for HDL-32E    
  string m_9_cpcDistance_32; //The distance string for the third part of CPC for HDL-32E


  uint64_t m_previousCPC32TimeStamp;//The sample time of the previous CPC container belonging to a HDL-32E scan
  uint8_t m_cpcMask_32; //The lowest 3 bits represent which part(s) of HDL-32E CPC of the same scan has been received. 0100 means the first part has arrived; 0010 means the second part has arrived; 0001 means the third part has arrived.
  CompactPointCloud m_cpc;
  odcore::base::Mutex m_cpcMutex;
  bool m_SPCReceived;//Set to true when the first shared point cloud is received
  bool m_CPCReceived;//Set to true when the first compact point cloud is received
  uint32_t m_recordingYear;//The year when a recording with CPC was taken

  MatrixXf m_pointCloud;
  bool m_isFirstPoint;
  int m_pointIndex;


};

}
}
}
}

#endif
