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
#include <opendavinci/odcore/base/Mutex.h>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

#include "opendavinci/generated/odcore/data/SharedPointCloud.h"
#include "opendavinci/generated/odcore/data/CompactPointCloud.h"

#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <opendlv/data/environment/Point3.h> 

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {


using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::wrapper;


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
  
  void SaveOneCPCPointNoIntensity(const int &pointIndex,const uint16_t &distance_integer, const double &azimuth, const double &verticalAngle, const uint8_t &distanceEncoding);
  //void SaveCPC32NoIntensity(const uint8_t &part, const uint8_t &entriesPerAzimuth, const double &startAzimuth, const double &endAzimuth, const uint8_t &distanceEncoding);
  void SavePointCloud();
  void ConeDetection();
  vector<vector<uint32_t>> NNSegmentation(MatrixXd &pointCloudConeROI, const double &connectDistanceThreshold);
  vector<vector<uint32_t>> FindConesFromObjects(MatrixXd &pointCloudConeROI, vector<vector<uint32_t>> &objectIndexList, const double &minNumOfPointsForCone, const double &maxNumOfPointsForCone, const double &nearConeRadiusThreshold, const double &farConeRadiusThreshold, const double &zRangeThreshold);
  MatrixXd ExtractConeROI(const double &xBoundary, const double &yBoundary, const double &groundLayerZ,  const double &coneHeight);
  double CalculateXYDistance(Eigen::MatrixXd &pointCloud, const uint32_t &index1, const uint32_t &index2);
  double CalculateConeRadius(Eigen::MatrixXd &potentialConePointCloud);
  double GetZRange(Eigen::MatrixXd &potentialConePointCloud);
  void SendingConesPositions(Eigen::MatrixXd &pointCloudConeROI, vector<vector<uint32_t>> &coneIndexList);
  opendlv::logic::sensation::Point Cartesian2Spherical(double &x, double &y, double &z);
  Eigen::MatrixXd RANSACRemoveGround(MatrixXd);
  Eigen::MatrixXd RemoveDuplicates(MatrixXd);


  // Define constants to decode CPC message
  const double START_V_ANGLE = -15.0; //For each azimuth there are 16 points with unique vertical angles from -15 to 15 degrees
  const double V_INCREMENT = 2.0; //The vertical angle increment for the 16 points with the same azimuth is 2 degrees
  const double START_V_ANGLE_32 = -30.67; //The starting angle for HDL-32E. Vertical angle ranges from -30.67 to 10.67 degress, with alternating increment 1.33 and 1.34
  const double V_INCREMENT_32_A = 1.33; //The first vertical angle increment for HDL-32E
  const double V_INCREMENT_32_B = 1.34; //The second vertical angle increment for HDL-32E

  // Constants for degree transformation
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;

  // Variables to handle HDL32 3 parts of message
  uint8_t m_12_startingSensorID_32; //From which layer for the first part(12 layers) of CPC for HDL-32E
  uint8_t m_11_startingSensorID_32; //From which layer for the second part(11 layers) of CPC for HDL-32E
  uint8_t m_9_startingSensorID_32; //From which layer for the third part(9 layers) of CPC for HDL-32E
  array<double, 12>  m_12_verticalAngles; //Store the 12 vertical angles for the first part (including 12 layers) of CPC for HDL-32E
  array<double, 11> m_11_verticalAngles; //Store the 11 vertical angles for the second part (including 11 layers) of CPC for HDL-32E
  array<double, 9> m_9_verticalAngles; //Store the 9 vertical angles for the third part (including 9 layers) of CPC for HDL-32E
  string m_12_cpcDistance_32; //The distance string for the first part of CPC for HDL-32E    
  string m_11_cpcDistance_32; //The distance string for the second part of CPC for HDL-32E    
  string m_9_cpcDistance_32; //The distance string for the third part of CPC for HDL-32E
  uint64_t m_previousCPC32TimeStamp;//The sample time of the previous CPC container belonging to a HDL-32E scan
  uint8_t m_cpcMask_32; //The lowest 3 bits represent which part(s) of HDL-32E CPC of the same scan has been received. 0100 means the first part has arrived; 0010 means the second part has arrived; 0001 means the third part has arrived.
  
  // Class variables to store Lidar message
  CompactPointCloud m_cpc;
  odcore::base::Mutex m_cpcMutex;
  bool m_SPCReceived;//Set to true when the first shared point cloud is received
  bool m_CPCReceived;//Set to true when the first compact point cloud is received
  uint32_t m_recordingYear;//The year when a recording with CPC was taken

  // Class variables to save point cloud 
  MatrixXd m_pointCloud;
  int m_pointIndex;
  // Define constants and thresolds forclustering algorithm
  double m_xBoundary;
  double m_yBoundary;
  double m_groundLayerZ;
  double m_coneHeight;
  double m_connectDistanceThreshold;
  double m_layerRangeThreshold;
  uint16_t m_minNumOfPointsForCone;
  uint16_t m_maxNumOfPointsForCone;
  double m_farConeRadiusThreshold;
  double m_nearConeRadiusThreshold;
  double m_zRangeThreshold;
  odcore::data::TimeStamp m_CPCReceivedLastTime;
  double m_algorithmTime;
  Eigen::MatrixXd m_generatedTestPointCloud;
  // RANSAC thresholds
  double m_inlierRangeThreshold;
  double m_inlierFoundTreshold;
  double m_ransacIterations;
  double m_dotThreshold;
  uint32_t m_senderStamp = 0;
  Eigen::MatrixXd m_lastBestPlane;
  


};

}
}
}
}

#endif
