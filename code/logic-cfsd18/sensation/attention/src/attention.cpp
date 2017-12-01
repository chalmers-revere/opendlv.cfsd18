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
#include <cmath>
#include <vector>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <opendavinci/generated/odcore/data/CompactPointCloud.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendlv/data/environment/Point3.h> 


#include "attention.hpp"


namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {

using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::wrapper;


Attention::Attention(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-sensation-attention")
  , m_12_startingSensorID_32(0)//The first HDL-32E CPC starts from Layer 0
  , m_11_startingSensorID_32(2)//The second HDL-32E CPC starts from Layer 2
  , m_9_startingSensorID_32(5)//The third HDL-32E CPC starts from Layer 5
  , m_12_verticalAngles()
  , m_11_verticalAngles()
  , m_9_verticalAngles()
  , m_12_cpcDistance_32("")
  , m_11_cpcDistance_32("")
  , m_9_cpcDistance_32("")
  , m_previousCPC32TimeStamp(0)
  , m_cpcMask_32(0)
  , m_cpc()
  , m_cpcMutex()
  , m_SPCReceived(false)
  , m_CPCReceived(false)
  , m_recordingYear()
  , m_pointCloud()
  , m_pointIndex(0)
  , m_startAngle()
  , m_endAngle()
  , m_xBoundary()
  , m_yBoundary()
  , m_groundLayerZ()
  , m_coneHeight()
  , m_connectDistanceThreshold()
  , m_layerRangeThreshold()
  , m_minNumOfPointsForCone()
  , m_maxNumOfPointsForCone()
  , m_farConeRadiusThreshold()
  , m_nearConeRadiusThreshold()
  , m_zRangeThreshold()
  , m_CPCReceivedLastTime()
  , m_algorithmTime()
  , m_generatedTestPointCloud()
  , m_inlierRangeThreshold()
  , m_inlierFoundTreshold()
  , m_ransacIterations()
  , m_dotThreshold()
  , m_lastBestPlane()
  {
  //#############################################################################
  // Following part are prepared to decode CPC of HDL32 3 parts
  //#############################################################################
  std::array<double, 32>  sensorIDs_32;
  bool use32IncrementA = true;
  sensorIDs_32[0] = START_V_ANGLE_32;
  //Derive the 32 vertical angles for HDL-32E based on the starting angle and the two increments
  for (uint8_t counter = 1; counter < 31; counter++) {
    sensorIDs_32[counter] += use32IncrementA ?  V_INCREMENT_32_A : V_INCREMENT_32_B;
    use32IncrementA = !use32IncrementA;
  }
  //Derive the 12 vertical angles associated with the first part of CPC for HDL-32E
  m_12_verticalAngles[0] = sensorIDs_32[m_12_startingSensorID_32];
  uint8_t currentSensorID = m_12_startingSensorID_32 + 1;
  m_12_verticalAngles[1] = sensorIDs_32[currentSensorID];
  for (uint8_t counter = 2; counter < 11; counter++) {
      m_12_verticalAngles[counter] = sensorIDs_32[currentSensorID + 3];    
  }
  //Derive the 11 vertical angles associated with the second part of CPC for HDL-32E
  m_11_verticalAngles[0] = sensorIDs_32[m_11_startingSensorID_32];
  currentSensorID = m_11_startingSensorID_32 + 1;
  m_11_verticalAngles[1] = sensorIDs_32[currentSensorID];
  for (uint8_t counter = 2; counter < 10; counter++) {
      m_11_verticalAngles[counter] = sensorIDs_32[currentSensorID + 3];    
  }
  //Derive the 9 vertical angles associated with the third part of CPC for HDL-32E
  m_9_verticalAngles[0] = sensorIDs_32[m_9_startingSensorID_32];
  currentSensorID = m_9_startingSensorID_32 + 1;
  m_9_verticalAngles[1] = sensorIDs_32[currentSensorID];
  for (uint8_t counter = 2; counter < 8; counter++) {
      m_9_verticalAngles[counter] = sensorIDs_32[currentSensorID + 3];    
  }
  //###########################################################################
}

Attention::~Attention()
{
}

void Attention::nextContainer(odcore::data::Container &a_container)
{
  TimeStamp incommingDataTime = a_container.getSampleTimeStamp();
  double timeSinceLastReceive = abs(static_cast<double>(incommingDataTime.toMicroseconds()-m_CPCReceivedLastTime.toMicroseconds())/1000000.0);
  cout << "Time since between 2 incomming messages: " << timeSinceLastReceive << "s" << endl;
  if (timeSinceLastReceive>m_algorithmTime)
  {

  if(a_container.getDataType() == odcore::data::SharedPointCloud::ID()){
    m_SPCReceived = true;
    cout << "Error: Don't use SharedPointCloud here!!!" << endl;
}
  if (a_container.getDataType() == odcore::data::CompactPointCloud::ID()) {
    m_CPCReceived = true;
    TimeStamp ts = a_container.getSampleTimeStamp();

    double timeBetween2ProcessedMessage = static_cast<double>(ts.toMicroseconds()-m_CPCReceivedLastTime.toMicroseconds())/1000000.0;

    cout << "Time between 2 processed messages is: " << timeBetween2ProcessedMessage << "s" << endl;

    m_CPCReceivedLastTime = ts;

    m_recordingYear = ts.getYear();

    if (!m_SPCReceived) {
      Lock lockCPC(m_cpcMutex);
      m_cpc = a_container.getData<CompactPointCloud>(); 
      const uint8_t numberOfLayers = m_cpc.getEntriesPerAzimuth();  // Get number of layers

      if (numberOfLayers == 16) {  // Deal with VLP-16
        cout << "Connecting to VLP16" <<  endl;

      }
      else {  // Deal with HDL-32
        cout << "Connecting to HDL-32" <<  endl;
        const uint64_t currentTime = ts.toMicroseconds();
        //Check if this HDL-32E CPC comes from a new scan. The interval between two scans is roughly 100ms. It is safe to assume that a new CPC comes from a new scan if the interval is longer than 50ms
        const uint64_t deltaTime = (currentTime > m_previousCPC32TimeStamp) ? (currentTime - m_previousCPC32TimeStamp) : (m_previousCPC32TimeStamp - currentTime);
        if (deltaTime > 50000) {
          m_cpcMask_32 = 0;//Reset the mask that represents which HDL-32E CPC part has arrived
        }
        m_previousCPC32TimeStamp = currentTime;
        if (numberOfLayers == 12) {
          m_cpcMask_32 = m_cpcMask_32 | 0x04;
          m_12_cpcDistance_32 = m_cpc.getDistances();
        }
        if (numberOfLayers == 11) {
          m_cpcMask_32 = m_cpcMask_32 | 0x02;
          m_11_cpcDistance_32 = m_cpc.getDistances();
        }
        if (numberOfLayers == 9) {
          m_cpcMask_32 = m_cpcMask_32 | 0x01;
          m_9_cpcDistance_32 = m_cpc.getDistances();
        }
      

      }
    }
  }
  odcore::data::TimeStamp TimeBeforeAlgorithm;
  SavePointCloud();
  ConeDetection();

  odcore::data::TimeStamp TimeAfterAlgorithm;
  double timeForProcessingOneScan = static_cast<double>(TimeAfterAlgorithm.toMicroseconds()-TimeBeforeAlgorithm.toMicroseconds())/1000000.0;
  //cout << "Speed for algorithm is: " << 1/timeSinceReceiveTheProcessingScan << " FPS" << endl;
  m_algorithmTime = timeForProcessingOneScan;
  cout << "Time for processing one scan of data is: " << timeForProcessingOneScan << "s" << endl;
  }
}

void Attention::setUp()
{
  auto kv = getKeyValueConfiguration();
  m_startAngle = kv.getValue<double>("logic-cfsd18-sensation-attention.startAngle");
  m_endAngle = kv.getValue<double>("logic-cfsd18-sensation-attention.endAngle");
  m_xBoundary = kv.getValue<double>("logic-cfsd18-sensation-attention.xBoundary");
  m_yBoundary = kv.getValue<double>("logic-cfsd18-sensation-attention.yBoundary");
  m_groundLayerZ = kv.getValue<double>("logic-cfsd18-sensation-attention.groundLayerZ");
  m_coneHeight = kv.getValue<double>("logic-cfsd18-sensation-attention.coneHeight");
  m_connectDistanceThreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.connectDistanceThreshold");
  m_layerRangeThreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.layerRangeThreshold");
  m_minNumOfPointsForCone = kv.getValue<uint16_t>("logic-cfsd18-sensation-attention.minNumOfPointsForCone");
  m_maxNumOfPointsForCone = kv.getValue<uint16_t>("logic-cfsd18-sensation-attention.maxNumOfPointsForCone");
  m_farConeRadiusThreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.farConeRadiusThreshold");
  m_nearConeRadiusThreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.nearConeRadiusThreshold");
  m_zRangeThreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.zRangeThreshold");
  m_inlierRangeThreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.inlierRangeTreshold");
  m_inlierFoundTreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.inlierFoundTreshold");
  m_ransacIterations = kv.getValue<double>("logic-cfsd18-sensation-attention.numberOfIterations");
  m_dotThreshold = kv.getValue<double>("logic-cfsd18-sensation-attention.dotThreshold");
  m_lastBestPlane = Eigen::MatrixXd::Zero(1,4);
  m_lastBestPlane << 0,0,1,0;

  //ConeDetection();

}

void Attention::tearDown()
{
}

void Attention::SaveOneCPCPointNoIntensity(const int &pointIndex,const uint16_t &distance_integer, const double &azimuth, const double &verticalAngle, const uint8_t &distanceEncoding)
{

  //Recordings before 2017 do not call hton() while storing CPC.
  //Hence, we only call ntoh() for recordings from 2017.
  uint16_t distanceCPCPoint = distance_integer;
  if (m_recordingYear > 2016) {
      distanceCPCPoint = ntohs(distanceCPCPoint);
  }
  double distance = 0.0;
  switch (distanceEncoding) {
      case CompactPointCloud::CM : distance = static_cast<double>(distanceCPCPoint / 100.0f); //convert to meter from resolution 1 cm
                                   break;
      case CompactPointCloud::MM : distance = static_cast<double>(distanceCPCPoint / 500.0f); //convert to meter from resolution 2 mm
                                   break;
      default : cout << "Error, distanceCPCPoint not correctly defined!" << endl;
                break;
  }

  // Compute x, y, z coordinate based on distance, azimuth, and vertical angle
  double xyDistance = distance * cos(verticalAngle * static_cast<double>(DEG2RAD));
  double xData = xyDistance * sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = xyDistance * cos(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(verticalAngle * static_cast<double>(DEG2RAD));
  m_pointCloud.row(pointIndex) << xData,yData,zData;
}

void Attention::SaveCPC32NoIntensity(const uint8_t &part, const uint8_t &entriesPerAzimuth, const double &startAzimuth, const double &endAzimuth, const uint8_t &distanceEncoding)
{
  double azimuth = startAzimuth;
  uint32_t numberOfPoints;
  stringstream sstr;
  if (part == 1) {
      numberOfPoints = m_12_cpcDistance_32.size() / 2;
      sstr.str(m_12_cpcDistance_32);
  } else if (part == 2) {
      numberOfPoints = m_11_cpcDistance_32.size() / 2;
      sstr.str(m_11_cpcDistance_32);
  } else {
      numberOfPoints = m_9_cpcDistance_32.size() / 2;
      sstr.str(m_9_cpcDistance_32);
  }
  uint32_t numberOfAzimuths = numberOfPoints / entriesPerAzimuth;
  double azimuthIncrement = (endAzimuth - startAzimuth) / numberOfAzimuths;//Calculate the azimuth increment
  uint16_t distance = 0;

  // Initialize m_pointCloud
  if (part == 1) {
      m_pointCloud = Eigen::MatrixXd::Zero(numberOfPoints,3);
      m_pointIndex = 0;
  } else {
      m_pointCloud.conservativeResizeLike(Eigen::MatrixXd::Zero(m_pointCloud.rows()+numberOfPoints, 3));

  }
  // Loop through all points and save each of them in the point cloud matrix
  for (uint32_t azimuthIndex = 0; azimuthIndex < numberOfAzimuths; azimuthIndex++) {
    for (uint8_t sensorIndex = 0; sensorIndex < entriesPerAzimuth; sensorIndex++) {
      sstr.read((char*)(&distance), 2); // Read distance value from the string in a CPC container point by point
      if (part == 1) {
          SaveOneCPCPointNoIntensity(m_pointIndex,distance, azimuth, m_12_verticalAngles[sensorIndex], distanceEncoding);
          m_pointIndex ++;
      } else if (part == 2) {
          SaveOneCPCPointNoIntensity(m_pointIndex,distance, azimuth, m_11_verticalAngles[sensorIndex], distanceEncoding);
          m_pointIndex ++;
      } else {
          SaveOneCPCPointNoIntensity(m_pointIndex,distance, azimuth, m_9_verticalAngles[sensorIndex], distanceEncoding);
          m_pointIndex ++;
      }
    }
    azimuth += azimuthIncrement;
  }

}


void Attention::SavePointCloud(){
  if (m_CPCReceived && !m_SPCReceived) {
    Lock lockCPC(m_cpcMutex);
    const double startAzimuth = m_cpc.getStartAzimuth();
    const double endAzimuth = m_cpc.getEndAzimuth();
    const uint8_t entriesPerAzimuth = m_cpc.getEntriesPerAzimuth(); // numberOfLayers
    const uint8_t numberOfBitsForIntensity = m_cpc.getNumberOfBitsForIntensity();
    const uint8_t intensityPlacement = m_cpc.getIntensityPlacement();
    uint16_t tmpMask = 0xFFFF;
    //double intensityMaxValue = 0.0f;

    if (numberOfBitsForIntensity > 0) {
      if (intensityPlacement == 0) {
        tmpMask = tmpMask >> numberOfBitsForIntensity; //higher bits for intensity
      } 
      else {
        tmpMask = tmpMask << numberOfBitsForIntensity; //lower bits for intensity
      }
      //intensityMaxValue = pow(2.0f, static_cast<double>(numberOfBitsForIntensity)) - 1.0f;
    }
    const uint8_t distanceEncoding = m_cpc.getDistanceEncoding();

    //#########################################
    // Need some function to rotate the model for ego vehicle state
    //#########################################

    uint16_t distance_integer = 0;
    if (entriesPerAzimuth == 16) {//A VLP-16 CPC
      double azimuth = startAzimuth;
      const string distances = m_cpc.getDistances();
      const uint32_t numberOfPoints = distances.size() / 2;
      const uint32_t numberOfAzimuths = numberOfPoints / entriesPerAzimuth;
      const double azimuthIncrement = (endAzimuth - startAzimuth) / numberOfAzimuths;//Calculate the azimuth increment
      stringstream sstr(distances);
      
      m_pointCloud = Eigen::MatrixXd::Zero(numberOfPoints,3);
      m_pointIndex = 0;
      for (uint32_t azimuthIndex = 0; azimuthIndex < numberOfAzimuths; azimuthIndex++) {
          double verticalAngle = START_V_ANGLE;
          for (uint8_t sensorIndex = 0; sensorIndex < entriesPerAzimuth; sensorIndex++) {
              sstr.read((char*)(&distance_integer), 2); // Read distance value from the string in a CPC container point by point
              if (numberOfBitsForIntensity == 0) {
                  SaveOneCPCPointNoIntensity(m_pointIndex,distance_integer, azimuth, verticalAngle, distanceEncoding);
                  m_pointIndex ++;
              } else {
                  //SaveOneCPCPointWithIntensity(distance_integer, azimuth, verticalAngle, distanceEncoding, numberOfBitsForIntensity, intensityPlacement, tmpMask, intensityMaxValue);
              }
              verticalAngle += V_INCREMENT;
          }
          azimuth += azimuthIncrement;
      }
      cout << "Angular resolution is: " << azimuthIncrement << endl;
      cout << "number of points are:"<< m_pointCloud.rows() << endl;
      //m_pointCloud = Eigen::MatrixXd::Zero(1,3); // Empty the point cloud matrix for this scan
      cout << "One scan complete! " << endl;
    }  else { //A HDL-32E CPC, one of the three parts of a complete scan  
      if ((m_cpcMask_32 & 0x04) > 0) {//The first part, 12 layers
        if (numberOfBitsForIntensity == 0) {
          cout << "The first part, 12 layers, no Intensity" <<  endl;
          SaveCPC32NoIntensity(1, entriesPerAzimuth, startAzimuth, endAzimuth, distanceEncoding);
        } 
        else {
          cout << "The first part, 12 layers, with Intensity" <<  endl;
          //drawCPC32withIntensity(1, numberOfLayers, startAzimuth, endAzimuth, distanceEncoding, numberOfBitsForIntensity, intensityPlacement, tmpMask, intensityMaxValue);
        }
      }
      if ((m_cpcMask_32 & 0x02) > 0) {//The second part, 11 layers
        if (numberOfBitsForIntensity == 0) {
          cout << "The second part, 11 layers, no Intensity" <<  endl;
          SaveCPC32NoIntensity(2, entriesPerAzimuth, startAzimuth, endAzimuth, distanceEncoding);
        } 
        else {
          cout << "The second part, 11 layers, with Intensity" <<  endl;
          //drawCPC32withIntensity(2, numberOfLayers, startAzimuth, endAzimuth, distanceEncoding, numberOfBitsForIntensity, intensityPlacement, tmpMask, intensityMaxValue);
        }
      }
      if ((m_cpcMask_32 & 0x01) > 0) {//The third part, 9 layers
        if (numberOfBitsForIntensity == 0) {
          cout << "The third part, 9 layers, no Intensity" << endl;
          SaveCPC32NoIntensity(3, entriesPerAzimuth, startAzimuth, endAzimuth, distanceEncoding);
        } 
        else {
          cout << "The third part, 9 layers, with Intensity" << endl;
          //drawCPC32withIntensity(3, numberOfLayers, startAzimuth, endAzimuth, distanceEncoding, numberOfBitsForIntensity, intensityPlacement, tmpMask, intensityMaxValue);
        }
      }
    } 
  }
}

void Attention::ConeDetection(){
  m_generatedTestPointCloud = GenerateTestPointCloud();

  Eigen::MatrixXd pointCloudConeROI = ExtractConeROI(m_xBoundary, m_yBoundary, m_groundLayerZ, m_coneHeight);

  cout << "RANSAC" << endl;
  Eigen::MatrixXd pcRefit = RANSACRemoveGround(pointCloudConeROI);
  
  double numOfPointsAfterRANSAC = pcRefit.rows();
  cout << "points After RANSAC" << endl;
  cout << numOfPointsAfterRANSAC << endl;
  /*
  cout << "points before RANSAC" << endl;
  double pcRaw = m_pointCloud.rows();
  cout << pcRaw << endl;*/
  if (numOfPointsAfterRANSAC <= 600.0)
  {
    vector<vector<uint32_t>> objectIndexList = NNSegmentation(pcRefit, m_connectDistanceThreshold); //out from ransac pointCloudConeROI to pointCloudFilt
    vector<vector<uint32_t>> coneIndexList = FindConesFromObjects(pcRefit, objectIndexList, m_minNumOfPointsForCone, m_maxNumOfPointsForCone, m_nearConeRadiusThreshold, m_farConeRadiusThreshold, m_zRangeThreshold);
    //cout << "Number of Object is: " << objectIndexList.size() << endl;
    cout << "Number of Cones is: " << coneIndexList.size() << endl;
    SendingConesPositions(pcRefit, coneIndexList);
    

  }
 
}

vector<vector<uint32_t>> Attention::NNSegmentation(Eigen::MatrixXd &pointCloudConeROI, const double &connectDistanceThreshold){
  uint32_t numberOfPointConeROI = pointCloudConeROI.rows();
  vector<uint32_t> restPointsList(numberOfPointConeROI);
  for (uint32_t i = 0; i < numberOfPointConeROI; i++)
  {
    restPointsList[i] = i;
  }
  vector<vector<uint32_t>> objectIndexList;
  vector<uint32_t> tmpObjectIndexList; tmpObjectIndexList.push_back(restPointsList[0]);
  uint32_t tmpPointIndex = restPointsList[0];
  uint32_t positionOfTmpPointIndexInList = 0;
  uint32_t tmpPointIndexNext;
  restPointsList.erase(restPointsList.begin());

  while (!restPointsList.empty())
  {
    uint32_t numberOfRestPoints = restPointsList.size();
    double minDistance = 100000; // assign a large value for inilization
    for (uint32_t i = 0; i < numberOfRestPoints; i++)
    {
      double distance = CalculateXYDistance(pointCloudConeROI, tmpPointIndex, restPointsList[i]);
      //cout << "Distance is " << distance << endl;
      if (distance < minDistance)
      {
        tmpPointIndexNext = restPointsList[i];
        positionOfTmpPointIndexInList = i;
        minDistance = distance;
      }
 
    }
    tmpPointIndex = tmpPointIndexNext;
    //cout << "Minimum Distance is " << minDistance << endl;
    // now we have minDistance and tmpPointIndex for next iteration
    if (minDistance <= connectDistanceThreshold)
    {
      tmpObjectIndexList.push_back(tmpPointIndex);
    } else {
      if (!tmpObjectIndexList.empty())
      {
        objectIndexList.push_back(tmpObjectIndexList);
      }
      tmpObjectIndexList.clear();
      tmpObjectIndexList.push_back(tmpPointIndex);
    }
    restPointsList.erase(restPointsList.begin()+positionOfTmpPointIndexInList);
  }
  if (!tmpObjectIndexList.empty())
  {
    objectIndexList.push_back(tmpObjectIndexList);
  }
  return objectIndexList;
}

vector<vector<uint32_t>> Attention::FindConesFromObjects(Eigen::MatrixXd &pointCloudConeROI, vector<vector<uint32_t>> &objectIndexList, const double &minNumOfPointsForCone, const double &maxNumOfPointsForCone, const double &nearConeRadiusThreshold, const double &farConeRadiusThreshold, const double &zRangeThreshold)
{
  uint32_t numberOfObjects = objectIndexList.size();

  // Select those objects with reasonable number of points and save the object list in a new vector
  vector<vector<uint32_t>> objectIndexListWithNumOfPointsLimit;
  for (uint32_t i = 0; i < numberOfObjects; i ++)
  {
    vector<uint32_t> objectIndex = objectIndexList[i];
    uint32_t numberOfPointsOnObject = objectIndex.size();

    bool numberOfPointsLimitation = ((numberOfPointsOnObject >= minNumOfPointsForCone) && (numberOfPointsOnObject <= maxNumOfPointsForCone));
    if (numberOfPointsLimitation)
    {
      objectIndexListWithNumOfPointsLimit.push_back(objectIndex);
    }
  }

  // Select among previous potention cones with reasonable radius
  vector<vector<uint32_t>> coneIndexList;
  for (uint32_t i = 0; i < objectIndexListWithNumOfPointsLimit.size(); i ++)
  {
    vector<uint32_t> selectedObjectIndex = objectIndexListWithNumOfPointsLimit[i];
    uint32_t numberOfPointsOnSelectedObject = selectedObjectIndex.size();
    Eigen::MatrixXd potentialConePointCloud = Eigen::MatrixXd::Zero(numberOfPointsOnSelectedObject,3);
    for (uint32_t j = 0; j < numberOfPointsOnSelectedObject; j++)
    {
      potentialConePointCloud.row(j) << pointCloudConeROI(selectedObjectIndex[j],0),pointCloudConeROI(selectedObjectIndex[j],1),pointCloudConeROI(selectedObjectIndex[j],2);
    }
    
    double coneRadius = CalculateConeRadius(potentialConePointCloud);
    double zRange = GetZRange(potentialConePointCloud);
    bool condition1 = (coneRadius < farConeRadiusThreshold); //Far point cones
    bool condition2 = (coneRadius>= farConeRadiusThreshold && coneRadius <= nearConeRadiusThreshold);
    bool condition3 = (zRange >= zRangeThreshold);  // Near point cones have to cover a larger Z range
    if (condition1 || (condition2 && condition3))
    {
      coneIndexList.push_back(selectedObjectIndex);
    }

  }

  return coneIndexList;

}

double Attention::CalculateConeRadius(Eigen::MatrixXd &potentialConePointCloud)
{
  double coneRadius = 0;
  uint32_t numberOfPointsOnSelectedObject = potentialConePointCloud.rows();
  double xMean = potentialConePointCloud.colwise().sum()[0]/numberOfPointsOnSelectedObject;
  double yMean = potentialConePointCloud.colwise().sum()[1]/numberOfPointsOnSelectedObject;
  for (uint32_t i = 0; i < numberOfPointsOnSelectedObject; i++)
  {
    double radius = sqrt(pow((potentialConePointCloud(i,0)-xMean),2)+pow((potentialConePointCloud(i,1)-yMean),2));
    if (radius >= coneRadius)
    {
      coneRadius = radius;
    }
  }
  return coneRadius;

}

double Attention::GetZRange(Eigen::MatrixXd &potentialConePointCloud)
{
  double zRange = potentialConePointCloud.colwise().maxCoeff()[2]-potentialConePointCloud.colwise().minCoeff()[2];
  return zRange;
}


Eigen::MatrixXd Attention::ExtractConeROI(const double &xBoundary, const double &yBoundary, const double &groundLayerZ,  const double &coneHeight){
  uint32_t numberOfPointsCPC = m_pointCloud.rows();
  uint32_t numberOfPointConeROI = 0;
  vector<int> pointIndexConeROI;
  for (uint32_t i = 0; i < numberOfPointsCPC; i++)
  {
    if ((m_pointCloud(i,1) >= -xBoundary) && (m_pointCloud(i,1) <= xBoundary) && (m_pointCloud(i,2) <= yBoundary) && (m_pointCloud(i,2) <= groundLayerZ + coneHeight))
    {
      pointIndexConeROI.push_back(i);
      numberOfPointConeROI ++;
    }
  }
  Eigen::MatrixXd pointCloudConeROI = Eigen::MatrixXd::Zero(numberOfPointConeROI,3);
  for (uint32_t j = 0; j < numberOfPointConeROI; j++)
  {
    pointCloudConeROI.row(j) << m_pointCloud(pointIndexConeROI[j],0),m_pointCloud(pointIndexConeROI[j],1),m_pointCloud(pointIndexConeROI[j],2);
  }

  return pointCloudConeROI;
}

double Attention::CalculateXYDistance(Eigen::MatrixXd &pointCloud, const uint32_t &index1, const uint32_t &index2)
{
  double x1 = pointCloud(index1,0);
  double y1 = pointCloud(index1,1);
  double x2 = pointCloud(index2,0);
  double y2 = pointCloud(index2,1);
  double distance = sqrt(pow((x1-x2),2) + pow((y1-y2),2));
  return distance;
}

void Attention::SendingConesPositions(Eigen::MatrixXd &pointCloudConeROI, vector<vector<uint32_t>> &coneIndexList)
{
  
  uint32_t numberOfCones = coneIndexList.size();
  Eigen::MatrixXd conePoints = Eigen::MatrixXd::Zero(numberOfCones,3);
  for (uint32_t i = 0; i < numberOfCones; i++)
  {
    uint32_t numberOfPointsOnCone = coneIndexList[i].size();
    double conePositionX = 0;
    double conePositionY = 0;
    double conePositionZ = 0;
    for (uint32_t j = 0; j< numberOfPointsOnCone; j++)
    {
      conePositionX += pointCloudConeROI(coneIndexList[i][j],0);
      conePositionY += pointCloudConeROI(coneIndexList[i][j],1);
      conePositionZ += pointCloudConeROI(coneIndexList[i][j],2);
    }
    conePositionX = conePositionX / numberOfPointsOnCone;
    conePositionY = conePositionY / numberOfPointsOnCone;
    conePositionZ = conePositionZ / numberOfPointsOnCone;
    conePoints.row(i) << conePositionX,conePositionY,conePositionZ;

    opendlv::logic::sensation::Point conePoint = Cartesian2Spherical(conePositionX,conePositionY,conePositionZ);
    odcore::data::Container c1(conePoint);
    getConference().send(c1);
    //cout << "a point sent out with distance: " <<conePoint.getDistance() <<"; azimuthAngle: " << conePoint.getAzimuthAngle() << "; and zenithAngle: " << conePoint.getZenithAngle() << endl;
  }

  cout << "Detected Cones are: " << conePoints << endl;

  
}

opendlv::logic::sensation::Point Attention::Cartesian2Spherical(double &x, double &y, double &z)
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

Eigen::MatrixXd Attention::RANSACRemoveGround(Eigen::MatrixXd pointCloudInRANSAC)
{

  //Get from configuration
  cout << "RANSAC Start" << endl;
  //Inlier dearch threshold
  //double m_inlierSearchThreshold = 0.05;
  //Best plane inlier removal threshold
  //double inlierRemovalThreshold = 0.03;
  //double inlierFoundTreshold = 5000;
  //k amount of iterations for RANSAC
  //int k = 100;
  //Reference vector

  //cout << "Test 1" << endl;
  Eigen::MatrixXd foundPlane(1,4), planeBest(1,4), planeBestBest(1,4), normal(1,3), pointOnPlane(1,3), indexRangeBest,indexDotter ,indexOutliers(pointCloudInRANSAC.rows(),1);
  //Eigen::MatrixXd planeBest;
  //Eigen::MatrixXd planeBestBest;
  foundPlane << 0,0,0,0;
  normal << 0,0,1;
  double d;
  double indexDotFound = 0;
  int planeCounter = 0;
  
  //cout << "Test 2" << endl;
  //double inlierBest = 0;
  int M = 1;
  int sizeCloud = pointCloudInRANSAC.rows()-1;
  Eigen::MatrixXd drawnSamples = Eigen::MatrixXd::Zero(3,3);
  Eigen::MatrixXd distance2Plane = Eigen::MatrixXd::Zero(pointCloudInRANSAC.rows(),1);
  Eigen::MatrixXd dotProd = Eigen::MatrixXd::Zero(pointCloudInRANSAC.rows(),1);
  Eigen::MatrixXd indexDot = Eigen::MatrixXd::Zero(pointCloudInRANSAC.rows(),1);
  //cout << "Test 3" << endl;
  
  Vector3d planeFromSamples, v0, v1, v2, crossVec1, crossVec2, crossCoefficients;
  double outliersFound, inliersFound, normalBest, normalBestLast;
  normalBestLast = 10000;
  //cout << "Test 4" << endl;
  /*Vector3d v0;
  Vector3d v1;
  Vector3d v2;
  Vector3d crossVec1;
  Vector3d crossVec2;
  Vector3d crossCoefficients;*/

  for(int i = 0; i < m_ransacIterations; i++)
  {
    outliersFound = 0;
    inliersFound = 0;
    indexOutliers = Eigen::MatrixXd::Zero(pointCloudInRANSAC.rows(),1);
    //cout << "Test 5" << endl;
    //Draw 3 random samples from pointCloud

    for(int j = 0; j < 3; j++){

      int indexShuffle = M + rand() / (RAND_MAX / (sizeCloud - M + 1) + 1);

      drawnSamples.row(j) = pointCloudInRANSAC.row(indexShuffle);
      //cout << indexShuffle << endl;

    }
    //cout << "Drawn Samples" << endl;
    //cout << drawnSamples << endl;
    //cout << "Test 6" << endl;
    //Calculate Plane coefficients [a b c]
    v0 << drawnSamples(0,0), drawnSamples(0,1), drawnSamples(0,2);
    v1 << drawnSamples(1,0), drawnSamples(1,1), drawnSamples(1,2);
    v2 << drawnSamples(2,0), drawnSamples(2,1), drawnSamples(2,2);
    //cout << "Test 6.1" << endl;
    crossVec1 = v0-v1;
    crossVec2 = v0-v2;
    //cout << "Test 6.2" << endl;
    crossCoefficients = crossVec2.cross(crossVec1);
    //cout << "Test 6.3" << endl;
    //cout << crossCoefficients << endl;
    //cout << "Test 6.5" << endl;
    crossCoefficients = crossCoefficients.normalized();
    //cout << "Test 6.4" << endl;
    //d = crossCoefficients(0)*v0(0) + crossCoefficients(1)*v0(1) + crossCoefficients(2)*v0(2);
    d = v0.dot(crossCoefficients);
    d = d*-1;
  
    foundPlane << crossCoefficients(0), crossCoefficients(1), crossCoefficients(2), d;
    //cout << "new it" << endl;
    //cout << v0(0) << " " << v0(1) << " " << v0(2) << " " << endl;
    //cout << crossCoefficients(0) << " " << crossCoefficients(1) << " " << crossCoefficients(2) << " " << " " <<  d << endl;

    //cout << "Test 6.6" << endl;
    //cout << "Test 7" << endl;

    //Calculate perpendicular distance to found plane
    for(int p = 0; p < pointCloudInRANSAC.rows(); p++){
      //cout << "Test 7.1" << endl;
      distance2Plane(p,0) = abs(foundPlane(0,0)*pointCloudInRANSAC(p,0) + foundPlane(0,1)*pointCloudInRANSAC(p,1) + foundPlane(0,2)*pointCloudInRANSAC(p,2) + foundPlane(0,3))/crossCoefficients.norm();
      //Find index of inliers
      //cout << "Test 7.2" << endl;
      if(distance2Plane(p,0) >= m_inlierRangeThreshold){
        indexOutliers(outliersFound,0) = p;
        outliersFound++;

      }
      //cout << "Test 7.3" << endl;
    } 
    //cout << "NUmber of outliers are: " << outliersFound << endl;
    //cout << "Test 8" << endl;
    if(outliersFound > 0){
      Eigen::MatrixXd indexRange = Eigen::MatrixXd::Zero(outliersFound,1);
      indexRange = indexOutliers.topRows(outliersFound+1);

      inliersFound = pointCloudInRANSAC.rows()-outliersFound;
      //cout << "Test 9" << endl;
      //cout << inliersFound << endl;
      if(inliersFound > m_inlierFoundTreshold){
  
        planeBest = foundPlane;
        //cout << "Test 10" << endl;
        normalBest = sqrt(pow((planeBest(0,0)-normal(0,0)),2) + pow((planeBest(0,1)-normal(0,1)),2) + pow((planeBest(0,2)-normal(0,2)),2));

        if(normalBest < normalBestLast){

          normalBestLast = normalBest;
          planeBestBest = planeBest;
          //cout << "Test 11" << endl;
          indexRangeBest.resize(indexRange.rows(),indexRange.cols());
          indexRangeBest = indexRange;
          //cout << "Test 12" << endl;
          pointOnPlane = drawnSamples.row(0);
          planeCounter++;
        

        }
        //cout << "Test 13" << endl;
      }
    }
    else{

      cout << "No plane was found" << endl;
      if(planeCounter > 0){

        m_lastBestPlane = planeBestBest;
      }
    }
    //cout << "NUmber of iterations is: " << i << endl;
  }

  if(planeCounter == 0){

    planeBestBest = m_lastBestPlane;
  }
  //cout << "Test 14" << endl;
  for(int p = 0; p < pointCloudInRANSAC.rows(); p++){

    dotProd(p,0) = planeBestBest(0,0)*(pointCloudInRANSAC(p,0)-pointOnPlane(0,0)) + planeBestBest(0,1)*(pointCloudInRANSAC(p,1)-pointOnPlane(0,1)) + planeBestBest(0,2)*(pointCloudInRANSAC(p,2)-pointOnPlane(0,2)); 

    if(dotProd(p,0) > m_dotThreshold){

      indexDot(indexDotFound,0) = p;
      indexDotFound++;
    }
  }
  //cout << "Test 11" << endl;
  indexDotter.resize(indexDot.rows(),indexDot.cols());
  indexDotter = indexDot.topRows(indexDotFound+1);

  //cout << "indexDotter is: " << indexDotter << " and indexRangeBest is : " << indexRangeBest << endl;

  //cout << "Test 12" << endl;
  Eigen::MatrixXd index2Keep(indexDotter.rows()+indexRangeBest.rows(),1);

  index2Keep << indexRangeBest,
                 indexDotter;

  
  //Remove duplicates
  //cout << "Test 13" << endl;
  Eigen::MatrixXd sortedIndex = RemoveDuplicates(index2Keep);
  //Remove found inlier index from

  //cout << "Test 14" << endl;
  Eigen::MatrixXd pcRefit = Eigen::MatrixXd::Zero(sortedIndex.rows(),3);
  for(int i = 0; i < sortedIndex.rows(); i++){

    pcRefit.row(i) = pointCloudInRANSAC.row(sortedIndex(i));


  }
  cout << "Ground plane found:" << endl;
  cout << planeBestBest << endl;

  return pcRefit;

}

Eigen::MatrixXd Attention::RemoveDuplicates(Eigen::MatrixXd needSorting)
{

  vector<double> vect;

  for(int i=0; i< needSorting.rows(); i++){

    vect.push_back(needSorting(i,0));

  }

  sort(vect.begin(),vect.end());
  vect.erase(unique(vect.begin(),vect.end()),vect.end());
  needSorting.resize(vect.size(),1);

  for(unsigned int i=0; i< vect.size(); i++){

    needSorting(i,0)=vect.at(i);

  }

  return needSorting;

}

Eigen::MatrixXd Attention::GenerateTestPointCloud()
{
  Eigen::MatrixXd generatedPointCloud = Eigen::MatrixXd::Zero(22000,3);

  for(int i = 0; i < generatedPointCloud.rows(); i++){


    if(i < 21000){

    double f = (double)rand() / RAND_MAX;
    double r1 = 5 + f*(5 - 0);

    double ff = (double)rand() / RAND_MAX;
    double r2 = 5 + ff*(5 - 0);
    
    double fff = (double)rand() / RAND_MAX;
    double r3 =  -0.20 + fff*(-0.15 + 0.20);

    generatedPointCloud.row(i) << r1,r2,r3;
    }
    if(i >= 18000 && i < 21000){

      double f = (double)rand() / RAND_MAX;
      double r1 = 40 + f*(40.2 - 40);

      double ff = (double)rand() / RAND_MAX;
      double r2 = 0.15 + ff*(1 - 0.15);
      
      double fff = (double)rand() / RAND_MAX;
      double r3 =  0.15 + fff*(1 - 0.15);

      generatedPointCloud.row(i) << r1,r2,r3;

    }

    if(i >= 21000 && i < 21050){

      double f = (double)rand() / RAND_MAX;
      double r1 = 3 + f*(3.2 - 3);

      double ff = (double)rand() / RAND_MAX;
      double r2 = 3 + ff*(3.2 - 3);
      
      double fff = (double)rand() / RAND_MAX;
      double r3 =  -0.15 + fff*(0.1 + 0.15);

      generatedPointCloud.row(i) << r1,r2,r3;

    }

    if(i >= 21050 && i < 21100){

      double f = (double)rand() / RAND_MAX;
      double r1 = 0 + f*(0.2 - 0);

      double ff = (double)rand() / RAND_MAX;
      double r2 = 3 + ff*(3.2 - 3);
      
      double fff = (double)rand() / RAND_MAX;
      double r3 =  -0.15 + fff*(0.1 + 0.15);

      generatedPointCloud.row(i) << r1,r2,r3;

    }
    if(i >= 21100 && i < 21130){

       double f = (double)rand() / RAND_MAX;
      double r1 = 3 + f*(3.2 - 3);

      double ff = (double)rand() / RAND_MAX;
      double r2 = 4 + ff*(4.2 - 4);
      
      double fff = (double)rand() / RAND_MAX;
      double r3 =  -0.15 + fff*(0.1 + 0.15);
      generatedPointCloud.row(i) << r1,r2,r3;
    }

      if(i >= 21130 && i < 21160){

      double f = (double)rand() / RAND_MAX;
      double r1 = 0 + f*(0.2 - 0);

      double ff = (double)rand() / RAND_MAX;
      double r2 = 4 + ff*(4.2 - 4);
      
      double fff = (double)rand() / RAND_MAX;
      double r3 =  -0.15 + fff*(0.1 + 0.15);

      generatedPointCloud.row(i) << r1,r2,r3;

    }

    if(i >= 21160 ){

      double f = (double)rand() / RAND_MAX;
      double r1 = 0 + f*(5 - 0);

      double ff = (double)rand() / RAND_MAX;
      double r2 = 0 + ff*(5 - 0);
      
      double fff = (double)rand() / RAND_MAX;
      double r3 =  0.15 + fff*(5 - 0.15);

      generatedPointCloud.row(i) << r1,r2,r3;

    }
    
      
  }

  return generatedPointCloud;
}


}
}
}
}
