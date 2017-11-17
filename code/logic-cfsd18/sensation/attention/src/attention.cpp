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
  //, pointCloud()
  , m_pointIndex(0)
  , m_startAngle()
  , m_endAngle()
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
  {
  //#############################################################################
  // Following part are prepared to decode CPC of HDL32 3 parts
  //#############################################################################
  std::array<float, 32>  sensorIDs_32;
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
  if(a_container.getDataType() == odcore::data::SharedPointCloud::ID()){
    m_SPCReceived = true;
    cout << "Error: Don't use SharedPointCloud here!!!" << endl;
}
  if (a_container.getDataType() == odcore::data::CompactPointCloud::ID()) {
    m_CPCReceived = true;
    TimeStamp ts = a_container.getSampleTimeStamp();
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

  SavePointCloud();
  //ConeDetection();
      /*  
      opendlv::logic::sensation::Attention o1;
      odcore::data::Container c1(o1);
      getConference().send(c1);
    */
}

void Attention::setUp()
{
  auto kv = getKeyValueConfiguration();
  m_startAngle = kv.getValue<float>("logic-cfsd18-sensation-attention.startAngle");
  m_endAngle = kv.getValue<float>("logic-cfsd18-sensation-attention.endAngle");
  m_yBoundary = kv.getValue<float>("logic-cfsd18-sensation-attention.yBoundary");
  m_groundLayerZ = kv.getValue<float>("logic-cfsd18-sensation-attention.groundLayerZ");
  m_coneHeight = kv.getValue<float>("logic-cfsd18-sensation-attention.coneHeight");
  m_connectDistanceThreshold = kv.getValue<float>("logic-cfsd18-sensation-attention.connectDistanceThreshold");
  m_layerRangeThreshold = kv.getValue<float>("logic-cfsd18-sensation-attention.layerRangeThreshold");
  m_minNumOfPointsForCone = kv.getValue<uint16_t>("logic-cfsd18-sensation-attention.minNumOfPointsForCone");
  m_maxNumOfPointsForCone = kv.getValue<uint16_t>("logic-cfsd18-sensation-attention.maxNumOfPointsForCone");
  m_farConeRadiusThreshold = kv.getValue<float>("logic-cfsd18-sensation-attention.farConeRadiusThreshold");
  m_nearConeRadiusThreshold = kv.getValue<float>("logic-cfsd18-sensation-attention.nearConeRadiusThreshold");
  m_zRangeThreshold = kv.getValue<float>("logic-cfsd18-sensation-attention.zRangeThreshold");
  ConeDetection();
}

void Attention::tearDown()
{
}

void Attention::SaveOneCPCPointNoIntensity(const int &pointIndex,const uint16_t &distance_integer, const float &azimuth, const float &verticalAngle, const uint8_t &distanceEncoding)
{

  //Recordings before 2017 do not call hton() while storing CPC.
  //Hence, we only call ntoh() for recordings from 2017.
  uint16_t distanceCPCPoint = distance_integer;
  if (m_recordingYear > 2016) {
      distanceCPCPoint = ntohs(distanceCPCPoint);
  }
  float distance = 0.0;
  switch (distanceEncoding) {
      case CompactPointCloud::CM : distance = static_cast<float>(distanceCPCPoint / 100.0f); //convert to meter from resolution 1 cm
                                   break;
      case CompactPointCloud::MM : distance = static_cast<float>(distanceCPCPoint / 500.0f); //convert to meter from resolution 2 mm
                                   break;
      default : cout << "Error, distanceCPCPoint not correctly defined!" << endl;
                break;
  }

  // Compute x, y, z coordinate based on distance, azimuth, and vertical angle
  float xyDistance = distance * cos(verticalAngle * static_cast<float>(DEG2RAD));
  float xData = xyDistance * sin(azimuth * static_cast<float>(DEG2RAD));
  float yData = xyDistance * cos(azimuth * static_cast<float>(DEG2RAD));
  float zData = distance * sin(verticalAngle * static_cast<float>(DEG2RAD));
  m_pointCloud.row(pointIndex) << xData,yData,zData;
}

void Attention::SaveCPC32NoIntensity(const uint8_t &part, const uint8_t &entriesPerAzimuth, const float &startAzimuth, const float &endAzimuth, const uint8_t &distanceEncoding)
{
  float azimuth = startAzimuth;
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
  float azimuthIncrement = (endAzimuth - startAzimuth) / numberOfAzimuths;//Calculate the azimuth increment
  uint16_t distance = 0;

  // Initialize m_pointCloud
  if (part == 1) {
      m_pointCloud = MatrixXf::Zero(numberOfPoints,3);
      m_pointIndex = 0;
  } else {
      m_pointCloud.conservativeResizeLike(MatrixXf::Zero(m_pointCloud.rows()+numberOfPoints, 3));

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
//void SaveCPC32WithIntensity(const uint8_t &part, const uint8_t &entriesPerAzimuth, const float &startAzimuth, const float &endAzimuth, const uint8_t &distanceEncoding, const uint8_t &numberOfBitsForIntensity, const uint8_t &intensityPlacement, const uint16_t &mask, const float &intensityMaxValue)
//{
  //opendlv::coord::Position pointPosition;
  //return pointPosition;
//}

void Attention::SavePointCloud(){
  if (m_CPCReceived && !m_SPCReceived) {
    Lock lockCPC(m_cpcMutex);
    const float startAzimuth = m_cpc.getStartAzimuth();
    const float endAzimuth = m_cpc.getEndAzimuth();
    const uint8_t entriesPerAzimuth = m_cpc.getEntriesPerAzimuth(); // numberOfLayers
    const uint8_t numberOfBitsForIntensity = m_cpc.getNumberOfBitsForIntensity();
    const uint8_t intensityPlacement = m_cpc.getIntensityPlacement();
    uint16_t tmpMask = 0xFFFF;
    //float intensityMaxValue = 0.0f;

    if (numberOfBitsForIntensity > 0) {
      if (intensityPlacement == 0) {
        tmpMask = tmpMask >> numberOfBitsForIntensity; //higher bits for intensity
      } 
      else {
        tmpMask = tmpMask << numberOfBitsForIntensity; //lower bits for intensity
      }
      //intensityMaxValue = pow(2.0f, static_cast<float>(numberOfBitsForIntensity)) - 1.0f;
    }
    const uint8_t distanceEncoding = m_cpc.getDistanceEncoding();

    //#########################################
    // Need some function to rotate the model for ego vehicle state
    //#########################################

    uint16_t distance_integer = 0;
    if (entriesPerAzimuth == 16) {//A VLP-16 CPC
      float azimuth = startAzimuth;
      const string distances = m_cpc.getDistances();
      const uint32_t numberOfPoints = distances.size() / 2;
      const uint32_t numberOfAzimuths = numberOfPoints / entriesPerAzimuth;
      const float azimuthIncrement = (endAzimuth - startAzimuth) / numberOfAzimuths;//Calculate the azimuth increment
      stringstream sstr(distances);
      
      m_pointCloud = MatrixXf::Zero(numberOfPoints,3);
      m_pointIndex = 0;
      for (uint32_t azimuthIndex = 0; azimuthIndex < numberOfAzimuths; azimuthIndex++) {
          float verticalAngle = START_V_ANGLE;
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
      cout << "number of points are:"<< m_pointCloud.rows() << endl;
      //m_pointCloud = MatrixXf::Zero(1,3); // Empty the point cloud matrix for this scan
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
  m_pointCloud = MatrixXf::Zero(5,3);
  m_pointCloud << 1.0,2.0,0.2,4.0,2.0,0.2,7.0,8.0,9.0,10.0,11.0,12.0,1.3,2.0,0.0;
  cout << "original point cloud is " << m_pointCloud << endl;
  
  float groundLayerZ = 0.0;
  float layerRangeThreshold = 0.1;
  float coneHeight = 3.0;

  MatrixXf pointCloudConeROI = ExtractConeROI(groundLayerZ, layerRangeThreshold, coneHeight);

  cout << "Cone ROI is: " << pointCloudConeROI << endl;
  cout << "Distance should be 3 and it's calculated as: " << CalculateXYDistance(pointCloudConeROI,0,1) << endl;
  vector<vector<uint32_t>> objectIndexList = NNSegmentation(pointCloudConeROI, m_connectDistanceThreshold);
  vector<vector<uint32_t>> coneIndexList = FindConesFromObjects(pointCloudConeROI, objectIndexList, m_minNumOfPointsForCone, m_maxNumOfPointsForCone, m_nearConeRadiusThreshold, m_farConeRadiusThreshold, m_zRangeThreshold);
  cout << "Number of Object is: " << objectIndexList.size() << endl;
  cout << "Number of Cones is" << coneIndexList.size() << endl;
  
 
}

vector<vector<uint32_t>> Attention::NNSegmentation(MatrixXf &pointCloudConeROI, const float &connectDistanceThreshold){
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
  restPointsList.erase(restPointsList.begin());

  while (!restPointsList.empty())
  {
    uint32_t numberOfRestPoints = restPointsList.size();
    float minDistance = 100000; // assign a large value for inilization
    for (uint32_t i = 0; i < numberOfRestPoints; i++)
    {
      float distance = CalculateXYDistance(pointCloudConeROI, tmpPointIndex, restPointsList[i]);
      cout << "Distance is " << distance << endl;
      if (distance < minDistance)
      {
        tmpPointIndex = restPointsList[i];
        positionOfTmpPointIndexInList = i;
        minDistance = distance;
      }
 
    }
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

vector<vector<uint32_t>> Attention::FindConesFromObjects(MatrixXf &pointCloudConeROI, vector<vector<uint32_t>> &objectIndexList, const float &minNumOfPointsForCone, const float &maxNumOfPointsForCone, const float &nearConeRadiusThreshold, const float &farConeRadiusThreshold, const float &zRangeThreshold)
{
  uint32_t numberOfObjects = objectIndexList.size();

  // Select those objects with reasonable number of points and save the object list in a new vector
  vector<vector<uint32_t>> objectIndexListWithNumOfPointsLimit;
  for (uint32_t i = 0; i < numberOfObjects; i ++)
  {
    vector<uint32_t> objectIndex = objectIndexList[i];
    uint32_t numberOfPointsOnObject = objectIndex.size();

    bool numberOfPointsLimitation = ((numberOfPointsOnObject > minNumOfPointsForCone) && (numberOfPointsOnObject < maxNumOfPointsForCone));
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
    MatrixXf potentialConePointCloud = MatrixXf::Zero(numberOfPointsOnSelectedObject,3);
    for (uint32_t j = 0; j < numberOfPointsOnSelectedObject; j++)
    {
      potentialConePointCloud.row(j) << pointCloudConeROI(selectedObjectIndex[j],0),pointCloudConeROI(selectedObjectIndex[j],1),pointCloudConeROI(selectedObjectIndex[j],2);
    }
    
    float coneRadius = CalculateConeRadius(potentialConePointCloud);
    float zRange = GetZRange(potentialConePointCloud);
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

float Attention::CalculateConeRadius(MatrixXf &potentialConePointCloud)
{
  float coneRadius = 0;
  uint32_t numberOfPointsOnSelectedObject = potentialConePointCloud.rows();
  float xMean = potentialConePointCloud.colwise().sum()[0]/numberOfPointsOnSelectedObject;
  float yMean = potentialConePointCloud.colwise().sum()[1]/numberOfPointsOnSelectedObject;
  for (uint32_t i = 0; i < numberOfPointsOnSelectedObject; i++)
  {
    float radius = sqrt(pow((potentialConePointCloud(i,0)-xMean),2)+pow((potentialConePointCloud(i,1)-yMean),2));
    if (radius >= coneRadius)
    {
      coneRadius = radius;
    }
  }
  return coneRadius;

}

float Attention::GetZRange(MatrixXf &potentialConePointCloud)
{
  float zRange = potentialConePointCloud.colwise().maxCoeff()[2]-potentialConePointCloud.colwise().minCoeff()[2];
  return zRange;
}


MatrixXf Attention::ExtractConeROI(const float &groundLayerZ, const float &layerRangeThreshold, const float &coneHeight){
  uint32_t numberOfPointsCPC = m_pointCloud.rows();
  uint32_t numberOfPointConeROI = 0;
  vector<int> pointIndexConeROI;
  for (uint32_t i = 0; i < numberOfPointsCPC; i++)
  {
    if ((m_pointCloud(i,2) >= groundLayerZ + layerRangeThreshold) && (m_pointCloud(i,2) <= groundLayerZ + coneHeight + layerRangeThreshold))
    {
      pointIndexConeROI.push_back(i);
      numberOfPointConeROI ++;
    }
  }
  MatrixXf pointCloudConeROI = MatrixXf::Zero(numberOfPointConeROI,3);
  for (uint32_t j = 0; j < numberOfPointConeROI; j++)
  {
    pointCloudConeROI.row(j) << m_pointCloud(pointIndexConeROI[j],0),m_pointCloud(pointIndexConeROI[j],1),m_pointCloud(pointIndexConeROI[j],2);
  }
  return pointCloudConeROI;
}

float Attention::CalculateXYDistance(MatrixXf &pointCloud, const uint32_t &index1, const uint32_t &index2)
{
  float x1 = pointCloud(index1,0);
  float y1 = pointCloud(index1,1);
  float x2 = pointCloud(index2,0);
  float y2 = pointCloud(index2,1);
  float distance = sqrt(pow((x1-x2),2) + pow((y1-y2),2));
  return distance;
}

}
}
}
}
