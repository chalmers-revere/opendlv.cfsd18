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

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include <thread>
#include "track.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace cognition {

Track::Track(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-cognition-track"),
  m_groundSpeed{0.0f},
  m_newFrame{true},
  m_objectId{1},
  m_groundSpeedMutex{},
  m_surfaceMutex{},
  m_surfaceFrame{},
  m_surfaceFrameBuffer{},
  m_nSurfacesInframe{},
  m_surfaceId{},
  m_timeReceived{},
  m_lastObjectId{},
  m_newId{true}
{

}

Track::~Track()
{
}

void Track::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    odcore::base::Lock lockGroundSpeed(m_groundSpeedMutex);
    auto groundSpeed = a_container.getData<opendlv::proxy::GroundSpeedReading>();
    m_groundSpeed = groundSpeed.getGroundSpeed();

  }

  if(a_container.getDataType() == opendlv::logic::perception::GroundSurfaceProperty::ID()){
      //std::cout << "TRACK RECIEVED A SURFACE!" << std::endl;
  //    m_lastTimeStamp = a_container.getSampleTimeStamp();
      auto surfaceProperty = a_container.getData<opendlv::logic::perception::GroundSurfaceProperty>();
      int surfaceId = surfaceProperty.getSurfaceId();
      auto nSurfacesInframe = surfaceProperty.getProperty();

      if (m_newFrame) { // If true, a frame has just been sent
        m_newFrame = false;
        m_nSurfacesInframe = std::stoul(nSurfacesInframe);
        m_surfaceId = surfaceId; // Currently not used.
        //std::cout << "SurfaceId: " << surfaceId<<std::endl;
        //std::cout << "nSurfacesInframe: " << nSurfacesInframe<<std::endl;
      }

  }

  if (a_container.getDataType() == opendlv::logic::perception::GroundSurfaceArea::ID()) {
    int objectId;
    {
    odcore::base::Lock lockSurface(m_surfaceMutex);
      auto groundSurfaceArea = a_container.getData<opendlv::logic::perception::GroundSurfaceArea>();
      objectId = groundSurfaceArea.getSurfaceId();
      odcore::data::TimeStamp containerStamp = a_container.getReceivedTimeStamp();
      double timeStamp = containerStamp.toMicroseconds(); // Save timeStamp for sorting purposes;
      if (m_newId) { // TODO: does it need to be global? can it be initialized in another way?
        m_objectId = (objectId!=m_lastObjectId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
        //std::cout << "newId, m_objectId: " <<m_objectId <<std::endl;
        m_newId=(m_objectId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
      }
      //std::cout << "objectId: " <<objectId <<std::endl;
      //std::cout << "m_objectId: " <<m_objectId <<std::endl;
      //std::cout << "m_lastObjectId: " <<m_lastObjectId <<std::endl;
      float x1 = groundSurfaceArea.getX1(); //Unpack message
      float y1 = groundSurfaceArea.getY1();
      float x2 = groundSurfaceArea.getX2();
      float y2 = groundSurfaceArea.getY2();
      float x3 = groundSurfaceArea.getX3();
      float y3 = groundSurfaceArea.getY3();
      float x4 = groundSurfaceArea.getX4();
      float y4 = groundSurfaceArea.getY4();
      std::vector<float> v(4);
      v[0] = (x1+x2)/2.0f;
      v[1] = (y1+y2)/2.0f;
      v[2] = (x3+x4)/2.0f;
      v[3] = (y3+y4)/2.0f;

      if (objectId == m_objectId) {
        //std::cout << "objectId in frame: " <<objectId <<std::endl;
        m_surfaceFrame[timeStamp] = v;
        /*std::cout << "Surfaces in frame: " <<m_surfaceFrame.size() <<std::endl;
        for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
          v = it->second;
          for (size_t i = 0; i < 4; i++) {
            std::cout<<v[i]<<"\n";
          }
        }*/
        m_timeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
      } else if (objectId != m_lastObjectId){ // If message doesn't belong to current or previous frame.
        //std::cout << "objectId in buffer: " <<objectId <<std::endl;
        m_surfaceFrameBuffer[timeStamp] = v; // Place message content coordinates in buffer
        //std::cout << "Surfaces in buffer: " <<m_surfaceFrameBuffer.size() <<std::endl;
        /*for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
          v = it->second;
          for (size_t i = 0; i < 4; i++) {
            std::cout<<v[i]<<"\n";
          }
        }*/
      }
    }
    auto wait = std::chrono::system_clock::now(); // Time point now
    std::chrono::duration<double> dur = wait-m_timeReceived; // Duration since last message recieved to m_surfaceFrame
    double duration = (m_objectId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame
    double receiveTimeLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.receiveTimeLimit");

    if (duration>receiveTimeLimit) { //Only for debug
      std::cout<<"DURATION TIME TRACK EXCEEDED: "<<duration<<std::endl;
      std::cout << "Surfaces to run: " <<m_surfaceFrame.size() <<std::endl;
      /*std::vector<float> v(4);
      for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
        v = it->second;
        for (size_t i = 0; i < 4; i++) {
          std::cout<<v[i]<<"\n";
        }
      }*/
    }

    // Run if frame is full or if we have waited to long for the remaining messages
    if ((m_surfaceFrame.size()==m_nSurfacesInframe || duration>receiveTimeLimit)) { //!m_newFrame && objectId==m_surfaceId &&
      //std::cout<<"Run condition OK "<<"\n";
      //std::cout << "duration: " <<duration <<std::endl;
      std::map< double, std::vector<float> > surfaceFrame;
      {
      odcore::base::Lock lockSurface(m_surfaceMutex);
        m_newFrame = true; // Allow for new surface property
        surfaceFrame = m_surfaceFrame; // Copy frame
        m_surfaceFrame = m_surfaceFrameBuffer; // Add buffer to new frame
        m_surfaceFrameBuffer.clear(); // Clear buffer
        m_lastObjectId = m_objectId; // Update last object id to ignore late messages
        m_newId = true; // Allow for new messages to be placed in m_surfaceFrame
        //std::cout << "Cleared buffer " <<std::endl;
      }
      //std::cout << "Run " << surfaceFrame.size() << " surfaces"<< std::endl;
      std::thread surfaceCollector(&Track::collectAndRun, this, surfaceFrame); // Run the surface in a thread
      surfaceCollector.detach();
    }
  }
}

  // TODO: logic for different states, start and stop
  /*
  if (a_container.getDataType() == opendlv::system::SignalStatusMessage::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::system::SystemOperationState::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::system::NetworkStatusMessage::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  */



void Track::setUp()
{
}

void Track::tearDown()
{
}

void Track::collectAndRun(std::map< double, std::vector<float> > surfaceFrame){
  std::vector<float> v;
  Eigen::MatrixXf localPath(surfaceFrame.size()*2,2);
  //std::cout<<"localPath.rows(): "<<localPath.rows()<<"\n";
  {
    odcore::base::Lock lockSurface(m_surfaceMutex);
    int I=0;
    for (std::map<double, std::vector<float> >::iterator it = surfaceFrame.begin();it !=surfaceFrame.end();it++){
      v=it->second;
      localPath(2*I,0)=v[0];
      localPath(2*I,1)=v[1];
      localPath(2*I+1,0)=v[2];
      localPath(2*I+1,1)=v[3];
      I++;
    }
  }
  // Remove negative path points
  bool preSet = false;
  bool STOP = false;
  if (localPath(0,0)<0.0f && localPath.rows()>0) {
    int count = 0;
    while (localPath(count,0)<0.0f){
        count++;
        if (count>localPath.rows()-1) {
          STOP = true;
          preSet = true;
          localPath.resize(1,2);
          localPath <<  1, 0;
          break;
        }
    }
    if (!preSet) {
      Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
      localPath.resize(localPath.rows()-count,2);
      localPath = localPathTmp;
    }
  }
  if (!preSet) {
    // Check for stop or "one point" signal
    if(localPath.rows() > 1){
      if (std::abs(localPath(1,0)) <= 0.0001f && std::abs(localPath(1,1)) <= 0.0001f) {
        Eigen::MatrixXf localPathTmp = localPath.row(0);
        localPath.resize(1,2);
        localPath = localPathTmp;
        STOP = true;
        std::cout << "STOP signal recieved " << std::endl;
      }
      else if(std::abs(localPath(0,0)) <= 0.0001f && std::abs(localPath(0,1)) <= 0.0001f && localPath.rows()<3){
        Eigen::MatrixXf localPathTmp = localPath.row(1);
        localPath.resize(1,2);
        localPath = localPathTmp;
        std::cout << "ONE POINT signal recieved " << std::endl;
      }
      else { //Place equidistant points
        float const distanceBetweenPoints = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.distanceBetweenPoints");
        bool const traceBack = getKeyValueConfiguration().getValue<bool>("logic-cfsd18-cognition-track.traceBack");
        Eigen::MatrixXf localPathCopy;
        if (traceBack){
          RowVector2f firstPoint = Track::traceBackToClosestPoint(localPath.row(0), localPath.row(1), Eigen::RowVector2f::Zero(1,2));
          localPathCopy.resize(localPath.rows()+1,2);
          localPathCopy.row(0) = firstPoint;
          localPathCopy.block(1,0,localPath.rows(),2) = localPath;
          localPath.resize(localPathCopy.rows(),2);
        } else{
          localPathCopy = localPath;
        }
        localPath = Track::placeEquidistantPoints(localPathCopy,false,-1,distanceBetweenPoints);
      }
    } //else Only one point remaining
  }
  //std::cout<<"localPath: "<<localPath<<"\n";
  auto kv = getKeyValueConfiguration();
  float const previewTime = kv.getValue<float>("logic-cfsd18-cognition-track.previewTime");
  float const velocityLimit = kv.getValue<float>("logic-cfsd18-cognition-track.velocityLimit");
  float const mu = kv.getValue<float>("logic-cfsd18-cognition-track.mu");
  float const axLimitPositive = kv.getValue<float>("logic-cfsd18-cognition-track.axLimitPositive");
  float const axLimitNegative = kv.getValue<float>("logic-cfsd18-cognition-track.axLimitNegative");
  float const headingErrorDependency = kv.getValue<float>("logic-cfsd18-cognition-track.headingErrorDependency");
  bool const sharp = kv.getValue<bool>("logic-cfsd18-cognition-track.RunSharp");
  float groundSpeedCopy;
  {
    odcore::base::Lock lockGroundSpeed(m_groundSpeedMutex);
    groundSpeedCopy = m_groundSpeed;
  }
  float previewDistance = std::abs(groundSpeedCopy)*previewTime;
  float pathLength=localPath.row(0).norm();
  if(localPath.rows()>1){
    for (int i = 0; i < localPath.rows()-1; i++) {
      pathLength+=(localPath.row(i+1)-localPath.row(i)).norm();
    }
  }
  if (previewDistance>pathLength) {
    std::cout<<"previewDistance "<< previewDistance <<" is longer than pathLength "<<pathLength<<std::endl;
    previewDistance = pathLength;
  }

  float headingRequest;
  float distanceToAimPoint;
  if (sharp) {
    headingRequest = Track::driverModelSharp(localPath, previewDistance);
    distanceToAimPoint = 3.0f;
  }
  else{
    auto steering = Track::driverModelSteering(localPath, groundSpeedCopy, previewTime);
    headingRequest = std::get<0>(steering);
    distanceToAimPoint = std::get<1>(steering);
  }
  float accelerationRequest = Track::driverModelVelocity(localPath, groundSpeedCopy, velocityLimit, axLimitPositive, axLimitNegative, previewDistance, headingRequest, headingErrorDependency, mu, STOP);

//std::cout << "Sending headingRequest: " << headingRequest << std::endl;
  //opendlv::logic::action::AimPoint o1;
  opendlv::proxy::GroundSteeringRequest o1;
  //o1.setAzimuthAngle(headingRequest);
  o1.setGroundSteering(headingRequest);
  odcore::data::Container c1(o1);
  getConference().send(c1);

  //Send for Ui TODO: Remove
  opendlv::logic::action::AimPoint o4;
  o4.setAzimuthAngle(headingRequest);
  o4.setDistance(distanceToAimPoint);
  odcore::data::Container c4(o4);
  getConference().send(c4);

  if (accelerationRequest >= 0.0f) {
//std::cout << "Sending accelerationRequest: " << accelerationRequest << std::endl;
    opendlv::proxy::GroundAccelerationRequest o2;
    o2.setGroundAcceleration(accelerationRequest);
    odcore::data::Container c2(o2);
    getConference().send(c2);
  }
  else if(accelerationRequest < 0.0f){
//std::cout << "Sending decelerationRequest: " << accelerationRequest << std::endl;
    opendlv::proxy::GroundDecelerationRequest o3;
    o3.setGroundDeceleration(-accelerationRequest);
    odcore::data::Container c3(o3);
    getConference().send(c3);
  }
}


Eigen::RowVector2f Track::traceBackToClosestPoint(Eigen::RowVector2f p1, Eigen::RowVector2f p2, Eigen::RowVector2f q)
{
   // Input: The coordinates of the first two points. (row vectors)
   //        A reference point q (vehicle location)
   // Output: the point along the line that is closest to the reference point.

   Eigen::RowVector2f v = p1-p2;	// The line to trace
   Eigen::RowVector2f n(1,2);	// The normal vector
   n(0,0) = -v(0,1); n(0,1) = v(0,0);
   //float d = (p1(0,0)*v(0,1)-v(0,0)*p1(0,1))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between [0,0] and the vector
   float d = (v(0,1)*(p1(0,0)-q(0,0))+v(0,0)*(q(0,1)-p1(0,1)))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between q and the vector
   return q+n*d;       // Follow the normal vector for that distance
}

Eigen::MatrixXf Track::placeEquidistantPoints(Eigen::MatrixXf oldPathPoints, bool nEqPointsIsKnown, int nEqPoints, float eqDistance)
{
// Places linearly equidistant points along a sequence of points.
// If nEqPoints is known it will not use the input value for eqDistance, and instead calculate a suitable value.
// If nEqPoints is not known it will not use the input value for nEqPoints, and instead calculate a suitable value.

  int nOld = oldPathPoints.rows();
  // Full path length, and save lengths of individual segments
  float pathLength = 0;
  Eigen::MatrixXf segLength(nOld-1,1);
  for(int i = 0; i < nOld-1; i = i+1)
  {
    segLength(i) = (oldPathPoints.row(i+1)-oldPathPoints.row(i)).norm();
    pathLength = pathLength + segLength(i);
  }

  if(nEqPointsIsKnown)
  {
    // Calculate equal subdistances
    eqDistance = pathLength/(nEqPoints-1);
  }
  else
  {
    //Calculate how many points will fit
    nEqPoints = std::ceil(pathLength/eqDistance)+1;
  }
  // The latest point that you placed
  Eigen::MatrixXf latestNewPointCoords = oldPathPoints.row(0);
  // The latest path point that you passed
  int latestOldPathPointIndex = 0;
  // How long is left of the current segment
  float remainderOfSeg = segLength(0);
  // The new list of linearly equidistant points
  Eigen::MatrixXf newPathPoints(nEqPoints,2);
  // The first new point should be at the same place as the first old point
  newPathPoints.row(0) = latestNewPointCoords;
  // A temporary vector
  Eigen::MatrixXf vec(1,2);
  // Temporary distances
  float distanceToGoFromLatestPassedPoint, lengthOfNextSeg;
  // Start stepping through the given path
  for(int i = 1; i < nEqPoints-1; i = i+1)
  {
    // If the next new point should be in the segment you are currently in, simply place it.
    if(remainderOfSeg > eqDistance)
    {
      vec = oldPathPoints.row(latestOldPathPointIndex+1)-latestNewPointCoords;
      latestNewPointCoords = latestNewPointCoords + (eqDistance/remainderOfSeg)*vec;
    }
    else // If you need to go to the next segment, keep in mind which old points you pass and how long distance you have left to go.
    {
      latestOldPathPointIndex = latestOldPathPointIndex+1;
      distanceToGoFromLatestPassedPoint = eqDistance-remainderOfSeg;
      lengthOfNextSeg = segLength(latestOldPathPointIndex);

      while(distanceToGoFromLatestPassedPoint > lengthOfNextSeg)
      {
        latestOldPathPointIndex = latestOldPathPointIndex+1;
        distanceToGoFromLatestPassedPoint = distanceToGoFromLatestPassedPoint - lengthOfNextSeg;
        lengthOfNextSeg = segLength(latestOldPathPointIndex);
      } // End of while

      latestNewPointCoords = oldPathPoints.row(latestOldPathPointIndex);
      vec = oldPathPoints.row(latestOldPathPointIndex+1)-latestNewPointCoords;
      latestNewPointCoords = latestNewPointCoords + (distanceToGoFromLatestPassedPoint/segLength(latestOldPathPointIndex))*vec;
    } // End of else
    // In every case, save the point you just placed and check how much of that segment is left.
    newPathPoints.row(i) = latestNewPointCoords;
    remainderOfSeg = (oldPathPoints.row(latestOldPathPointIndex+1)-latestNewPointCoords).norm();
  } // End of for
  // The last point should be at the same place as the last cone.
  newPathPoints.row(nEqPoints-1) = oldPathPoints.row(nOld-1);

  return newPathPoints;
} // End of placeEquidistantPoints

float Track::driverModelSharp(Eigen::MatrixXf localPath, float previewDistance){
  auto kv = getKeyValueConfiguration();
  int n = kv.getValue<int>("logic-cfsd18-cognition-track.nSharp");
  float K1 = kv.getValue<float>("logic-cfsd18-cognition-track.K1Sharp");
  float Ky = kv.getValue<float>("logic-cfsd18-cognition-track.KySharp");
  float C = kv.getValue<float>("logic-cfsd18-cognition-track.bigCSharp");
  float c = kv.getValue<float>("logic-cfsd18-cognition-track.smallcSharp");
  float wheelAngleLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.wheelAngleLimit");
  float headingRequest;
  if (localPath.rows()>2) {

  float sectionLength = previewDistance/n;
  std::vector<float> error(n);
  Eigen::MatrixXf::Index idx;
  float xMax = localPath.col(0).maxCoeff(&idx);
  error[0]=localPath(0,1);
  int k = 1;
  float sumPoints = 0.0f;
  for (int i = 0; i < localPath.rows()-1 ; i++) {
  sumPoints += (localPath.row(i+1)-localPath.row(i)).norm();
    if (sumPoints>=k*sectionLength) {
      if(k*sectionLength<xMax){
        error[k] = localPath(i,1);
        k++;
      }
      else{
        //std::cout<<"index: "<<idx<<std::endl;
        //std::cout<<"localPath(idx): "<<localPath(idx)<<std::endl;
        error[k] = localPath(idx,1);
        k++;
      }
    }
    if(k>n-1){
      break;
    }
  }
  if(k<n-2){
    error[n-1] = localPath(localPath.rows()-1,1);
  }

  float ey = std::atan2(localPath(10,1)-localPath(0,1),localPath(10,0)-localPath(0,0));
    //std::cout<<"K1e1 "<<K1*error[0]<<std::endl;
    //std::cout<<"Kyey "<<Ky*ey<<std::endl;
  float errorSum = 0.0f;
  int j=0;
  for (int i = 1; i < n; i++) {
    errorSum += error[i]*1.0f/expf(j)*C;
    //std::cout<<"e "<< i <<": "<<error[i]<<std::endl;
    //std::cout<<"K "<<i<<": "<<1.0f/expf(j)*C<<std::endl;
    //std::cout<<"K[i]e[i] i= "<<i<<": "<<error[i]*1.0f/expf(j)*C<<std::endl;
    j+=c;
  }

  headingRequest = Ky*ey + K1*error[1] + errorSum;
  if (headingRequest>=0) {
    headingRequest = std::min(headingRequest,wheelAngleLimit*3.14159265f/180.0f);
  } else {
    headingRequest = std::max(headingRequest,-wheelAngleLimit*3.14159265f/180.0f);
  }
  }
  else{
    headingRequest = 0.0f;
  }
  return headingRequest;
}


std::tuple<float, float> Track::driverModelSteering(Eigen::MatrixXf localPath, float groundSpeedCopy, float previewTime) {
  //driverModelSteering calculates the desired heading of CFSD18 vehicle
  // given the vehicles velocity, a set time to aimpoint and a path in local
  // coordinates (vehicle is origo).
  //
  //Input
  //   LOCALPATH               [n x 2] Local coordinates of the path [x,y]
  //   GROUNDSPEEDCOPY         [1 x 1] Velocity of the vehicle [m/s]
  //   PREVIEWTIME             [1 x 1] Time to aimpoint [s].
  //
  //Output
  //   HEADINGREQUEST  [1 x 1] Desired heading angle [rad]
  float headingRequest;
  Eigen::Vector2f aimPoint(2);
  bool preSet = false;
  bool const moveOrigin = getKeyValueConfiguration().getValue<bool>("logic-cfsd18-cognition-track.moveOrigin");
  if (moveOrigin && localPath.rows()>2) {
    // Move localPath to front wheel axis
    float const frontToCog = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.frontToCog");
    Eigen::MatrixXf foo = Eigen::MatrixXf::Zero(localPath.rows(),2);
    foo.col(0).fill(frontToCog);
    localPath = localPath-foo;

    // Remove negative path points
    if (localPath(0,0)<0.0f && localPath.rows()>0) {
      int count = 0;
      while (localPath(count,0)<0.0f){
          count++;
          if (count>localPath.rows()-1) {
            aimPoint << 1,
                        0;
            preSet = true;
            break;
          }
      }
      if(!preSet){
        Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
        localPath.resize(localPath.rows()-count,2);
        localPath = localPathTmp;
      }
    }
  }
  if (!preSet) {
    // Calculate the distance between vehicle and aimpoint;
    float previewDistance = std::abs(groundSpeedCopy)*previewTime;
    //std::cout << "previewDistance: "<<previewDistance<<"\n";
    float sumPoints = localPath.row(0).norm();
    // Sum the distance between all path points until passing previewDistance
    // or reaching end of path
    int k=0;
    while (previewDistance >= sumPoints && k < localPath.rows()-1) {
      sumPoints += (localPath.row(k+1)-localPath.row(k)).norm();
      k++;
    }

    if (sumPoints >= previewDistance) { // it means that the path is longer than previewDistance
      float distanceP1P2, overshoot, distanceP1AimPoint;
      if (k > 0) {// then the aimpoint will be placed after the first path element
        k--;
        // Distance between last two considered path elements P1 P2 where P2 is the overshoot element.
        distanceP1P2 = (localPath.row(k+1)-localPath.row(k)).norm();
        // Difference between distance to aimpoint and summed points.
        overshoot = sumPoints - previewDistance;
        // Distance between next to last considered path element P1 and aimpoint
        distanceP1AimPoint = distanceP1P2 - overshoot;
        // Linear interpolation
        aimPoint = (localPath.row(k+1)-localPath.row(k))*(distanceP1AimPoint/distanceP1P2) + localPath.row(k);
      }
      else {// Place aimpoint on first path element
         //interpolation
        distanceP1P2 = localPath.row(0).norm(); // Distance is equal to the distance to the first point;
        overshoot = sumPoints - previewDistance;
        distanceP1AimPoint = distanceP1P2 - overshoot;
        aimPoint = localPath.row(0)*(distanceP1AimPoint/distanceP1P2);
        //aimPoint = localPath.row(0);
      }
    }
    // If the path is too short, place aimpoint at the last path element
    else {
      aimPoint = localPath.row(localPath.rows()-1);
    }
  }
  // Angle to aimpoint
  headingRequest = atan2(aimPoint(1),aimPoint(0));
  // Limit heading request due to physical limitations
  float wheelAngleLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.wheelAngleLimit");
  if (headingRequest>=0) {
    headingRequest = std::min(headingRequest,wheelAngleLimit*3.14159265f/180.0f);
  } else {
    headingRequest = std::max(headingRequest,-wheelAngleLimit*3.14159265f/180.0f);
  }

  float distanceToAimPoint=aimPoint.norm();
  //std::cout << "AimPoint: "<<aimPoint<<"\n";
  //std::cout << "distanceToAimPoint: "<<distanceToAimPoint<<"\n";


  return std::make_tuple(headingRequest,distanceToAimPoint);
}

float Track::driverModelVelocity(Eigen::MatrixXf localPath, float groundSpeedCopy, float velocityLimit, float axLimitPositive, float axLimitNegative, float previewDistance, float headingRequest, float headingErrorDependency, float mu, bool STOP){
  //driverModelVelocity calculates the desired acceleration of the CFSD18
  //vehicle.
  //
  //Input
  //   LOCALPATH                   [n x 2] Local coordinates of the path [x,y]
  //   GROUNDSPEEDCOPY             [1 x 1] Velocity of the vehicle [m/s]
  //   VELOCITYLIMIT               [1 x 1] Maximum allowed velocity [m/s]
  //   (AYLIMIT                     [1 x 1] Allowed lateral acceleration [m/s^2])
  //   AXLIMITPOITIVE              [1 x 1] Allowed longitudinal acceleration (positive) [m/s^2]
  //   AXLIMITNEGATIVE             [1 x 1] Allowed longitudinal acceleration (negative) [m/s^2]
  //   HEADINGREQUEST              [1 x 1] Heading error to aimpoint [rad]
  //   HEADINGERRORDEPENDENCY      [1 x 1] Constant
  //   MU                          [1 x 1] Friction coefficient
  //
  //Output
  //   ACCELERATIONREQUEST         [1 x 1] Signed desired acceleration
  float accelerationRequest;
  Eigen::VectorXf speedProfile;
  std::vector<float> curveRadii;
  int step;
  float g = 9.81f;
  float axSpeedProfile = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.axSpeedProfile");
  if (axSpeedProfile<0) {
    axSpeedProfile = std::max(axLimitPositive,-axLimitNegative);
  }
  float ayLimit = sqrtf(powf(mu*g,2)-powf(axSpeedProfile,2));
  if (std::isnan(ayLimit)) {
    std::cout<<"ayLimit is NAN, axLimit set too High"<<std::endl;
    ayLimit = 1.0f;
    std::cout<<"ayLimit set to: "<<ayLimit<<std::endl;
  }

  if ((!STOP) && (localPath.rows() > 2)){
    // Caluclate curvature of path
    bool polyFit = getKeyValueConfiguration().getValue<bool>("logic-cfsd18-cognition-track.polyFit");
      if (polyFit){
        step = 0;
        curveRadii = curvaturePolyFit(localPath);
      }
      else{
        step = getKeyValueConfiguration().getValue<int>("logic-cfsd18-cognition-track.step");
        while (localPath.rows()-(2*step)<=0 && step > 0){ //This makes sure that step is as big as possible (up to limit)
          step--;
        }
        curveRadii = curvatureTriCircle(localPath,step);
      }

    float const wheelAngleLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.wheelAngleLimit");
    float const distanceBetweenPoints = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.distanceBetweenPoints");
    int index = floor(previewDistance/distanceBetweenPoints);
    if (index>localPath.rows()-1) {
      index = localPath.rows()-1;
      std::cout<<"INDEX TOO FAR (it was possible)"<<std::endl;
    }
    float previewAngle = std::atan2(localPath(index,1),localPath(index,0));
    if (previewAngle>=0) {
      previewAngle = std::min(previewAngle,wheelAngleLimit*3.14159265f/180.0f);
    } else {
      previewAngle = std::max(previewAngle,-wheelAngleLimit*3.14159265f/180.0f);
    }
    float headingError = std::abs(headingRequest)/(wheelAngleLimit*3.14159265f/180.0f);
    //std::cout<<"headingError: "<<headingError<<std::endl;
    //std::cout<<"headinError scaling: "<<1.0f-headingError*headingErrorDependency<<std::endl;

    // Set velocity candidate based on expected lateral acceleration limit
    speedProfile.resize(curveRadii.size());
    for (uint32_t k = 0; k < curveRadii.size(); k++){
    speedProfile(k) = std::min(sqrtf(ayLimit*curveRadii[k]),velocityLimit)*(1.0f-headingError*headingErrorDependency);
    }
    /*---------------------------------------------------------------------------*/
    //std::cout<<"SpeedProfile: "<<speedProfile.transpose()<<std::endl;
    /*for (uint32_t i = 0; i < curveRadii.size(); ++i)
    {
      std::cout<<curveRadii[i]<<std::endl;
    }*/

    //float const distanceBetweenPoints = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.distanceBetweenPoints");
    //float brakeMetersBefore = 4;
    //int K = round(brakeMetersBefore/distanceBetweenPoints);
    float tb;
    float tv;
    float ta;
    float s=0;
    float tmp;
    float brakeTime=100000.0f;
    float accPointMin=100000.0f;
    //float rollResistance = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.rollResistance");
    int i;
    int idx;
    int accIdx = 0;
    std::vector<float> distanceToCriticalPoint;
    for (int k=0; k<step; k++){
      s+=(localPath.row(k+1)-localPath.row(k)).norm();
    }
    for (i=0; i<speedProfile.size(); i++){
      s+=(localPath.row(i+step+1)-localPath.row(i+step)).norm();
      distanceToCriticalPoint.push_back(s);
      tb = (speedProfile(i)-groundSpeedCopy)/(axLimitNegative);//time to reach velocity
      tv = s/groundSpeedCopy;
      if (tb>0.0f) { //braking is needed to reach this velocity
        tmp = tv-tb; // when we need to brake, if =0 we need to brake now, if <0 we are braking too late
        if(tmp<brakeTime){
          brakeTime=tmp;
          idx=i;
        }
      }
      else {
        if (speedProfile(i)<accPointMin) {
          accPointMin = speedProfile(i);
          accIdx = i;
          ta = tv;
        }
      }
    }
    float const wheelBase = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.wheelBase");
    float ay = powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest)));
    if(brakeTime<=0.0f){
      if (brakeTime<0.0f) {
        //std::cout<<"braking too late, brakeTime: "<<brakeTime<<std::endl;
      }
      accelerationRequest = axLimitNegative;
      //std::cout<<"brake max: "<<accelerationRequest<<std::endl;
      /*if (sqrtf(powf(ay,2)+powf(accelerationRequest,2)) >= g*mu) {
        accelerationRequest = -sqrtf(powf(g*mu,2)-powf(ay,2))*0.9f;
        std::cout<<"accreq limited: "<<accelerationRequest<<std::endl;
      }
      if (std::isnan(accelerationRequest)) {
        std::cout<<"accelerationRequest 1 is NaN, ay = "<<ay<<std::endl;
        accelerationRequest = 0.0f;
      }*/
    }
    else if (brakeTime>0.0f && brakeTime<=0.3f){
      /*if (idx-K>=0 && curveRadii[idx]<10.0f) {
        accelerationRequest = std::max((speedProfile(idx)-groundSpeedCopy)/(2*distanceToCriticalPoint[idx-K]),axLimitNegative);
      }else{*/
        accelerationRequest = std::max((speedProfile(idx)-groundSpeedCopy)/(2*distanceToCriticalPoint[idx]),axLimitNegative);
        accelerationRequest = axLimitNegative;
        //std::cout<<"brake prematurely: "<<accelerationRequest<<std::endl;
      //}
      /*if (sqrtf(powf(ay,2)+powf(accelerationRequest,2)) >= g*mu) {
        accelerationRequest = -sqrtf(powf(g*mu,2)-powf(ay,2))*0.9f;
        std::cout<<"accreq limited: "<<accelerationRequest<<std::endl;
        if (std::isnan(accelerationRequest)) {
          std::cout<<"accelerationRequest 2 is NaN, ay = "<<ay<<std::endl;
          accelerationRequest = 0.0f;
        }
      }*/
      /*if (accelerationRequest>rollResistance*g) {
        accelerationRequest = 0.0f;
        std::cout<<"roll resistance is enough"<<std::endl;
      }*/
    }
    else if(brakeTime<= 0.5f){
      accelerationRequest = 0.0f;
      //std::cout<<"no need to accelerate, braking soon: "<<accelerationRequest<<std::endl;
    }
    else if((speedProfile(accIdx)-groundSpeedCopy) < 0.5f && ta<-0.5f){ //UNUSED
      accelerationRequest = 0.0f;
      std::cout<<"no need to accelerate: "<<accelerationRequest<<std::endl;
    }
    else{
      accelerationRequest = axLimitPositive;
      //std::cout<<"accelerate max: "<<accelerationRequest<<std::endl;
      if (sqrtf(powf(ay,2)+powf(accelerationRequest,2)) >= g*mu) {
        accelerationRequest = sqrtf(powf(g*mu,2)-powf(ay,2))*0.9f;
        std::cout<<"accreq limited: "<<accelerationRequest<<std::endl;
        if (std::isnan(accelerationRequest)) {
          std::cout<<"accelerationRequest 3 is NaN, ay = "<<ay<<std::endl;
          accelerationRequest = 0.0f;
        }
      }

    }

    /*---------------------------------------------------------------------------*/
    /*// Back propagate the whole path and lower velocities if deceleration cannot
    // be achieved. //TODO not pow g*mu
    for (int k = speedProfile.size()-1; k > 0 ; k-=1) {
      // Distance between considered path points
      float pointDistance = (localPath.row(k+step)-localPath.row(k+step-1)).norm();
      // Requiered acceleration to achieve velocity of following point from previous point
      float ax = (speedProfile(k)-speedProfile(k-1))/(2.0f*pointDistance);
      // Lateral acceleration at k
      float ay = powf(speedProfile(k),2)/curveRadii[k];
      // If deceleration is needed
      if (ax < 0.0f) {
        // If deceleration is too high for point k, or higher than vehicle specific limit
        if (sqrtf(powf(ay,2)+powf(ax,2)) >= powf(g*mu,2) || ax < axLimitNegative) {
          ax = std::max((-sqrtf(powf(g*mu,2)-powf(powf(speedProfile(k),2)/curveRadii[k],2)))*0.9f,axLimitNegative); //0.9 is a safetyfactor since ax must be less than rhs
          speedProfile(k-1) = sqrtf(powf(speedProfile(k),2)-2.0f*ax*pointDistance);
        }
      }
    }
    std::cout << "speedProfile = " << speedProfile.transpose() << "\n";
    // Choose velocity to achieve
    // Limit it dependent on the path length and heading request (heading error)
    if(previewTime*groundSpeedCopy>distanceToAimPoint*1000000.0f){
      speedProfile(0)=distanceToAimPoint/previewTime;
    }
    float pathLength = 0;
    for (int k=0; k<localPath.rows()-1; k++) {
      pathLength += (localPath.row(k+1)-localPath.row(k)).norm();
    }
  std::cout << "pathLength: " <<pathLength<< '\n';
    if(previewTime*groundSpeedCopy>pathLength){
      speedProfile(0)=distanceToAimPoint/previewTime;
    }
    float desiredVelocity = speedProfile(0)/(1.0f + headingErrorDependency*std::abs(headingRequest));
    // Calculate distance to desired velocity
    float distanceToAimVelocity = (localPath.row(step)).norm(); //TODO localPath.row(step) = 0 if spec case one point....
    // Transform into acceleration
    accelerationRequest = (desiredVelocity-groundSpeedCopy)/(2.0f*distanceToAimVelocity); // TODO use ax directly?
    // Limit acceleration request for positive acceleration
    float const wheelBase = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.wheelBase");
    if (accelerationRequest > 0.0f && (sqrtf(powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)+powf(accelerationRequest,2)) >= powf(g*mu,2) || accelerationRequest > axLimitPositive)) {
      std::cout << "accelerationRequest pos limited: " << accelerationRequest << std::endl;
      if (powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))) < g*mu){
        accelerationRequest = std::min((sqrtf(powf(g*mu,2)-powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)))*0.9f,axLimitPositive); //0.9 is a safetyfactor since ax must be less than rhs
      }
      else{
        std::cout<<"We are going to fast, just Roll with it"<<"\n";
        accelerationRequest = 0.0f;
      }
    }
    // Limit acceleration request for negative acceleration TODO: use groundSpeed instead of speedProfile(0)?? Fails if driving too fast
    if (accelerationRequest < 0.0f && (sqrtf(powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)+powf(accelerationRequest,2)) >= powf(g*mu,2) || accelerationRequest < axLimitNegative)) {
      std::cout << "accelerationRequest neg limited: " << accelerationRequest << std::endl;
      if (powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))) < g*mu){
      accelerationRequest = std::max((-sqrtf(powf(g*mu,2)-powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)))*0.9f,axLimitNegative); //0.9 is a safetyfactor since ax must be less than rhs
      }
      else{
        std::cout<<"Fuck it, we are already sliding, BREAK HARD"<<"\n";
        accelerationRequest = axLimitNegative;
      }
    }*/
    /*---------------------------------------------------------------------------*/
  } //end if(!STOP)
  else if(STOP){
    if (std::abs(groundSpeedCopy) > 0.01f){
      accelerationRequest = axLimitNegative;
      std::cout << "BREAKING TO STOP " << std::endl;
    } else {accelerationRequest = 0.0f;}
  }
  else{
    accelerationRequest = 0.0f;
  }
  //std::cout << "groundSpeedCopy: " << groundSpeedCopy << std::endl;
  return accelerationRequest;
}

std::vector<float> Track::curvatureTriCircle(Eigen::MatrixXf localPath, int step){
  // Segmentize the path and calculate radius of segments
  // - First radius is calculated at 2nd path point
  // - Last radius is calculated at 2nd to last path point
  // Note! 3 points in a row gives infinate radius.
  std::vector<float>  curveRadii(localPath.rows()-(2*step));
  for (int k = 0; k < localPath.rows()-(2*step); k++) {
    // Choose three points and make a triangle with sides A(p1p2),B(p2p3),C(p1p3)
    float A = (localPath.row(k+step)-localPath.row(k)).norm();
    float B = (localPath.row(k+2*step)-localPath.row(k+step)).norm();
    float C = (localPath.row(k+2*step)-localPath.row(k)).norm();

    // sort side lengths as A >= B && B >= C
    if (A < B) {
        std::swap(A, B);
    }
    if (A < C) {
        std::swap(A, C);
    }
    if (B < C) {
        std::swap(B, C);
    }

    if (C-(A-B) <= 0) {
      //std::cout << "WARNING! The data are not side-lengths of a real triangle" << std::endl;
      curveRadii[k] = 10000; // Large radius instead of inf value will reach velocitylimit
    }
    else {
      // https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
      // Calculate triangle area
      float triangleArea = 0.25f*sqrtf((A+(B+C))*(C-(A-B))*(C+(A-B))*(A+(B-C)));
      // Calculate the radius of the circle that matches the points
      curveRadii[k] = (A*B*C)/(4*triangleArea);
    }
  }
  /*std::cout<<"curveRadii: "<<" ";
  for (size_t i = 0; i < curveRadii.size(); i++) {
    std::cout<<curveRadii[i]<<" ";
  }
  std::cout<<"\n";*/

  return curveRadii;
}

std::vector<float> Track::curvaturePolyFit(Eigen::MatrixXf localPath){ // TODO: double check coordinate system, maybe switch x/y
  auto startPoly = std::chrono::system_clock::now();
  int n = getKeyValueConfiguration().getValue<int>("logic-cfsd18-cognition-track.polynomialDegree");
  int pointsPerSegment =  getKeyValueConfiguration().getValue<int>("logic-cfsd18-cognition-track.pointsPerSegment");
  int i,j,segments,N;
  int k=0;
  uint32_t l=0;
  Eigen::VectorXf dividedPathX(localPath.rows()); // TODO: This is now maximum possible size, which in some cases is unneccesary
  Eigen::VectorXf dividedPathY(localPath.rows());
  std::vector<Eigen::VectorXf> dividedPathsX;
  std::vector<Eigen::VectorXf> dividedPathsY;
      //curveRadii.insert(curveRadii.end(), R.begin(), R.end() );
  while (l < localPath.rows()-2){ //TODO improve this section
    while ((localPath(l,0) >= localPath(l+1,0)) && l < localPath.rows()-2){ // While X decreasing
      dividedPathX(k) = localPath(l,0);
      dividedPathY(k) = localPath(l,1);
      l++;
      k++;
        if (!((localPath(l,0) >= localPath(l+1,0)) && l < localPath.rows()-2)){ //If not decreasing, save this part of path
          dividedPathsX.push_back(dividedPathX.segment(0,k));
          dividedPathsY.push_back(dividedPathY.segment(0,k));
          k=0;
        }
    }
    while ((localPath(l,0) < localPath(l+1,0)) && l < localPath.rows()-2){ //While X increasing
      dividedPathX(k) = localPath(l,0);
      dividedPathY(k) = localPath(l,1);
      l++;
      k++;
        if (!((localPath(l,0) < localPath(l+1,0)) && l < localPath.rows()-2)){ // If not increasing, save this part of path
          dividedPathsX.push_back(dividedPathX.segment(0,k));
          dividedPathsY.push_back(dividedPathY.segment(0,k));;
          k=0;
        }
    }
  }

  std::vector<float> curveRadii;
  std::vector<float> R;
  Eigen::VectorXf pathx;
  Eigen::VectorXf pathy;
  Eigen::VectorXf x;
  Eigen::VectorXf y;
  Eigen::VectorXf a(n+1);
  int segmentBegin;
  int segmentLength;

  for (uint32_t P=0; P<dividedPathsX.size(); P++) {
    pathx = dividedPathsX[P];
    pathy = dividedPathsY[P];
    N = pointsPerSegment; // number of path points per segment
    if (pathx.size()<N) {
      N = pathx.size();
    }
    segments = floor(pathx.size()/N); //number of segments

    for (int p=0; p<segments; p++){
      if ((p<segments-1) || (!(segments*N<pathx.size())) ){
      segmentBegin = p*N;
      segmentLength = N;
      }
      if (segments*N<pathx.size() && p+1>=segments){ // Use all points in last segment, N+rest
       segmentBegin = p*N;
       segmentLength = N+pathx.size()-N*segments;
       N = segmentLength;
      }
      x = pathx.segment(segmentBegin,segmentLength).array()-x(0);
      y = pathy.segment(segmentBegin,segmentLength).array()-y(0);

      Eigen::VectorXf X(2*n+1);                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      for (i=0;i<2*n+1;i++){
        X(i)=0;
        for (j=0;j<N;j++)
            X(i)=X(i)+powf(x(j),i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      }
      //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
      Eigen::MatrixXf B(n+1,n+2);
      for (i=0;i<=n;i++)
        for (j=0;j<=n;j++)
            B(i,j)=X(i+j);            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
      Eigen::VectorXf Y(n+1);                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      for (i=0;i<n+1;i++){
        Y(i)=0;
        for (j=0;j<N;j++)
        Y(i)=Y(i)+powf(x(j),i)*y(j);        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      }
      for (i=0;i<=n;i++)
        B(i,n+1)=Y(i);                //load the values of Y as the last column of B(Normal Matrix but augmented)

      for (i=0;i<n+1;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n+1;k++)
            if (B(i,i)<B(k,i))
                for (j=0;j<=n+1;j++){
                    float temp=B(i,j);
                    B(i,j)=B(k,j);
                    B(k,j)=temp;
                }
      for (i=0;i<n;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<n+1;k++){
                float t=B(k,i)/B(i,i);
                for (j=0;j<=n+1;j++)
                    B(k,j)=B(k,j)-t*B(i,j);    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
      for (i=n;i>=0;i--){                //back-substitution
        a(i)=B(i,n+1);                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n+1;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
                a(i)=a(i)-B(i,j)*a(j);
        a(i)=a(i)/B(i,i);            //now finally divide the rhs by the coefficient of the variable to be calculated
      }

      R.resize(x.size()); // stores curvatures
      for(uint32_t m=0; m<R.size();m++){
        R[m] = 1/std::abs(2*a(2)+6*a(3)*x(m))/powf(1+powf(a(1)+2*a(2)*x(m)+3*a(3)*powf(x(m),2),2),1.5);
      }
      curveRadii.insert(curveRadii.end(), R.begin(), R.end());
    } // end p-loop
  } // end P-loop
  auto stopPoly = std::chrono::system_clock::now();
  auto timePoly = std::chrono::duration_cast<std::chrono::microseconds>(stopPoly - startPoly);
  std::cout << "Polyfit Time:" << timePoly.count() << std::endl;
  std::cout<<"curveRadii: "<<" ";
  for (size_t m = 0; m < curveRadii.size(); m++) {
    std::cout<<curveRadii[m]<<" ";
  }
  std::cout<<"\n";
  return curveRadii;
} // end curvaturePolyFit


} // end namespace
}
}
}
