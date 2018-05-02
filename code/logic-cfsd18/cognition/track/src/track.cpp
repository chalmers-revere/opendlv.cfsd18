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
  m_pathMutex{},
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
      std::cout << "TRACK RECIEVED A SURFACE!" << std::endl;
  //    m_lastTimeStamp = a_container.getSampleTimeStamp();
      auto surfaceProperty = a_container.getData<opendlv::logic::perception::GroundSurfaceProperty>();
      int surfaceId = surfaceProperty.getSurfaceId();
      auto nSurfacesInframe = surfaceProperty.getProperty();

      if (m_newFrame) {
        m_nSurfacesInframe = std::stoul(nSurfacesInframe);
        m_surfaceId = surfaceId;
        std::cout << "SurfaceId: " << surfaceId<<std::endl;
        std::cout << "nSurfacesInframe: " << nSurfacesInframe<<std::endl;
        m_newFrame = false;
      }

  }

  if (a_container.getDataType() == opendlv::logic::perception::GroundSurfaceArea::ID()) {
    int objectId;
    {
    odcore::base::Lock lockSurface(m_surfaceMutex);
      auto groundSurfaceArea = a_container.getData<opendlv::logic::perception::GroundSurfaceArea>();
      objectId = groundSurfaceArea.getSurfaceId();
      odcore::data::TimeStamp containerStamp = a_container.getReceivedTimeStamp();
      double timeStamp = containerStamp.toMicroseconds();
      if (m_newId) { // TODO: does it need to be global? can it be initialized in another way?
        m_objectId = (objectId!=m_lastObjectId)?(objectId):(-1);
        //std::cout << "newId, m_objectId: " <<m_objectId <<std::endl;
        m_newId=(m_objectId !=-1)?(false):(true);
      }
      //std::cout << "objectId: " <<objectId <<std::endl;
      //std::cout << "m_objectId: " <<m_objectId <<std::endl;
      //std::cout << "m_lastObjectId: " <<m_lastObjectId <<std::endl;
      float x1 = groundSurfaceArea.getX1();
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
        //std::cout << "Surfaces in frame: " <<m_surfaceFrame.size() <<std::endl;
        /*for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
          v = it->second;
          for (size_t i = 0; i < 4; i++) {
            std::cout<<v[i]<<"\n";
          }
        }*/
        m_timeReceived = std::chrono::system_clock::now();
      } else if (objectId != m_lastObjectId){
        //std::cout << "objectId in buffer: " <<objectId <<std::endl;
        m_surfaceFrameBuffer[timeStamp] = v;
        //std::cout << "Surfaces in buffer: " <<m_surfaceFrameBuffer.size() <<std::endl;
        /*for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
          v = it->second;
          for (size_t i = 0; i < 4; i++) {
            std::cout<<v[i]<<"\n";
          }
        }*/
      }
    }
    auto wait = std::chrono::system_clock::now();
    std::chrono::duration<double> dur = wait-m_timeReceived;
    double duration = (m_objectId!=-1)?(dur.count()):(-1.0);
    double receiveTimeLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.receiveTimeLimit");

    if ((m_surfaceFrame.size()==m_nSurfacesInframe || duration>receiveTimeLimit)) { //!m_newFrame && objectId==m_surfaceId &&
      //std::cout<<"Run condition OK "<<"\n";
      //std::cout << "duration: " <<duration <<std::endl;
      std::map< double, std::vector<float> > surfaceFrame;
      {
      odcore::base::Lock lockSurface(m_surfaceMutex);
        m_newFrame = true;
        surfaceFrame = m_surfaceFrame;
        m_surfaceFrame = m_surfaceFrameBuffer;
        m_surfaceFrameBuffer.clear();
        m_lastObjectId = m_objectId;
        m_newId = true;
        //std::cout << "Cleared buffer " <<std::endl;
      }
      //std::cout << "Run " << surfaceFrame.size() << " surfaces"<< std::endl;
      std::thread surfaceCollector(&Track::collectAndRun, this, surfaceFrame);
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

  //std::vector<double> timeStamps(surfaceFrame.size());
  std::vector<float> v;
  Eigen::MatrixXf localPath(surfaceFrame.size()*2,2);
  //std::cout<<"localPath.rows(): "<<localPath.rows()<<"\n";
  {
    odcore::base::Lock lockSurface(m_surfaceMutex);
    odcore::base::Lock lockPath(m_pathMutex);
    int I=0;
    for (std::map<double, std::vector<float> >::iterator it = surfaceFrame.begin();it !=surfaceFrame.end();it++){
      v=it->second;
      localPath(2*I,0)=v[0];
      localPath(2*I,1)=v[1];
      localPath(2*I+1,0)=v[2];
      localPath(2*I+1,1)=v[3];
      I++;
    }
    //std::cout<<"localPath: "<<localPath<<"\n";

  }
  //################ RUN and SEND ##################
  if(localPath.rows() > 0){ // TODO: Check for bad path
    float groundSpeedCopy;
    {
      odcore::base::Lock lockGroundSpeed(m_groundSpeedMutex);
      groundSpeedCopy = m_groundSpeed;
    }

    bool STOP = false;
    Eigen::MatrixXf localPathCopy;
    {
      odcore::base::Lock lockPath(m_pathMutex);
      if (std::abs(localPath(1,0)) <= 0.0001f && std::abs(localPath(1,1)) <= 0.0001f && localPath.rows()<3) {
        localPathCopy = localPath.row(0);
        STOP = true;
        std::cout << "STOP! " << std::endl;
      }
      else if(localPath.rows()<3){
        localPathCopy = localPath.row(1);
      }
      else{
        if (localPath(0,0)<=0.0f) {
          int i = 0;
          while (localPath(i,0)<=0.0f){
            i++;
            if (i>localPath.rows()) { //Whole path removed!
              STOP = true;
              i--;
              localPathCopy <<  1, 0;
              break;
            }
          }
          Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-i);
          localPath.resize(localPath.rows()-i,2);
          localPath = localPathTmp;
        }
        if (!STOP){
          float const distanceBetweenPoints = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.distanceBetweenPoints");
          bool const traceBack = getKeyValueConfiguration().getValue<bool>("logic-cfsd18-cognition-track.traceBack");
          if (traceBack) {
            RowVector2f firstPoint = Track::traceBackToClosestPoint(localPath.row(0), localPath.row(1), Eigen::RowVector2f::Zero(1,2));
            localPathCopy.resize(localPath.rows()+1,2);
            localPathCopy.row(0) = firstPoint;
            localPathCopy.block(1,0,localPath.rows(),2) = localPath;
          } else{localPathCopy = localPath;}
          localPathCopy = Track::placeEquidistantPoints(localPathCopy,false,-1,distanceBetweenPoints);
          //std::cout << "LocalPathCopy: " <<localPathCopy<<"\n";
        }
      }
    }

    auto kv = getKeyValueConfiguration();
    float const previewTime = kv.getValue<float>("logic-cfsd18-cognition-track.previewTime");
    float const velocityLimit = kv.getValue<float>("logic-cfsd18-cognition-track.velocityLimit");
    float const mu = kv.getValue<float>("logic-cfsd18-cognition-track.mu");
    float const axLimitPositive = kv.getValue<float>("logic-cfsd18-cognition-track.axLimitPositive");
    float const axLimitNegative = kv.getValue<float>("logic-cfsd18-cognition-track.axLimitNegative");
    float const headingErrorDependency = kv.getValue<float>("logic-cfsd18-cognition-track.headingErrorDependency");

    auto steering = Track::driverModelSteering(localPathCopy, groundSpeedCopy, previewTime);
    float headingRequest = std::get<0>(steering);
    float distanceToAimPoint = std::get<1>(steering);
    float accelerationRequest = Track::driverModelVelocity(localPathCopy, groundSpeedCopy, velocityLimit, axLimitPositive, axLimitNegative, headingRequest, headingErrorDependency, distanceToAimPoint, previewTime, mu, STOP);

std::cout << "Sending headingRequest: " << headingRequest << std::endl;
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
std::cout << "Sending accelerationRequest: " << accelerationRequest << std::endl;
      opendlv::proxy::GroundAccelerationRequest o2;
      o2.setGroundAcceleration(accelerationRequest);
      odcore::data::Container c2(o2);
      getConference().send(c2);
    }
    else if(accelerationRequest < 0.0f){
std::cout << "Sending decelerationRequest: " << accelerationRequest << std::endl;
      opendlv::proxy::GroundDecelerationRequest o3;
      o3.setGroundDeceleration(-accelerationRequest);
      odcore::data::Container c3(o3);
      getConference().send(c3);
    }
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

  // Move localPath to front wheels
    /*float const frontToCog = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.frontToCog");
    Eigen::MatrixXf foo = Eigen::MatrixXf::Zero(localPath.rows(),2);
    foo.col(0).fill(frontToCog);
    localPath = localPath-foo;*/
  // Calculate the distance between vehicle and aimpoint;
  float previewDistance = std::abs(groundSpeedCopy)*previewTime;
  std::cout<<"previewDistance: "<<previewDistance<<" m \n";
  //Distance to aimpoint is currently calculated only on path, Not from vehicle
  float sumPoints = localPath.row(0).norm(); //TODO
  // Sum the distance between all path points until passing previewDistance
  // or reaching end of path
  int k=0;
  while (previewDistance >= sumPoints && k < localPath.rows()-1) {
    sumPoints += (localPath.row(k+1)-localPath.row(k)).norm();
    k++;
  }
  Eigen::Vector2f aimPoint(2);
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
    else {// not needed if sumPoints is initialized as zero, (and previewDistance>0)
      /*
      distanceP1P2 = localPath.row(0).norm(); // Distance is equal to the distance to the first point;
      overshoot = sumPoints - previewDistance;
      distanceP1AimPoint = distanceP1P2 - overshoot;
      aimPoint = localPath.row(0)*(distanceP1AimPoint/distanceP1P2);*/
      aimPoint = localPath.row(0);
    }
  }
  // If the path is too short, place aimpoint at the last path element
  else {
    aimPoint = localPath.row(localPath.rows()-1);
  }
  // Angle to aimpoint
  float headingRequest;
  headingRequest = atan2(aimPoint(1),aimPoint(0));
  // Limit heading request due to physical limitations
  float wheelAngleLimit = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.wheelAngleLimit");
  if (headingRequest>=0) {
    headingRequest = std::min(headingRequest,wheelAngleLimit*3.14159265f/180.0f);
  } else {
    headingRequest = std::max(headingRequest,-wheelAngleLimit*3.14159265f/180.0f);
  }
  float distanceToAimPoint=aimPoint.norm();
  std::cout << "AimPoint: "<<aimPoint<<"\n";
  std::cout << "distanceToAimPoint: "<<distanceToAimPoint<<"\n";
  std::cout << "previewDistance: "<<previewDistance<<"\n";

  return std::make_tuple(headingRequest,distanceToAimPoint);
}

float Track::driverModelVelocity(Eigen::MatrixXf localPath, float groundSpeedCopy, float velocityLimit, float axLimitPositive, float axLimitNegative, float headingRequest, float headingErrorDependency, float distanceToAimPoint, float previewTime, float mu, bool STOP){
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
  float ayLimit = mu*g*0.8f;

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
    // Set velocity candidate based on expected lateral acceleration limit
    speedProfile.resize(curveRadii.size());
    for (uint32_t k = 0; k < curveRadii.size(); k++){
    speedProfile(k) = std::min(sqrtf(ayLimit*curveRadii[k]),velocityLimit);
    }

    // Back propagate the whole path and lower velocities if deceleration cannot
    // be achieved.
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
    if(previewTime*groundSpeedCopy>distanceToAimPoint){
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
    }
  } //end if(!STOP)
  else if(STOP){
    if (std::abs(groundSpeedCopy) > 0.001f){
      accelerationRequest = axLimitNegative;
      std::cout << "BREAKING TO STOP " << std::endl;
    } else {accelerationRequest = 0.0f;}
  }
  else{
    accelerationRequest = 0.0f;
  }
  std::cout << "groundSpeedCopy: " << groundSpeedCopy << std::endl;
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
  std::cout<<"curveRadii: "<<" ";
  for (size_t i = 0; i < curveRadii.size(); i++) {
    std::cout<<curveRadii[i]<<" ";
  }
  std::cout<<"\n";
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
