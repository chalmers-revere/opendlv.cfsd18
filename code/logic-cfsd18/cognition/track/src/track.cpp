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

#include "track.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace cognition {

Track::Track(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-cognition-track"),
  m_localPath()
{
}

Track::~Track()
{
}


void Track::nextContainer(odcore::data::Container &a_container)
{

<<<<<<< HEAD
  if (a_container.getDataType() == opendlv::logic::perception::Surface::ID()) {
    auto surface = a_container.getData<opendlv::logic::perception::Surface>();
    auto localPath = surface.getSurfaceId();
    //std::string type = Surface.getType();
    std::cout << "[cognition] TRACK.CPP IS RECIEVING SURFACE " << localPath << std::endl;
    /*float timeToAimPoint = 1;
    float currentVelocity = 5;
    float accelerationLimit = 5;
    float decelerationLimit = -5;
    float headingRequest = Track::driverModelSteering(currentVelocity, timeToAimPoint, localPath);
    float accelerationRequest = Track::driverModelVelocity(currentVelocity, accelerationLimit, decelerationLimit, headingRequest,localPath);
    */
    float accelerationRequest;
    if (localPath > 12345){
      accelerationRequest = 3.4;
    }
    else {
      accelerationRequest = -2.0;
    }
    float headingRequest = 1.2;
=======
  if (a_container.getDataType() == opendlv::logic::perception::GroundSurface::ID()) {
    auto surface = a_container.getData<opendlv::logic::perception::GroundSurface>();
    m_localPath << surface.getSurfaceId();

    std::cout << "Message " << a_container.getDataType() <<" recieved" << std::endl;
    std::cout << "m_localPath set to " << m_localPath << std::endl;
    float previewTime = 1;
    float currentVelocity = 5;
    float accelerationLimit = 5;
    float decelerationLimit = -5;
    float headingRequest = Track::driverModelSteering(currentVelocity, previewTime);
    float accelerationRequest = Track::driverModelVelocity(currentVelocity, accelerationLimit, decelerationLimit, headingRequest);
>>>>>>> 6ecbaa2cf598b286371412b2235889c653b1bc33

    opendlv::logic::action::AimPoint o1;
    o1.setAzimuthAngle(headingRequest);
    odcore::data::Container c1(o1);
    getConference().send(c1);
    std::cout << "[cognition] TRACK.CPP IS SENDING headingRequest = " << headingRequest << std::endl;
    if (accelerationRequest >= 0.0f) {
      opendlv::proxy::GroundAccelerationRequest o2;
      o2.setGroundAcceleration(accelerationRequest);
      odcore::data::Container c2(o2);
      getConference().send(c2);
      std::cout << "[cognition] TRACK.CPP IS SENDING GroundAccelerationRequest = " << accelerationRequest << std::endl;
    }
    else {
      opendlv::proxy::GroundDecelerationRequest o3;
      o3.setGroundDeceleration(accelerationRequest);
      odcore::data::Container c3(o3);
      getConference().send(c3);
      std::cout << "[cognition] TRACK.CPP IS SENDING GroundDecelerationRequest = " << accelerationRequest << std::endl;
    }
  }
  if (a_container.getDataType() == opendlv::system::SignalStatusMessage::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::system::SystemOperationState::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
  if (a_container.getDataType() == opendlv::system::NetworkStatusMessage::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }

  /*
    std::cout << "JAG LEVER!!!" << std::endl;
    ArrayXXf m_localPath(15,2);
    m_localPath <<
    -0.646636, -0.0466392,
    -0.682606,   0.452065,
    -0.716798,   0.950891,
    -0.725,    1.45075,
    -0.720662,    1.95071,
    -0.700775,    2.45029,
    -0.669754,    2.94932,
    -0.634384,    3.44802,
    -0.560804,    3.94058,
    -0.332111,    3.79835,
    -0.236519,    3.30803,
    -0.211409,    2.80907,
    -0.197508,    2.30927,
    -0.183607,    1.80946,
    -0.175,        1.5;

    float previewTime = 1;
    float currentVelocity = 5;
    float headingRequest = Track::driverModelSteering(currentVelocity, previewTime, m_localPath);
    float accelerationLimit = 5;
    float decelerationLimit = -5;
    float accelerationRequest = Track::driverModelVelocity(currentVelocity, accelerationLimit, decelerationLimit, headingRequest,m_localPath);

      std::cout << "headingRequest =" << headingRequest << std::endl;
      std::cout << "accelerationRequest =" << accelerationRequest << std::endl;
    // Send headingRequest



  std::cout << "a_container.getDataType() =" << a_container.getDataType() << std::endl;
  std::cout << "opendlv::logic::action::AimDirection::ID() =" << opendlv::logic::action::AimDirection::ID() << std::endl;
    if (a_container.getDataType() == 8) {
      auto HEADING = a_container.getData<opendlv::logic::action::AimDirection>();
      std::cout << "HEADING =" << HEADING << std::endl;
    }
    */

}

void Track::setUp()
{
  // std::string const exampleConfig =
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-cognition-track.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Track::tearDown()
{
}

float Track::driverModelSteering(float currentVelocity, float previewTime) {
  //driverModelSteering calculates the desired heading of CFSD18 vehicle
  // given the vehicles velocity, a set time to aimpoint and a path in local
  // coordinates (vehicle is origo).
  //
  //Input
  //   CURRENTVELOCITY         [1 x 1] Velocity of the vehicle [m/s]
  //   PREVIEWTIME             [1 x 1] Time to aimpoint [s].
  //   m_localPath               [n x 2] Local coordinates of the path [x,y]
  //
  //Output
  //   HEADINGREQUEST  [1 x 1] Desired heading angle [rad]

  // Calculate the distance between vehicle and aimpoint;
  float previewDistance = currentVelocity*previewTime;
  //Distance to aimpoint is currently calculated only on path, Not from vehicle
  float sumPoints = 0.0f;
  // Sum the distance between all path points until passing previewDistance
  // or reaching end of path
  int k=0;
  while (previewDistance >= sumPoints && k < m_localPath.rows()) {
    sumPoints += (m_localPath.row(k+1)-m_localPath.row(k)).matrix().norm();
    k++;
  }

    ArrayXXf aimPoint(1,2); //REMAKE AS VECTOR?
  if (sumPoints >= previewDistance) { // it means that the path is longer than previewDistance
    float distanceP1P2, overshoot, distanceP1AimPoint, percentage;
    if (k > 0) {// then the aimpoint will be placed after the first path element
      // Distance between last two considered path elements P1 P2 where P2 is the overshoot element.
      distanceP1P2 = (m_localPath.row(k+1)-m_localPath.row(k)).matrix().norm();
      // Difference between distance to aimpoint and summed points.
      overshoot = sumPoints - previewDistance;
      // Distance between next to last considered path element P1 and aimpoint
      distanceP1AimPoint = distanceP1P2 - overshoot;
      // Linear interpolation
      percentage = distanceP1AimPoint/distanceP1P2;
      aimPoint = (m_localPath.row(k+1)-m_localPath.row(k))*percentage + m_localPath.row(k);
    }
    else {// not needed if sumPoints is initialized as zero, (and previewDistance>0)

      distanceP1P2 = m_localPath.row(0).matrix().norm(); // Distance is equal to the distance to the first point;
      overshoot = sumPoints - previewDistance;
      distanceP1AimPoint = distanceP1P2 - overshoot;
      percentage = distanceP1AimPoint/distanceP1P2;
      aimPoint = m_localPath.row(0)*percentage;
    }
  }
  // If the path is too short, place aimpoint at the last path element
  else {
    aimPoint = m_localPath.row(m_localPath.rows()-1);
  }
  // Angle to aimpoint
  float headingRequest;
  headingRequest = atan2(aimPoint(1),aimPoint(0));
  std::cout << "headingRequest =" << headingRequest << std::endl;
  return headingRequest;
}

float Track::driverModelVelocity(float currentVelocity, float accelerationLimit, float decelerationLimit, float headingRequest){
  //driverModelVelocity calculates the desired acceleration of the CFSD18
  //vehicle.
  //
  //Input
  //   CURRENTVELOCITY             [1 x 1] Velocity of the vehicle [m/s]
  //   VELOCITYLIMIT               [1 x 1] Maximum allowed velocity [m/s]
  float velocityLimit = 10;
  //   LATERALACCELERATIONLIMIT    [1 x 1] Allowed lateral acceleration [m/s^2]
  float lateralAccelerationLimit = 2*9.81;
  //   ACCELERATIONLIMIT           [1 x 1] Allowed longitudinal acceleration (positive) [m/s^2]
  //   DECELERATIONLIMIT           [1 x 1] Allowed longitudinal acceleration (negative) [m/s^2]
  //   HEADINGREQUEST              [1 x 1] Heading error to aimpoint [rad]
  //   HEADINGERRORDEPENDENCY      [1 x 1] Constant
  float headingErrorDependency = 0; //
  //   PATH                        [n x 2] Local coordinates of the path [x,y]
  //
  //Output
  //   ACCELERATIONREQUEST         [1 x 1] Desired acceleration

  ArrayXXf curveRadii = curvature();

  // Set velocity candidate based on expected lateral acceleration limit
  ArrayXXf speedProfile(m_localPath.rows()-2,1);
  for (int k = 0; k < m_localPath.rows()-2; k++){
  speedProfile(k) = std::min(sqrtf(lateralAccelerationLimit*curveRadii(k)),velocityLimit);
  }

  std::cout << "curveRadii =" << curveRadii << std::endl;
  std::cout << "speedProfile before BT =" << speedProfile << std::endl;

  // Back propagate the whole path and lower velocities if deceleration cannot
  // be achieved.
  for (int k = speedProfile.rows()-1; k > 0 ; k-=1) {
    std::cout << "k =" << k << std::endl;
    // Distance between considered path points
    float pointDistance = (m_localPath.row(k+1)-m_localPath.row(k)).matrix().norm();
    std::cout << "pointDistance =" << pointDistance << std::endl;
    std::cout << "speedProfile(k) =" << speedProfile(k) << std::endl;
    // Time between points if using averaged velocity
    float timeBetweenPoints = pointDistance/((speedProfile(k)+speedProfile(k-1))/2); //Using the highest velocity is safer?
    std::cout << "timeBetweenPoints =" << timeBetweenPoints  << std::endl;
    // Requiered acceleration to achieve velocity of following point from previous point
    float requieredAcceleration = (speedProfile(k)-speedProfile(k-1))/timeBetweenPoints;
    // If deceleration is needed
    if (requieredAcceleration < 0.0f) {
      // If deceleration can't be achieved by braking...
      if (requieredAcceleration < decelerationLimit) {
        // Set deceleration to max achivable deceleration (vehicle specific)
        // Calculate new velocity
        speedProfile(k-1) = sqrtf(powf(speedProfile(k),2)-2.0f*decelerationLimit*pointDistance); // time based on average(v2-v1)/2
      }
    }
  }
  std::cout << "speedProfile after BT =" << speedProfile << std::endl;
  // Choose velocity to achieve
  // Limit it dependent on the heading request (heading error)
  float desiredVelocity = speedProfile(0)/(1.0f + headingErrorDependency*abs(headingRequest));
  // Calculate time to desired velocity
  float timeToVelocity = (m_localPath.row(1)).matrix().norm()/((currentVelocity+desiredVelocity)/2);
  // Transform into acceleration
  float accelerationRequest = (desiredVelocity-currentVelocity)/timeToVelocity;
  // Limit acceleration request for positive acceleration
  if (accelerationRequest > 0.0f && accelerationRequest > accelerationLimit) {
    accelerationRequest = accelerationLimit;
  }
  // Limit acceleration request for negative acceleration
  if (accelerationRequest < 0.0f && accelerationRequest < decelerationLimit) {
    accelerationRequest = decelerationLimit;
  }

  return accelerationRequest;
}

ArrayXXf Track::curvature(){
  // Segmentize the path and calculate radius of segments
  // - First radius is calculated at 2nd path point
  // - Last radius is calculated at 2nd to last path point
  // Note! 3 points in a row gives infinate radius.
  ArrayXXf curveRadii(m_localPath.rows()-2,1);
  for (int k = 0; k < m_localPath.rows()-2; k++) {
    // Choose three points and make a triangle with sides A(p1p2),B(p2p3),C(p1p3)
    float A = (m_localPath.row(k+1)-m_localPath.row(k)).matrix().norm();
    float B = (m_localPath.row(k+2)-m_localPath.row(k+1)).matrix().norm();
    float C = (m_localPath.row(k+2)-m_localPath.row(k)).matrix().norm();

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

    std::cout << "A =" << A << std::endl;
    std::cout << "B =" << B << std::endl;
    std::cout << "C =" << C << std::endl;
    if (C-(A-B) < 0) {
      std::cout << "WARNING! The data are not side-lengths of a real triangle" << std::endl;
      curveRadii(k) = 10000;
    }
    else {
      // https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
      // Calculate triangle area
      std::cout << "uttryck =" << (A+(B+C))*(C-(A-B))*(C+(A-B))*(A+(B-C))<< std::endl;
      float triangleArea = 0.25f*sqrtf((A+(B+C))*(C-(A-B))*(C+(A-B))*(A+(B-C)));
      std::cout << "triangleArea =" << triangleArea << std::endl;
      // Calculate the radius of the circle that matches the points
      curveRadii(k) = (A*B*C)/(4*triangleArea);
    }
  }
  return curveRadii;
}






}
}
}
}
