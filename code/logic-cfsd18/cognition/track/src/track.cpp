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
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-cognition-track")
{
}

Track::~Track()
{
}



void Track::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::logic::perception::Surface::ID()) {
     auto surface = a_container.getData<opendlv::logic::perception::Surface>();

     std::string type = surface.getType();

    opendlv::logic::action::AimPoint o1;
    o1.setDistance(4.5);
    odcore::data::Container c1(o1);
    getConference().send(c1);

    opendlv::logic::action::PreviewPoint o2;
    odcore::data::Container c2(o2);
    getConference().send(c2);

    opendlv::logic::cognition::GroundSpeedLimit o3;
    odcore::data::Container c3(o3);
    getConference().send(c3);
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

float Track::driverModelSteering(float currentVelocity, float timeToAimPoint, ArrayXXf localPath) {
  //driverModelSteering calculates the desired heading of CFSD18 vehicle
  // given the vehicles velocity, a set time to aimpoint and a path in local
  // coordinates (vehicle is origo).
  //
  //Input
  //   CURRENTVELOCITY         [1 x 1] Velocity of the vehicle [m/s]
  //   TIMETOAIMPOINT          [1 x 1] Time to aimpoint [s].
  //   LOCALPATH               [n x 2] Local coordinates of the path [x,y]
  //
  //Output
  //   HEADINGREQUEST  [1 x 1] Desired heading angle [rad]

  // Calculate distance to aimpoint;
  float distanceToAimPoint = currentVelocity*timeToAimPoint;
  // Initial distance considered(0 or distance between vehicle and first path point)
  float sumPoints = 0.0f; //norm(localPath(1,:)); // Distance to aimpoint is calculated only on path OR from vehicle
  // Sum the distance between all path points until passing distanceToAimPoint
  // or reaching end of path
  int k=0;
  while (distanceToAimPoint >= sumPoints && k < localPath.rows())
  {
    sumPoints += (localPath.row(k+1)-localPath.row(k)).matrix().norm();
    k++;
  }

    ArrayXXf aimPoint(1,2); //REMAKE AS VECTOR
  if (sumPoints >= distanceToAimPoint) // The path is longer than distanceToAimpoint
  {
    float distanceP1P2, overshoot, distanceP1AimPoint, percentage;
    if (k > 0) // The aimpoint will be placed after the first path element
    {
      // Distance between last two considered path elements P1 P2 where P2 is the overshoot element.
      distanceP1P2 = (localPath.row(k+1)-localPath.row(k)).matrix().norm();
      // Difference between distance to aimpoint and summed points.
      overshoot = sumPoints - distanceToAimPoint;
      // Distance between next to last considered path element P1 and aimpoint
      distanceP1AimPoint = distanceP1P2 - overshoot;
      // Linear interpolation
      percentage = distanceP1AimPoint/distanceP1P2;
      aimPoint = (localPath.row(k+1)-localPath.row(k))*percentage + localPath.row(k);
    }
    else // not needed if sumPoints is initialized as zero, (and distanceToAimpoint>0)
    {
      distanceP1P2 = localPath.row(0).matrix().norm(); // Distance is equal to the distance to the first point;
      overshoot = sumPoints - distanceToAimPoint;
      distanceP1AimPoint = distanceP1P2 - overshoot;
      percentage = distanceP1AimPoint/distanceP1P2;
      aimPoint = localPath.row(0)*percentage;
    }
  }
  else // If the path is too short, place aimpoint at last path point
  {
    aimPoint = localPath.row(localPath.rows()-1);
  }
  // Angle to aimpoint
  float headingRequest;
  headingRequest = atan2(aimPoint(1),aimPoint(0));

  return headingRequest;
}

float Track::driverModelVelocity(float currentVelocity, float accelerationLimit, float decelerationLimit, float headingRequest, ArrayXXf localPath)
{
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
  ////
  // Segmentize the path and calculate radius of segments
  // - First radius is calculated at 2nd path point
  // - Last radius is calculated at 2nd to last path point
  // Note! 3 points in a row gives infinate radius.

  ArrayXXf curveRadii(localPath.rows()-2,1);
  ArrayXXf velocityCandidate(localPath.rows()-2,1);
  for (int k = 0; k < localPath.rows()-2; k++)
  {
    // Choose three points and make a triangle with sides
    // A(p1p2),B(p2p3),C(p1p3)
    float A = (localPath.row(k+1)-localPath.row(k)).matrix().norm();
    float B = (localPath.row(k+2)-localPath.row(k+1)).matrix().norm();
    float C = (localPath.row(k+2)-localPath.row(k)).matrix().norm();
    // Calculate triangle area
    float S = (A+B+C)/2; // Heron's formula
    float triangleArea = sqrtf(S*(S-A)*(S-B)*(S-C));

    float tol = 0.01;
    // Infinate radius check - Check if needed in C++! (not needed in MatLab that can handle inf)
    if (triangleArea < tol)
    {
      curveRadii(k) = 10000; // Large value
    }
    else
    {
      // Calculate the radius of the circle that matches the points
      curveRadii(k) = (A*B*C)/(4*triangleArea);
    }
    // Set velocity candidate based on expected lateral acceleration
    //(remember to add safe zone)
    velocityCandidate(k) = std::min(sqrtf(lateralAccelerationLimit*curveRadii(k)),velocityLimit);
  }

  // Back propagate the whole path and lower velocities if deceleration cannot
  // be achieved.
  for (int k = velocityCandidate.rows(); k > 1 ; k-=1)
  {
    // Distance between considered path points
    float pointDistance = (localPath.row(k+1)-localPath.row(k)).matrix().norm();
    // Time between points if using averaged velocity
    float timeBetweenPoints = pointDistance/((velocityCandidate(k)+velocityCandidate(k-1))/2); //Using the highest velocity is safer?
    // Requiered acceleration to achieve velocity of following point from previous point
    float requieredAcceleration = (velocityCandidate(k)-velocityCandidate(k-1))/timeBetweenPoints;
    // If braking is needed
    if (requieredAcceleration < 0)
    {
      // If acceleration can't be achieved by braking
      if (requieredAcceleration < decelerationLimit)
      {
        // Set acceleration to maxBrakingAcceleration
        float modifiedDeceleration = decelerationLimit;
        // Calculate new velocity
        velocityCandidate(k-1) = sqrtf(powf(velocityCandidate(k),2)-2*modifiedDeceleration*pointDistance); // time based on average(v2-v1)/2
      }
    }
  }

  // Choose velocity to achieve
  //int velocityAimpoint = 1; //On which path point to base acceleration, min 1
  // Limit it dependent on the heading request (heading error)
  float desiredVelocity = velocityCandidate(0)/(1 + headingErrorDependency*abs(headingRequest));

  // Calculate time to desired velocity
  float timeToVelocity = (localPath.row(1)).matrix().norm()/((currentVelocity+desiredVelocity)/2);

  // Transform into acceleration
  float accelerationRequest = (desiredVelocity-currentVelocity)/timeToVelocity;

  // Limit acceleration request for positive acceleration
  if (accelerationRequest > 0 && accelerationRequest > accelerationLimit)
  {
    accelerationRequest = accelerationLimit;
  }
  // Limit acceleration request for negative acceleration
  if (accelerationRequest < 0 && accelerationRequest < decelerationLimit)
  {
    accelerationRequest = decelerationLimit;
  }
  return accelerationRequest;
}

}
}
}
}
