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
#include <math.h>
#include <chrono>
#include <string>
#include <sstream>


#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>


#include "vehsim.hpp"

#define PI 3.14159265f

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {


Vehsim::Vehsim(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-vehsim"),
  m_aimPoint(),
  m_torqueRequest1(),
  m_torqueRequest2(),
  m_deceleration(),
  m_brakeEnabled(),
  m_delta()
  //m_outputData()
  {
  }

Vehsim::~Vehsim()
{
}

void Vehsim::nextContainer(odcore::data::Container &a_container)
{
  auto kv = getKeyValueConfiguration();
  // Load sender stamps for left and right motor
  uint32_t leftMotorID = kv.getValue<uint32_t>("global.sender-stamp.left-motor");
  uint32_t rightMotorID = kv.getValue<uint32_t>("global.sender-stamp.right-motor");

  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    // Get the heading request. Change to use steering output instead
    m_aimPoint = a_container.getData<opendlv::logic::action::AimPoint>();
  }

  if (a_container.getDataType() == opendlv::proxy::TorqueRequest::ID()) {
    m_brakeEnabled = false; // Brakes are now disengaged
    // store the requested torque. 1 is left, 2 is right.
    if (a_container.getSenderStamp() ==  leftMotorID) {
      auto torqueContainer = a_container.getData<opendlv::proxy::TorqueRequest>();
      m_torqueRequest1 = torqueContainer.getTorque();
    } else if(a_container.getSenderStamp() == rightMotorID) {
      auto torqueContainer = a_container.getData<opendlv::proxy::TorqueRequest>();
      m_torqueRequest2 = torqueContainer.getTorque();
    }
  }

  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    m_brakeEnabled = true; // Engage Brakes
    // Store the requested deceleration. This will be changed to match brake output
    auto decelContainer = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    m_deceleration = decelContainer.getGroundDeceleration();
  }

  if (a_container.getDataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
    auto deltaContainer = a_container.getData<opendlv::proxy::GroundSteeringRequest>();
    m_delta = deltaContainer.getGroundSteering();
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Vehsim::body()
{
  auto kv = getKeyValueConfiguration();

  // Vehicle parameters
  float mass = kv.getValue<float>("global.vehicle-parameter.m");  // Vehicle mass
  float Iz = kv.getValue<float>("global.vehicle-parameter.Iz");   // Inertia about z-axis
  float L = kv.getValue<float>("global.vehicle-parameter.l");     // Wheelbase
  float lf = kv.getValue<float>("global.vehicle-parameter.lf");   // Distance from CoG to front axle
  float lr = kv.getValue<float>("global.vehicle-parameter.lr");   // Distance from CoG to rear axle
  float wf = kv.getValue<float>("global.vehicle-parameter.wf");   // Track width front
  float wr = kv.getValue<float>("global.vehicle-parameter.wr");   // Track width rear

  // Simulation Specific stuff
  float sampleTime = 1/static_cast<double>(getFrequency()); //1/static_cast<float>(getFrequency());
  // use second part to run real time

  // States of the vehicle
  float u0 = 5.0f; // initial speed
  // Initial velocities
  Eigen::ArrayXf x(6); x << u0,0,0,0,0,0;
  // Initial position
  Eigen::ArrayXf X(6); X << 0,0,0,0,0,0;
  // All wheel speeds equal to free rolling
  Eigen::ArrayXf omega(4); omega << 0,0,0,0;
  omega += u0/0.22f;

  Eigen::ArrayXf motorTorque(2); motorTorque << 0,0;

  std::cout << "Initializing states." << std::endl;

  // Lateral tire data used for tire modelling
  Eigen::VectorXf tireLoad(15);
  Eigen::VectorXf tireSlipY(50);
  Eigen::MatrixXf tireForceY(50,15);

  // Longitudinal tire data
  Eigen::VectorXf tireSlipX(20);
  Eigen::MatrixXf tireForceX(20,15);

  // Read experimental data from text file and make a look up table
  std::ifstream fileY("/opt/opendlv.data/TireDataY", std::ifstream::in );
  std::ifstream fileX("/opt/opendlv.data/TireDataX", std::ifstream::in );
  std::string row;
  std::string value;
  int irow = 0;
  int icol;

  // Lateral
  if (fileY.is_open()){
    while(fileY.good()){
      icol = 0;
      getline ( fileY, row);
      std::stringstream ss(row);
      while (ss >> value){
          if (irow == 0 && icol != 0){
            tireLoad(icol-1) = -1*std::stof(value);
          } else if (icol == 0 && irow != 0) {
            tireSlipY(irow-1) = std::stof(value)*PI/180;
          } else if (icol != 0 && irow != 0){
            tireForceY(irow-1,icol-1) = -1*std::stof(value);
          }
          icol ++;
      }
      irow ++;
    }
  }
  irow = 0;

  // Longitudinal
  if (fileX.is_open()){
    while(fileX.good()){
      icol = 0;
      getline ( fileX, row);
      std::stringstream ss(row);
      while (ss >> value){
          if (irow == 0 && icol != 0){
            //tireLoad(icol-1) = -1*std::stof(value);
          } else if (icol == 0 && irow != 0) {
            tireSlipX(irow-1) = std::stof(value);
          } else if (icol != 0 && irow != 0){
            tireForceX(irow-1,icol-1) = 1*std::stof(value);
          }
          icol ++;
      }
      irow ++;
    }
  }

  // Initialize a simulation clock
  float timer = 0.0f;

  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {
        // crate an optimal yawrate based on bicylce model
        float yawRef = 0.0f; // yawModel(m_aimPoint, x);
        // Calculate steering angle based on optimal yawrate
        Eigen::Array4f delta(m_delta,m_delta,0,0);
        // delta << 0.1f,0.1f,0,0;
        // Calculate vertical load on each wheel
        Eigen::ArrayXf Fz = loadTransfer(x, mass, L, lf, lr, wf, wr);
        // Model motor response
        motorTorque = motorModel(sampleTime,motorTorque,omega);
        // Calculate the Fx and the speed of the wheels
        Eigen::ArrayXf Fx = longitudinalControl(Fz, x, tireLoad, tireSlipX,
          tireForceX, &omega, motorTorque, sampleTime);
        // Calculate lateral forces
        Eigen::ArrayXf Fy = tireModel(delta,x,Fz, lf, lr, wf, wr, tireLoad,
          tireSlipY, tireForceY);
        // Calculate the accelerations of the vehicle by force equilibriums
        Eigen::ArrayXf dx = motion(delta,Fy,Fx,x, mass, Iz, lf, lr, wf, wr,
          sampleTime);

        // Calculate the rotation of the vehicle by integrating and assuming constant accelerations
        X(2) = X(2) + x(2)*sampleTime + dx(2)*(float)pow(sampleTime,2)/2;

        // Integrate and rotate positions to global frame
        X(0) = X(0) + (std::cos(X(2))*x(0) - std::sin(X(2))*x(1))*sampleTime + (std::cos(X(2))*dx(0) - std::sin(X(2))*dx(1))*(float)pow(sampleTime,2)/2;
        X(1) = X(1) + (std::sin(X(2))*x(0) + std::cos(X(2))*x(1))*sampleTime + (std::sin(X(2))*dx(0) + std::cos(X(2))*dx(1))*(float)pow(sampleTime,2)/2;
        X.tail(3) = x.head(3);

        // Integrate velocities
        x += dx*sampleTime;

        // send vehicle states
        //opendlv::sim::Frame outPos(X(0),X(1),0,0,0,X(2));
        opendlv::sim::KinematicState outVel(x(0),x(1),0,0,0,x(2));
        //odcore::data::Container posC(outPos);
        odcore::data::Container velC(outVel);
        velC.setSenderStamp(0);
        //getConference().send(posC);
        getConference().send(velC);

        float averageSpeed = x(0);
        opendlv::proxy::GroundSpeedReading outSpeed(averageSpeed);
        odcore::data::Container speedC(outSpeed);
        getConference().send(speedC);

        // send acceleration requests
        sendAccelerationRequest(yawRef, x);

        // update simulation clock

        // if (m_outputData.is_open()) {
        //   m_outputData << timer << " " << X.transpose() << std::endl;
        //   m_outputData.flush();
        // }
        timer += sampleTime;
    }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

float Vehsim::yawModel(opendlv::logic::action::AimPoint aimPoint, Eigen::ArrayXf x)
{

  float headingReq = aimPoint.getAzimuthAngle();  // Angle to the aim point
  float dist  = aimPoint.getDistance();           // Distance to the aim point
  float u = x(0);                                 // Vehicle speed

  // Approximate a curvature between vehicle position and the aim point
  float R = dist/(float)(sqrt(2.0f*(1.0f-(float)cos(2.0f*headingReq))));
  // Calculate the average yaw rate to turn for that specific curve
  float r = std::copysign(u/R,headingReq);

  return r;
}

// Eigen::ArrayXf Vehsim::calcSteerAngle(float rRef, Eigen::ArrayXf x, float mass, float L, float Ku)
// {
//   float maxDelta = PI/6;      // Define a maximum steering output
//   float u = x(0);             // Vehicle speed
//   float deltaf = 0.0f;        // pre-allocate delta
//
//   if(u>0.1f){ // if the speed is low, dont try to steer
//       // calculate average steer angle based on bicycle model and the reference yaw rate
//       deltaf = (L+Ku*mass*(float)pow(u,2.0f))*rRef/u;
//   }
//
//   deltaf = 0.1f;    // Overwrite delta to make specific test case, comment this out later
//   // saturate steer angle based on maximum allowed value
//   deltaf = std::max<float>(std::min<float>(deltaf,maxDelta),-maxDelta);
//
//   // Calculate steer angle for each wheel
//   Eigen::ArrayXf delta(4); delta << 1,1,0,0; // decide which wheels steer
//   delta *= deltaf;
//
//   return delta;
// }

Eigen::ArrayXf Vehsim::loadTransfer(Eigen::ArrayXf x, float mass, float L,
  float lf, float lr, float wf, float wr)
{
  float u   = x(0);   // Longitudinal speed
  float ax  = x(3);   // Longitudinal acceleration
  float ay  = x(4);   // Lateral acceleration

  float const g = 9.81f;      // Gravitational constant
  float const rho = 1.1842f;  // Density of air
  float const Cl = 1.698f;    // Lift coefficient
  float const Cd = 1.21f;     // Drag coefficient
  float const h1 = 0.04f;     // Roll center, front
  float const h2 =0.093f;     // Roll center, rear
  float const h =0.282f;      // CoG height
  float const hp = 0.432f;    // Height at which center of pressure acts (aero dynamic related)
  float const area = 1.14f;   // frontal area of the vehicle


  float Fl  = 0.5f*area*rho*Cl*(float)pow(u,2.0f); // Lift force
  float Fd  = 0.5f*area*rho*Cd*(float)pow(u,2.0f); // Drag force

  Eigen::ArrayXf weightDist(4);     // static weight distribution
  weightDist << lr, lr, lf, lf;
  weightDist /= (2.0f*L);

  Eigen::ArrayXf FzStatic = weightDist*(Fl + mass*g); // Static load (including lift)

  //##################### dFz calculations
  // Lateral load transfer:
  // Assumes steady state (springs and dampers have no effect).
  Eigen::ArrayXf dFzy(4);
  dFzy << -2.0f*mass*(weightDist(0)+weightDist(1))*ay*h1/wf,
          +2.0f*mass*(weightDist(0)+weightDist(1))*ay*h1/wf,
          -2.0f*mass*(weightDist(2)+weightDist(3))*ay*h2/wr,
          +2.0f*mass*(weightDist(2)+weightDist(3))*ay*h2/wr;

  // Longitudinal load transfer
  // neglect pitch dynamics
  Eigen::ArrayXf dFzx(4);
  dFzx << -mass*ax*h/2/L - Fd*hp/2.0f/L,
          -mass*ax*h/2.0f/L - Fd*hp/2.0f/L,
          +mass*ax*h/2.0f/L + Fd*hp/2.0f/L,
          +mass*ax*h/2.0f/L + Fd*hp/2.0f/L;

  // Total vertical load on wheels
  Eigen::ArrayXf Fz = FzStatic+dFzy+dFzx;

  // Additional step to make sure no wheels are lifting
  // This is a bad model if wheels lift
  float eps = 1e-3;
  for (int i=0; i < Fz.size(); i++)
  {
    if (Fz(i) < 0)
    {
      Fz(i) = eps;
    }
  }

  return Fz;
}

Eigen::ArrayXf Vehsim::tireModel(Eigen::ArrayXf delta, Eigen::ArrayXf x,
  Eigen::ArrayXf Fz, float lf, float lr, float wf, float wr,
  Eigen::VectorXf tireLoad, Eigen::VectorXf tireSlipY, Eigen::MatrixXf tireForceY)
{
  Eigen::ArrayXf Fy(4);  // Lateral force on wheels

  float u = x(0);   // Vehicle speed
  float v = x(1);   // Lateral speed
  float r = x(2);   // Yaw rate

  float a1, a2, a3, a4; // Wheel slip angles

  // This will cause problems when denominator is 0. Expetion for speed = 0 is neccessary
  // if(abs(v)>1e-20){
    a1 = -atan2(v+lf*r,u+wf*r/2);
    a2 = -atan2(v+lf*r,u-wf*r/2);
    a3 = -atan2(v-lr*r,u+wr*r/2);
    a4 = -atan2(v-lr*r,u-wr*r/2);
  // } else{
  //   a1 = 0;
  //   a2 = 0;
  //   a3 = 0;
  //   a4 = 0;
  // }

  Eigen::ArrayXf alpha(4);
  alpha << a1, a2, a3, a4;
  alpha = alpha + delta;

  // Calculate forces from look up table. This data has been recorded from the specific tires used.
  // However, the friction in the test environment was extremely high (~2.5-3)
  float Fy1 = interp2(tireLoad, tireSlipY, tireForceY, Fz(0)+2000, alpha(0));
  float Fy2 = interp2(tireLoad, tireSlipY, tireForceY, Fz(1), alpha(1));
  float Fy3 = interp2(tireLoad, tireSlipY, tireForceY, Fz(2), alpha(2));
  float Fy4 = interp2(tireLoad, tireSlipY, tireForceY, Fz(3), alpha(3));

  Fy << Fy1,Fy2,Fy3,Fy4;

  return Fy;
}

Eigen::ArrayXf Vehsim::motorModel(float sampleTime,
  Eigen::ArrayXf Torque, Eigen::ArrayXf omega)
  {
    float gearRatio = 16;
    float maxPower = 38e3;
    Eigen::ArrayXf motorSpeed = omega.tail(2)*gearRatio;

    float dtMax = 50*sampleTime;
    Eigen::VectorXf torqueRequest(2);
    torqueRequest << m_torqueRequest1, m_torqueRequest2;

    for (int i=0;i<2;i++){
      float torqueError = torqueRequest(i)-Torque(i);
      Torque(i) = Torque(i) + std::min(std::max(torqueError,-dtMax),dtMax);
      float maxTorque;


      if (std::fabs(motorSpeed(i)) < 11000*PI/30){
        maxTorque = 24;
      } else if (std::fabs(motorSpeed(i)) > 20000*PI/30){
        maxTorque = 0;
      } else {
        maxTorque = maxPower/std::fabs(motorSpeed(i));
      }
      Torque(i) = std::max(std::min(Torque(i),maxTorque),-maxTorque);
      }
  return Torque;
}

Eigen::ArrayXf Vehsim::longitudinalControl(Eigen::ArrayXf Fz, Eigen::ArrayXf x,
  Eigen::VectorXf tireLoad, Eigen::VectorXf tireSlipX,
  Eigen::MatrixXf tireForceX, Eigen::ArrayXf *wheelSpeed, Eigen::ArrayXf motorTorque,
  float sampleTime) {

  float u = x(0);                   // Vehicle speed
  float const wheelRadius = 0.22f;  // Radius of tires
  float const Iwy = 2.0f;           // Inertia of a wheel
  Eigen::ArrayXf omega = *wheelSpeed; // Wheel rotational speed


  Eigen::ArrayXf brakeDist(4);      // Brake distribution. This is tunable in the car, this is a normal value though.
  brakeDist << 0.125, 0.125, 0.375, 0.375;

  Eigen::ArrayXf Fx(4);             // Initialize odsupercomponent arrays
  Eigen::ArrayXf wheelSlip(4);
  Eigen::ArrayXf omegaDot(4);

  float tr1 = motorTorque(0);    // Get most recent torque requests recieved
  float tr2 = motorTorque(1);    // These can be negative! (Still only from motors, not brakes)

  Eigen::ArrayXf FxBrakes(4); FxBrakes << 0,0,0,0;
  // Initialize brakes forces, and assign values if brakes are enabled
  if (m_brakeEnabled) {
    FxBrakes = m_deceleration*brakeDist;
  }

  if (abs(u) < 1e-4) {          // Exception if u = 0
    for (int i=0; i<wheelSlip.size();i++){
      if (abs(omega(i))<1e-10){
        wheelSlip(i) = 0;
      } else {
        wheelSlip(i) = std::copysign(tireSlipX(0),omega(i));
      }
    }
  } else {
    // Calculate the Longitudinal wheel slip [-] (Not an angle!)
    wheelSlip = (omega*wheelRadius-u)/std::abs(u);
  }

  // Use look up tables to calculate what force the slip corresponds to
  float Fx1 = interp2(tireLoad,tireSlipX,tireForceX,Fz(0),wheelSlip(0));
  float Fx2 = interp2(tireLoad,tireSlipX,tireForceX,Fz(1),wheelSlip(1));
  float Fx3 = interp2(tireLoad,tireSlipX,tireForceX,Fz(2),wheelSlip(2));
  float Fx4 = interp2(tireLoad,tireSlipX,tireForceX,Fz(3),wheelSlip(3));

  Fx << Fx1,Fx2,Fx3,Fx4; // This is the force between tire and road


  // Moment equilibriums for wheels. Calculates the rotational acceleration of the wheels
  omegaDot(0) = wheelRadius*(FxBrakes(0)-Fx(0))/Iwy;
  omegaDot(1) = wheelRadius*(FxBrakes(1)-Fx(1))/Iwy;
  omegaDot(2) = (tr1 + wheelRadius*(FxBrakes(2)-Fx(2)))/Iwy;
  omegaDot(3) = (tr2 + wheelRadius*(FxBrakes(3)-Fx(3)))/Iwy;

  // Calculate the wheel speed. This is neccessary to calculate slip in the next iteration
  *wheelSpeed = omega + omegaDot*sampleTime;

  return Fx;
}


Eigen::ArrayXf Vehsim::motion(Eigen::ArrayXf delta, Eigen::ArrayXf Fy,
  Eigen::ArrayXf Fx, Eigen::ArrayXf x, float mass, float Iz, float lf,
  float lr, float wf, float wr, float sampleTime)
{
  float u = x(0);   // Vehicle speed
  float v = x(1);   // lateral speed
  float r = x(2);   // yaw rate
  float g = 9.81;   // Gravity constant
  float fr = 0.01;  // Rolling res. coefficient
  float const area = 1.14f;   // frontal area of the vehicle
  float const Cd = 1.21f;     // Drag coefficient
  float const rho = 1.1842f;  // Density of air

  Eigen::ArrayXf dx(6);
//################################
  // Fy and Fx where calculate in the tires coordinate frame,
  // this section converts to the vehicles coordinate system
  float Ffly = Fy(0)*(float)cos(delta(0))+Fx(0)*(float)sin(delta(0)); // Force,front,left,y
  float Ffry = Fy(1)*(float)cos(delta(1))+Fx(1)*(float)sin(delta(1)); // Force,front,right,y
  float Frly = Fy(2)*(float)cos(delta(2))+Fx(2)*(float)sin(delta(2)); // Force,rear,left,y
  float Frry = Fy(3)*(float)cos(delta(3))+Fx(3)*(float)sin(delta(3)); // Force,rear,right,y

  // Same index convetion as above
  float Fflx = Fx(0)*(float)cos(delta(0))-Fy(0)*(float)sin(delta(0));
  float Ffrx = Fx(1)*(float)cos(delta(1))-Fy(1)*(float)sin(delta(1));
  float Frlx = Fx(2)*(float)cos(delta(2))-Fy(2)*(float)sin(delta(2));
  float Frrx = Fx(3)*(float)cos(delta(3))-Fy(3)*(float)sin(delta(3));

  // Air and tire resistance
  float FxRes = mass*g*fr + 1/2*rho*Cd*area*static_cast<float>(pow(u,2));
  //#########################

  // sum(Fx) = m*ax = m*(VxDot - yawrate*Vy)
  float du = (Ffrx+Fflx+Frlx+Frrx-FxRes)/mass + r*v;
  // sum(Fy) = m*ax = m*(VxDot + yawrate*Vy)
  float dv = (Ffry+Ffly+Frly+Frry)/mass - r*u;
  // sum(Mz) = Izz*yawAcceleration
  float dr = (-lr*(Frry+Frly)+lf*(Ffry+Ffly)+wf/2*(Ffrx-Fflx)+wr/2*(Frrx-Frlx))/Iz;

  // Extra states that were neccessary in simulink.
  // These are obsolete in this model and will be removed
  float ddu = (du - v*r - x(3))*(1/sampleTime);
  float ddv = (dv + u*r - x(4))*(1/sampleTime);
  float ddr = (dr - x(5))*(1/sampleTime);

  dx << du, dv, dr, ddu, ddv, ddr;
  return dx;
}



Eigen::ArrayXf Vehsim::atanArr(Eigen::ArrayXf a)
{ // Function to calculate atan of a vector/array. Currently not used, remove?
  Eigen::ArrayXf arrOut(a.size());
  for (int i = 0; i < a.size(); i++)
  {
    arrOut(i) = atan(a(i));
  }
  return arrOut;
}

void Vehsim::sendAccelerationRequest(float yawRef, Eigen::ArrayXf x)
{ // Send out the acceleration request. This would normally be done by driver model, but this is used for now.
  (void) yawRef;
  float u = x(0);         // vehicle speed
  float v_ref = 5;        // desired speed
  float Kp = 1;           // control parameter
  float e = Kp*(v_ref - u);  // control error

  if(e > 0) {
    opendlv::proxy::GroundAccelerationRequest out1(e);
    odcore::data::Container c1(out1);
    getConference().send(c1);
  } else {
    opendlv::proxy::GroundDecelerationRequest out2(e);
    odcore::data::Container c2(out2);
    getConference().send(c2);
  }

}

float Vehsim::interp2(Eigen::VectorXf arg_X, Eigen::VectorXf arg_Y, Eigen::MatrixXf arg_V, float arg_xq, float arg_yq)
{   // 2-d interpolation function. Only works if arg_X is a vector in
    // decending order and arg_Y in ascending order.. (working on chage)
    // arg_V is the data with arg_X.size() columns and arg_Y.size() rows.
    // arg_xq and arg_yq is at which point data is requested.
    // Currently no solution if arg_xq and arg_yq are not within arg_X and arg_Y
  Eigen::VectorXf XY(4);
  Eigen::VectorXf b(4);
  Eigen::MatrixXf coeffs(4,4);
  Eigen::VectorXf V(4);

  int i = 1;
  int j = 1;
  // Find betweem which two points x lies
  while (arg_xq <= arg_X(i)) {
    if (i >= arg_X.size()-1) {
      // Exception if xq was not found in X
      std::cout << "Interp2: x-value out of bounds. -> " << arg_xq << std::endl;
      std::cout << "X max: " << arg_X(i-1) << std::endl;
      break;
    }
    i++; // I will decide between which two points in X, xq is
  }
  // Find betweem which two points y lies
  while (arg_yq >= arg_Y(j)) {
    if (j >= arg_Y.size()-1) {
      // Exception, same as X.
      // std::cout << "Interp2: y-value out of bounds" << std::endl;
      j = 0;
      arg_yq = std::copysign(arg_Y(j),arg_yq);
    }
    j++; // same as i, but in Y vector
  }

  // X and Y are weights. V are the four closest data points to V(xq,yq)
  XY << 1, arg_xq, arg_yq, arg_xq*arg_yq;
  coeffs << 1, arg_X(i-1), arg_Y(j-1), arg_X(i-1)*arg_Y(j-1),
            1, arg_X(i-1), arg_Y(j), arg_X(i-1)*arg_Y(j),
            1, arg_X(i), arg_Y(j-1), arg_X(i)*arg_Y(j-1),
            1, arg_X(i), arg_Y(j), arg_X(i)*arg_Y(j);

  V << arg_V(j-1,i-1), arg_V(j,i-1), arg_V(j-1,i), arg_V(j,i);

  b = coeffs.inverse().transpose()*XY;
  float vq = b.transpose()*V;
  return vq;
}

void Vehsim::setUp()
{
  std::cout << "vehsim setup" << std::endl;

  // m_outputData.open("/opt/opendlv.data/outputData",std::ofstream::out);
  //
  // if (m_outputData.is_open()) {
  //   m_outputData << "% time\t" << "X\t" << "Y\t" << "Psi\t" << "u\t" << "v\t" << "r" << std::endl;
  // }

  // add aim point infront of the car as initializer
  m_aimPoint = opendlv::logic::action::AimPoint(0.0f,0.0f,5.0f);
}

void Vehsim::tearDown()
{
//  m_outputData.close();
}

}
}
}
}
