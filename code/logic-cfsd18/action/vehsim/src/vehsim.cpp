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
#include <fstream>
#include <string>
#include <sstream>


#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>


#include "vehsim.hpp"

#define PI 3.14159265

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {


Vehsim::Vehsim(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-vehsim"),
  m_aimPoint(),
  m_torqueRequest1(),
  m_torqueRequest2(),
  m_Fbrake(),
  m_brakeEnabled()
  {
}

Vehsim::~Vehsim()
{
}

void Vehsim::nextContainer(odcore::data::Container &a_container)
{
  auto kv = getKeyValueConfiguration();
  uint32_t leftMotorID = kv.getValue<uint32_t>("global.sender-stamp.left-motor");
  uint32_t rightMotorID = kv.getValue<uint32_t>("global.sender-stamp.right-motor");

  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    m_aimPoint = a_container.getData<opendlv::logic::action::AimPoint>();
  }

  if (a_container.getDataType() == opendlv::proxy::TorqueRequest::ID()) {
    m_brakeEnabled = false;
    if (a_container.getSenderStamp() ==  leftMotorID) {
      auto torqueContainer = a_container.getData<opendlv::proxy::TorqueRequest>();
      m_torqueRequest1 = torqueContainer.getTorque();
    } else if(a_container.getSenderStamp() == rightMotorID) {
      auto torqueContainer = a_container.getData<opendlv::proxy::TorqueRequest>();
      m_torqueRequest2 = torqueContainer.getTorque();
    }
  }

  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    m_brakeEnabled = true;
    auto decelContainer = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    m_Fbrake = decelContainer.getGroundDeceleration();
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Vehsim::body()
{
  auto kv = getKeyValueConfiguration();

  // Vehicle parameters
  float mass = kv.getValue<float>("global.vehicle-parameter.m");  // 0
  float Iz = kv.getValue<float>("global.vehicle-parameter.Iz");   // 1
  float L = kv.getValue<float>("global.vehicle-parameter.l");     // 4
  float lf = kv.getValue<float>("global.vehicle-parameter.lf");   // 5
  float lr = kv.getValue<float>("global.vehicle-parameter.lr");   // 6
  float wf = kv.getValue<float>("global.vehicle-parameter.wf");   // 7
  float wr = kv.getValue<float>("global.vehicle-parameter.wf");   // 8
  float Ca1 = kv.getValue<float>("global.vehicle-parameter.Ca1"); // 15
  float Ca2 = kv.getValue<float>("global.vehicle-parameter.Ca2"); // 16
  float Ku = (Ca2*lr-Ca1*lf)/(Ca1*Ca2*L);                         // 22

  // Store in vector to access in function


  // Simulation Specific stuff
  float sampleTime = 1/static_cast<float>(getFrequency());


  // States of the vehicle
  Eigen::ArrayXf x(6);
  Eigen::ArrayXf X(6);

  m_aimPoint = opendlv::logic::action::AimPoint(0.0f,0.0f,5.0f);

  // Tire data used for tire modelling
  Eigen::VectorXf tireLoad(15);
  Eigen::VectorXf tireSlip(50);
  Eigen::MatrixXf tireForce(50,15);

  std::ifstream file("/opt/opendlv.data/TireData.txt", std::ifstream::in );
  std::string row;
  std::string value;
  int irow = 0;
  int icol;


  if (file.is_open()){
    std::cout << "File open" << std::endl;
    while(file.good()){
      icol = 0;
      getline ( file, row);
      std::stringstream ss(row);
      while (ss >> value){
          if (irow == 0 && icol != 0){
            tireLoad(icol-1) = -1*std::stof(value);
          } else if (icol == 0 && irow != 0) {
            tireSlip(irow-1) = std::stof(value);
          } else if (icol != 0 && irow != 0){
            tireForce(irow-1,icol-1) = -1*std::stof(value);
          }
          icol ++;
      }
      irow ++;
    }
  }
  std::cout << "File read, data loaded" << std::endl;


  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

        float yawRef = 0.0f; //yawModel(m_aimPoint, x);
        Eigen::ArrayXf delta = calcSteerAngle(yawRef, x, mass, L, Ku);
        Eigen::ArrayXf Fz = loadTransfer(x, mass, L, lf, lr, wf, wr);
        Eigen::ArrayXf Fx = longitudinalControl(Fz, x, L, lf, lr);
        Eigen::ArrayXf Fy = tireModel(delta,x,Fz, lf, lr, wf, wr, tireLoad,
          tireSlip, tireForce);
        Eigen::ArrayXf dx = motion(delta,Fy,Fx,x, mass, Iz, lf, lr, wf, wr,
          sampleTime);


        X += x*sampleTime + dx*(float)pow(sampleTime,2)/2;
        x += dx*sampleTime;


        opendlv::sim::Frame out1(X(0),X(1),X(2),X(3),X(4),X(5));
        odcore::data::Container c1(out1);
        getConference().send(c1);

        std::cout << std::endl << X << std::endl;

        sendAccelerationRequest(yawRef, x);
    }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

float Vehsim::yawModel(opendlv::logic::action::AimPoint aimPoint, Eigen::ArrayXf x)
{
  float headingReq = aimPoint.getAzimuthAngle();
  float dist  = aimPoint.getDistance();
  float sgn   = (float) !(headingReq > 0);
  float u = x(0);

  float R = dist/(float)(sqrt(2.0f*(1.0f-(float)cos(2.0f*headingReq))));
  float r = sgn*u/R;

  return r;
}

Eigen::ArrayXf Vehsim::calcSteerAngle(float rRef, Eigen::ArrayXf x, float mass, float L, float Ku)
{
  float maxDelta = PI/6;
  float u = x(0);
  float deltaf = 0.0f;

  if(u>0.1f){
      deltaf = (L+Ku*mass*(float)pow(u,2.0f))*rRef/u;
  }

  deltaf = std::max<float>(std::min<float>(deltaf,maxDelta),-maxDelta);

  Eigen::ArrayXf delta(4);
  delta *= deltaf;

  return delta;
}

Eigen::ArrayXf Vehsim::loadTransfer(Eigen::ArrayXf x, float mass, float L,
  float lf, float lr, float wf, float wr)
{
  float u   = x(0);
  float ax  = x(3);
  float ay  = x(4);

  float const g = 9.81f;
  float const rho = 1.2f;
  float const Cl = 0.3f;
  float const Cd = 0.3f;
  float const h1 = 0.5f;
  float const h2 =0.4f;
  float const h =0.45f;
  float const hp = 0.1f;
  float const area = 1.0f;

  float Fl  = 1/2*area*rho*Cl*(float)pow(u,2.0f);
  float Fd  = 1/2*area*rho*Cd*(float)pow(u,2.0f);

  Eigen::ArrayXf weightDist(4);
  weightDist << lr, lr, lf, lf;
  weightDist /= (2.0f*L);

  Eigen::ArrayXf FzStatic = weightDist*(Fl + mass*g);

  //##################### dFz calculations
  Eigen::ArrayXf dFzy(4);
  dFzy << (-(mass*(-lr)*ay*h1/wf/L/2.0f)),
  ((mass*(-lr)*ay*h1/wf/L/2.0f)),
  (-(mass*(lf)*ay*h2/wr/L/2.0f)),
  ((mass*(lf)*ay*h2/wr/L/2.0f));

  Eigen::ArrayXf dFzx(4);
  dFzx << -mass*ax*h/2/L - Fd*hp/2.0f/L,
  -mass*ax*h/2.0f/L - Fd*hp/2.0f/L,
  +mass*ax*h/2.0f/L + Fd*hp/2.0f/L,
  +mass*ax*h/2.0f/L + Fd*hp/2.0f/L;


  Eigen::ArrayXf Fz = FzStatic+dFzy+dFzx;

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
  Eigen::VectorXf tireLoad, Eigen::VectorXf tireSlip, Eigen::MatrixXf tireForce)
{
  Eigen::ArrayXf Fy(4);

  float u = x(0);
  float v = x(1);
  float r = x(2);

  float a1, a2, a3, a4;

  if(abs(v)>1e-4){
    a1 = -atan2(v+lf*r,u+wf*r);
    a2 = -atan2(v+lf*r,u-wf*r);
    a3 = -atan2(v-lr*r,u+wr*r);
    a4 = -atan2(v-lr*r,u-wr*r);
  } else{
    a1 = 0;
    a2 = 0;
    a3 = 0;
    a4 = 0;
  }

  Eigen::ArrayXf alpha(4);
  alpha << a1, a2, a3, a4;
  alpha = alpha + delta;

  float Fy1 = interp2(tireLoad, tireSlip, tireForce, Fz(0), a1);
  float Fy2 = interp2(tireLoad, tireSlip, tireForce, Fz(1), a2);
  float Fy3 = interp2(tireLoad, tireSlip, tireForce, Fz(2), a3);
  float Fy4 = interp2(tireLoad, tireSlip, tireForce, Fz(3), a4);

  Fy << Fy1,Fy2,Fy3,Fy4;
  return Fy;
}


Eigen::ArrayXf Vehsim::longitudinalControl(Eigen::ArrayXf Fz, Eigen::ArrayXf states, float L, float lf, float lr)
{
  float u = states(0);
  (void) u;
  float const wheelRadius = 0.22f;

  Eigen::ArrayXf Fx(4);
  float tr1 = m_torqueRequest1;
  float tr2 = m_torqueRequest2;

  if (!m_brakeEnabled) {
    float fx1 = tr1/wheelRadius;
    float fx2 = tr2/wheelRadius;

    Fx << 0,0,fx1,fx2;
  } else if (m_brakeEnabled) {
    Fx << -lr, -lr, lf, lf;
    Fx /= L;
    Fx *= m_Fbrake;
  }


  (void) (Fz);
  return Fx;
}


Eigen::ArrayXf Vehsim::motion(Eigen::ArrayXf delta, Eigen::ArrayXf Fy,
  Eigen::ArrayXf Fx, Eigen::ArrayXf x, float mass, float Iz, float lf,
  float lr, float wf, float wr, float sampleTime)
{
  float u = x(0);
  float v = x(1);
  float r = x(2);

  Eigen::ArrayXf dx(6);
//################################
  float Ffly = Fy(0)*(float)cos(delta(0))+Fx(0)*(float)sin(delta(0));
  float Ffry = Fy(1)*(float)cos(delta(1))+Fx(1)*(float)sin(delta(1));
  float Frly = Fy(2)*(float)cos(delta(2))+Fx(2)*(float)sin(delta(2));
  float Frry = Fy(3)*(float)cos(delta(3))+Fx(3)*(float)sin(delta(3));

  float Fflx = Fx(0)*(float)cos(delta(0))-Fy(0)*(float)sin(delta(0));
  float Ffrx = Fx(1)*(float)cos(delta(1))-Fy(1)*(float)sin(delta(1));
  float Frlx = Fx(2)*(float)cos(delta(2))-Fy(2)*(float)sin(delta(2));
  float Frrx = Fx(3)*(float)cos(delta(3))-Fy(3)*(float)sin(delta(3));

  float du = (Ffrx+Fflx+Frlx+Frrx)/mass + r*v;
  float dv = (Ffry+Ffly+Frly+Frry)/mass - r*u;
  float dr = (lr*(Frry+Frly)+lf*(Ffry+Ffly)+wf*(Ffrx-Fflx)+wr*(Frrx-Frlx))/Iz;

  float ddu = (du - v*r - x(3))*(1/sampleTime);
  float ddv = (dv + u*r - x(4))*(1/sampleTime);
  float ddr = (dr - x(5))*(1/sampleTime);

  dx << du, dv, dr, ddu, ddv, ddr;
  return dx;
}



Eigen::ArrayXf Vehsim::atanArr(Eigen::ArrayXf a)
{
  Eigen::ArrayXf arrOut(a.size());
  for (int i = 0; i < a.size(); i++)
  {
    arrOut(i) = atan(a(i));
  }
  return arrOut;
}

void Vehsim::sendAccelerationRequest(float yawRef, Eigen::ArrayXf x)
{
  (void) yawRef;

  float u = x(0);
  float v_ref = 5; //std::max(u_min,v);
  float e = (v_ref - u);

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
{
  Eigen::RowVectorXf Y(2);
  Eigen::VectorXf X(2);
  Eigen::MatrixXf V(2,2);

  int i = 0;
  int j = 0;
  // Find betweem which two points x lies
  while (arg_xq < arg_X(i)) {
    if (i >= arg_X.size()-1) {
      std::cout << "Interp2: x-value out of bounds. -> " << arg_xq << std::endl;
      std::cout << "X max: " << arg_X(i-1) << std::endl;
      break;
    }
    i++;
  }
  // Find betweem which two points y lies
  while (arg_yq > arg_Y(j)) {
    if (j >= arg_Y.size()-1) {
      std::cout << "Interp2: y-value out of bounds" << std::endl;
      break;
    }
    j++;
  }

  X << (arg_X(i)-arg_xq)/(arg_X(i)-arg_X(i-1)), (arg_X(i-1)-arg_xq)/(arg_X(i-1)-arg_X(i));
  Y << (arg_Y(j)-arg_yq)/(arg_Y(j)-arg_Y(j-1)), (arg_Y(j-1)-arg_yq)/(arg_Y(j-1)-arg_Y(j));
  V << arg_V(j-1,i-1), arg_V(j-1,i),
       arg_V(j,i-1), arg_V(j,i);

  auto vq = Y*V*X;

  return vq;
}

void Vehsim::setUp()
{
  std::cout << "vehsim setup" << std::endl;
}

void Vehsim::tearDown()
{
}

}
}
}
}
