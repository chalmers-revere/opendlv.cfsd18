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
  m_mass(),
  m_mus(),
  m_ms(),
  m_Iz(),
  m_Ix(),
  m_g(),
  m_L(),
  m_lf(),
  m_lr(),
  m_wf(),
  m_wr(),
  m_mu(),
  m_wR(),
  m_Ku(),
  m_A(),
  m_rho(),
  m_Cl(),
  m_Cd(),
  m_h0(),
  m_h1(),
  m_h2(),
  m_h(),
  m_hp(),
  m_Ca1(),
  m_Ca2(),
  m_kPhi(),
  m_cPhi(),
  m_kPhi1(),
  m_cPhi1(),
  m_kPhi2(),
  m_cPhi2(),
  m_kLambda(),
  m_cLambda(),
  m_phi(),
  m_phiDot(),
  m_wheelLiftOffFlag(),
  m_B(),
  m_C(),
  m_D(),
  m_E(),
  m_x(),
  m_X(),
  m_sampleTime(),
  m_torqueRequest1(),
  m_torqueRequest2(),
  m_Fbrake(),
  m_leftMotorID(),
  m_rightMotorID(),
  m_brakeEnabled(),
  m_tireLoad(),
  m_tireSlip(),
  m_tireForce()
  {
}

Vehsim::~Vehsim()
{
}

void Vehsim::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    m_aimPoint = a_container.getData<opendlv::logic::action::AimPoint>();
  }

  if (a_container.getDataType() == opendlv::proxy::TorqueRequest::ID()) {
    m_brakeEnabled = false;
    if (a_container.getSenderStamp() ==  m_leftMotorID) {
      auto torqueContainer = a_container.getData<opendlv::proxy::TorqueRequest>();
      m_torqueRequest1 = torqueContainer.getTorque();
    } else if(a_container.getSenderStamp() == m_rightMotorID) {
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
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

        float yawRef = yawModel(m_aimPoint, m_x);
        Eigen::ArrayXf delta = calcSteerAngle(yawRef, m_x);
        Eigen::ArrayXf Fz = loadTransfer(m_x);
        Eigen::ArrayXf Fx = longitudinalControl(Fz);
        Eigen::ArrayXf Fy = tireModel(delta,m_x,Fz);
        Eigen::ArrayXf dx = motion(delta,Fy,Fx,m_x);

/*
        std::cout << "du: " << dx(0);
        std::cout << ".   dv: " << dx(1);
        std::cout << ".   dr: " << dx(2);
        std::cout << ".   ddu: " << dx(3);
        std::cout << ".   ddv: " << dx(4);
        std::cout << ".   ddr: " << dx(5) << std::endl;

        std::cout << "u: " << m_x(0);
        std::cout << ".   v: " << m_x(1);
        std::cout << ".   r: " << m_x(3) << std::endl;
*/

        m_X += m_x*m_sampleTime + dx*(float)pow(m_sampleTime,2)/2;
        m_x += dx*m_sampleTime;

        opendlv::sim::Frame out1(m_X(0),m_X(1),m_X(2),m_X(3),m_X(4),m_X(5));
        odcore::data::Container c1(out1);
        getConference().send(c1);

        sendAccelerationRequest(yawRef);
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

Eigen::ArrayXf Vehsim::calcSteerAngle(float rRef, Eigen::ArrayXf x)
{
  float maxDelta = PI/6;
  float u = x(0);
  float deltaf = 0.0f;

  if(u>0.1f){
      deltaf = (m_L+m_Ku*m_mass*(float)pow(u,2.0f))*rRef/u;
  }

  deltaf = std::max<float>(std::min<float>(deltaf,maxDelta),-maxDelta);

  Eigen::ArrayXf delta(4);
  delta *= deltaf;

  return delta;
}

Eigen::ArrayXf Vehsim::loadTransfer(Eigen::ArrayXf x)
{
  float u   = x(0);
  float ax  = x(3);
  float ay  = x(4);

  float Fl  = 1/2*m_A*m_rho*m_Cl*(float)pow(u,2.0f);
  float Fd  = 1/2*m_A*m_rho*m_Cd*(float)pow(u,2.0f);

  Eigen::ArrayXf weightDist(4);
  weightDist << -m_lr, -m_lr, m_lf, m_lf;
  weightDist /= (2.0f*m_L);

  Eigen::ArrayXf FzStatic = weightDist*(Fl + m_mass*m_g);

  //##################### dFz calculations
  float phiDDot = 0; //(m_ms*ay*m_h0-m_kPhi*m_phiDot-(m_cPhi)*m_phi+m_ms*m_g*m_h0*(float)sin(m_phi))/(m_Ix+m_ms*(float)pow(m_h0,2.0f));

  Eigen::ArrayXf dFzy(4);
  dFzy << (-(m_mass*(-m_lr)*ay*m_h1/m_wf/m_L/2.0f)+(m_cPhi1*m_phi+m_kPhi1*m_phiDot)),
  ((m_mass*(-m_lr)*ay*m_h1/m_wf/m_L/2.0f)-(m_cPhi1*m_phi+m_kPhi1*m_phiDot)),
  (-(m_mass*(m_lf)*ay*m_h2/m_wr/m_L/2.0f)+(m_cPhi2*m_phi+m_kPhi2*m_phiDot)),
  ((m_mass*(m_lf)*ay*m_h2/m_wr/m_L/2.0f)-(m_cPhi2*m_phi+m_kPhi2*m_phiDot));

  Eigen::ArrayXf dFzx(4);
  dFzx << -m_mass*ax*m_h/2/m_L - Fd*m_hp/2.0f/m_L,
  -m_mass*ax*m_h/2.0f/m_L - Fd*m_hp/2.0f/m_L,
  +m_mass*ax*m_h/2.0f/m_L + Fd*m_hp/2.0f/m_L,
  +m_mass*ax*m_h/2.0f/m_L + Fd*m_hp/2.0f/m_L;


  Eigen::ArrayXf Fz = FzStatic+dFzy+dFzx;

  float eps = 1e-3;
  m_wheelLiftOffFlag = 0;
  for (int i=0; i < Fz.size(); i++)
  {
    if (Fz(i) < 0)
    {
      m_wheelLiftOffFlag = 1;
      Fz(i) = eps;
    }
  }

  m_phiDot += m_phiDot*m_sampleTime + phiDDot*(float)pow(m_sampleTime,2);
  m_phiDot += phiDDot*m_sampleTime;

  return Fz;
}

Eigen::ArrayXf Vehsim::tireModel(Eigen::ArrayXf delta, Eigen::ArrayXf x, Eigen::ArrayXf Fz)
{
  Eigen::ArrayXf Fy(4);

  float u = x(0);
  float v = x(1);
  float r = x(2);

  float a1, a2, a3, a4;

  if(abs(v)>1e-4){
    a1 = -atan2(v+m_lf*r,u+m_wf*r);
    a2 = -atan2(v+m_lf*r,u-m_wf*r);
    a3 = -atan2(v-m_lf*r,u+m_wf*r);
    a4 = -atan2(v-m_lf*r,u-m_wf*r);
  } else{
    a1 = 0;
    a2 = 0;
    a3 = 0;
    a4 = 0;
  }

  Eigen::ArrayXf alpha(4);
  alpha << a1, a2, a3, a4;
  alpha = alpha + delta;

  float Fy1 = interp2(m_tireLoad, m_tireSlip, m_tireForce, Fz(0), a1);
  float Fy2 = interp2(m_tireLoad, m_tireSlip, m_tireForce, Fz(1), a2);
  float Fy3 = interp2(m_tireLoad, m_tireSlip, m_tireForce, Fz(2), a3);
  float Fy4 = interp2(m_tireLoad, m_tireSlip, m_tireForce, Fz(3), a4);

  Fy << Fy1,Fy2,Fy3,Fy4;
  return Fy;
}


Eigen::ArrayXf Vehsim::longitudinalControl(Eigen::ArrayXf Fz)
{
  Eigen::ArrayXf Fx(4);
  float tr1 = m_torqueRequest1;
  float tr2 = m_torqueRequest2;

  if (!m_brakeEnabled) {
    float fx1 = tr1/m_wr;
    float fx2 = tr2/m_wr;

    Fx << 0,0,fx1,fx2;
  } else if (m_brakeEnabled) {
    Fx << -m_lr, -m_lr, m_lf, m_lf;
    Fx /= m_L;
    Fx *= m_Fbrake;
  }


  (void) (Fz);
  return Fx;
}


Eigen::ArrayXf Vehsim::motion(Eigen::ArrayXf delta, Eigen::ArrayXf Fy, Eigen::ArrayXf Fx, Eigen::ArrayXf x)
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

  float du = (Ffrx+Fflx+Frlx+Frrx)/m_mass + r*v;
  float dv = (Ffry+Ffly+Frly+Frry)/m_mass - r*u;
  float dr = (m_lr*(Frry+Frly)+m_lf*(Ffry+Ffly)+m_wf*(Ffrx-Fflx)+m_wr*(Frrx-Frlx))/m_Iz;

  float ddu = (du - v*r - x(3))*(1/m_sampleTime);
  float ddv = (dv + u*r - x(4))*(1/m_sampleTime);
  float ddr = (dr - x(5))*(1/m_sampleTime);

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

void Vehsim::sendAccelerationRequest(float yawRef)
{
  (void) yawRef;

  float u = m_x(0);
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
  Eigen::RowVectorXf X(2);
  Eigen::VectorXf Y(2);
  Eigen::MatrixXf V(2,2);

  int i = 0;
  int j = 0;
  // Find betweem which two points x lies
  while (arg_xq > arg_X(i)) {
    if (i >= arg_X.size()-1) {
      std::cout << "Interp2: x-value out of bounds" << std::endl;
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

  X << arg_X(i)-arg_xq, arg_xq-arg_X(i-1);
  Y << arg_Y(j)-arg_yq, arg_yq-arg_Y(j-1);
  V << arg_V(j-1,i-1), arg_V(j-1,i),
       arg_V(j,i-1), arg_V(j,i);

  auto vq = X*V*Y;
  (void) vq;

  std::cout << vq << std::endl;

  (void) arg_Y;
  (void) arg_X;
  (void) arg_V;
  (void) arg_xq;
  (void) arg_yq;
  (void) Y;

  return 0.01f;
}

void Vehsim::setUp()
{
  std::cout << "vehsim setup" << std::endl;

  auto kv = getKeyValueConfiguration();

  m_mass = kv.getValue<float>("global.vehicle-parameter.m");
  m_mus = kv.getValue<float>("global.vehicle-parameter.m-unsprung");
  m_Iz = kv.getValue<float>("global.vehicle-parameter.Iz");
  m_Ix = kv.getValue<float>("global.vehicle-parameter.Ix");
  m_g = kv.getValue<float>("global.vehicle-parameter.g");
  m_L = kv.getValue<float>("global.vehicle-parameter.l");
  m_lf = kv.getValue<float>("global.vehicle-parameter.lf");
  m_lr = kv.getValue<float>("global.vehicle-parameter.lr");
  m_wf = kv.getValue<float>("global.vehicle-parameter.wf");
  m_wr = kv.getValue<float>("global.vehicle-parameter.wf");
  m_mu = kv.getValue<float>("global.vehicle-parameter.mu");
  m_wR = kv.getValue<float>("global.vehicle-parameter.wr");
  m_A = kv.getValue<float>("global.vehicle-parameter.area");
  m_rho = kv.getValue<float>("global.vehicle-parameter.rho");
  m_Cl = kv.getValue<float>("global.vehicle-parameter.Cl");
  m_Cd = kv.getValue<float>("global.vehicle-parameter.Cd");
  m_Ca1 = kv.getValue<float>("global.vehicle-parameter.Ca1");
  m_Ca2 = kv.getValue<float>("global.vehicle-parameter.Ca2");
  m_h0 = kv.getValue<float>("global.vehicle-parameter.h0");
  m_h1 = kv.getValue<float>("global.vehicle-parameter.h1");
  m_h2 = kv.getValue<float>("global.vehicle-parameter.h2");
  m_h = kv.getValue<float>("global.vehicle-parameter.h");
  m_hp = kv.getValue<float>("global.vehicle-parameter.hp");
  m_kPhi = kv.getValue<float>("global.vehicle-parameter.kPhi");
  m_cPhi = kv.getValue<float>("global.vehicle-parameter.cPhi");
  m_kLambda = kv.getValue<float>("global.vehicle-parameter.kLambda");
  m_cLambda = kv.getValue<float>("global.vehicle-parameter.cLambda");
  m_sampleTime = kv.getValue<float>("opendlv-logic-cfsd18-action-vehsim.simulation-setup.sampleTime");
  float Bf = kv.getValue<float>("opendlv-logic-cfsd18-action-vehsim.simulation-setup.Bf");
  float Br = kv.getValue<float>("opendlv-logic-cfsd18-action-vehsim.simulation-setup.Br");
  m_C = kv.getValue<float>("opendlv-logic-cfsd18-action-vehsim.simulation-setup.C");
  float Df = kv.getValue<float>("opendlv-logic-cfsd18-action-vehsim.simulation-setup.Df");
  float Dr = kv.getValue<float>("opendlv-logic-cfsd18-action-vehsim.simulation-setup.Dr");
  m_E = kv.getValue<float>("opendlv-logic-cfsd18-action-vehsim.simulation-setup.E");
  m_leftMotorID = kv.getValue<uint32_t>("global.sender-stamp.left-motor");
  m_rightMotorID = kv.getValue<uint32_t>("global.sender-stamp.right-motor");


  m_Ku = (m_Ca2*m_lr-m_Ca1*m_lf)/(m_Ca1*m_Ca2*m_L);
  m_ms = m_mass-m_mus;
  m_cPhi1 = m_cPhi*m_cLambda;
  m_cPhi2 = m_cPhi-m_cPhi1;
  m_kPhi1 = m_kPhi*m_kLambda;
  m_kPhi2 = m_kPhi-m_kPhi1;

  m_x = Eigen::ArrayXf(6);
  m_X = Eigen::ArrayXf(6);
  m_x << 0,0,0,0,0,0;
  m_X << 0,0,0,0,0,0;

  m_B = Eigen::ArrayXf(4);
  m_D = Eigen::ArrayXf(4);
  m_B << Bf, Bf, Br, Br;
  m_D << Df, Df, Dr, Dr;

  m_aimPoint = opendlv::logic::action::AimPoint(0.0f,0.0f,5.0f);

  // char result[ PATH_MAX ];
  // ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  std::cout << "Path: " << "/home/oscar/Desktop/" <<  std::endl;

  std::ifstream file( "/opt/opendlv.data/TireData.txt", std::ifstream::in );
  std::string row;
  std::string value;
  int irow = 1;
  int icol;

  std::cout << file.is_open() << std::endl;
  if (file.is_open()){
    std::cout << "File open" << std::endl;
    while(file.good()){
      std::cout << "reading new row" << std::endl;
      icol = 1;
      getline ( file, row);
      while(file.good()){
        std::cout << "reading new column, ";
        getline ( file, value,',');
        if(irow == 1 && icol != 1){
          m_tireLoad(icol-1) = std::stof(value);
        } else if (icol == 1 && irow != 1) {
          m_tireSlip(irow) = std::stof(value);
        } else {
          m_tireForce(icol-1,irow-1) = std::stof(value);
        }
        std::cout << m_tireLoad << std::endl;
        std::cout << m_tireSlip << std::endl;
        icol ++;
      }
      irow ++;
    }
  }

// m_tireLoad << 1500, 1392.857, 1285.714, 1178.571, 1071.429,
//    964.2857, 857.1429, 750, 642.8571, 535.7143, 428.5714, 321.4286, 214.2857, 107.1429, 0;
}

void Vehsim::tearDown()
{
}

}
}
}
}
