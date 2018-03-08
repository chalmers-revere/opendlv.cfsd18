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

#ifndef OPENDLV_LOGIC_CFSD18_ACTION_VEHSIM_HPP
#define OPENDLV_LOGIC_CFSD18_ACTION_VEHSIM_HPP

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

class Vehsim : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  Vehsim(int32_t const &, char **);
  Vehsim(Vehsim const &) = delete;
  Vehsim &operator=(Vehsim const &) = delete;
  virtual ~Vehsim();
  virtual void nextContainer(odcore::data::Container &);

 private:
  odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
  void setUp();
  void tearDown();
  float yawModel(opendlv::logic::action::AimPoint, Eigen::ArrayXf);
  Eigen::ArrayXf calcSteerAngle(float,Eigen::ArrayXf);
  Eigen::ArrayXf loadTransfer(Eigen::ArrayXf);
  Eigen::ArrayXf tireModel(Eigen::ArrayXf, Eigen::ArrayXf, Eigen::ArrayXf);
  Eigen::ArrayXf atanArr(Eigen::ArrayXf);
  Eigen::ArrayXf longitudinalControl(Eigen::ArrayXf);
  Eigen::ArrayXf motion(Eigen::ArrayXf,Eigen::ArrayXf,Eigen::ArrayXf,Eigen::ArrayXf);
  void sendAccelerationRequest(float);

 private:
   opendlv::logic::action::AimPoint m_aimPoint;
   float m_mass;
   float m_mus;
   float m_ms;
   float m_Iz;
   float m_Ix;
   float m_g;
   float m_L;
   float m_lf;
   float m_lr;
   float m_wf;
   float m_wr;
   float m_mu;
   float m_wR;
   float m_Ku;
   float m_A;
   float m_rho;
   float m_Cl;
   float m_Cd;
   float m_h0;
   float m_h1;
   float m_h2;
   float m_h;
   float m_hp;
   float m_Ca1;
   float m_Ca2;
   float m_kPhi;
   float m_cPhi;
   float m_kPhi1;
   float m_cPhi1;
   float m_kPhi2;
   float m_cPhi2;
   float m_kLambda;
   float m_cLambda;
   float m_phi;
   float m_phiDot;
   float m_wheelLiftOffFlag;
   Eigen::ArrayXf m_B;
   float m_C;
   Eigen::ArrayXf m_D;
   float m_E;
   Eigen::ArrayXf m_x;
   Eigen::ArrayXf m_X;
   float m_sampleTime;
   float m_torqueRequest1;
   float m_torqueRequest2;
   float m_Fbrake;
   uint32_t m_leftMotorID;
   uint32_t m_rightMotorID;
   bool m_brakeEnabled;
};

}
}
}
}

#endif
