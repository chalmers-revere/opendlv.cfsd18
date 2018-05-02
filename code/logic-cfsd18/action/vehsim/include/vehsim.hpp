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
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <fstream>

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
  Eigen::ArrayXf calcSteerAngle(float,Eigen::ArrayXf, float, float, float);
  Eigen::ArrayXf loadTransfer(Eigen::ArrayXf, float, float, float, float,
  float, float);
  Eigen::ArrayXf tireModel(Eigen::ArrayXf, Eigen::ArrayXf, Eigen::ArrayXf,
    float, float, float, float, Eigen::VectorXf, Eigen::VectorXf, Eigen::MatrixXf);
  Eigen::ArrayXf atanArr(Eigen::ArrayXf);
  Eigen::ArrayXf longitudinalControl(Eigen::ArrayXf, Eigen::ArrayXf, Eigen::VectorXf,
    Eigen::VectorXf, Eigen::MatrixXf, Eigen::ArrayXf*, Eigen::ArrayXf, float, float);
  Eigen::ArrayXf motion(Eigen::ArrayXf,Eigen::ArrayXf,Eigen::ArrayXf,
    Eigen::ArrayXf, float, float, float, float, float, float, float);
  void sendAccelerationRequest(float, Eigen::ArrayXf);
  float interp2(Eigen::VectorXf, Eigen::VectorXf, Eigen::MatrixXf, float, float);
  Eigen::ArrayXf motorModel(float,
    Eigen::ArrayXf, Eigen::ArrayXf);

 private:
   opendlv::proxy::GroundSteeringRequest m_headingRequest;
   float m_torqueRequest1;
   float m_torqueRequest2;
   float m_deceleration;
   bool m_brakeEnabled;
   float m_delta;
   std::ofstream m_outputData;
};

}
}
}
}

#endif
