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

#ifndef OPENDLV_SIM_CFSD18_PERCEPTION_DETECTCONE_HPP
#define OPENDLV_SIM_CFSD18_PERCEPTION_DETECTCONE_HPP

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <cmath>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
//#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <opendavinci/odcore/base/Lock.h>

namespace opendlv {
namespace sim {
namespace cfsd18 {
namespace perception {

class DetectCone : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  DetectCone(int32_t const &, char **);
  DetectCone(DetectCone const &) = delete;
  DetectCone &operator=(DetectCone const &) = delete;
  virtual ~DetectCone();
  virtual void nextContainer(odcore::data::Container &);

 private:
  float m_heading;
  ArrayXXf m_location;
  ArrayXXf m_leftCones;
  ArrayXXf m_rightCones;
  ArrayXXf m_smallCones;
  ArrayXXf m_bigCones;
  uint32_t m_senderStamp = 1;
  odcore::base::Mutex m_locationMutex;
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD

  odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
  void setUp();
  void tearDown();
  void readMap(std::string);
  ArrayXXf simConeDetectorBox(ArrayXXf, ArrayXXf, float, float, float);
  void sendMatchedContainer(Eigen::MatrixXd, int, int);
  void Cartesian2Spherical(double, double, double, opendlv::logic::sensation::Point &);
};

}
}
}
}

#endif
