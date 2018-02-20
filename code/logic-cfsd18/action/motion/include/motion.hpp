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

#ifndef OPENDLV_LOGIC_CFSD18_ACTION_MOTION_HPP
#define OPENDLV_LOGIC_CFSD18_ACTION_MOTION_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

class Motion : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Motion(int32_t const &, char **);
  Motion(Motion const &) = delete;
  Motion &operator=(Motion const &) = delete;
  virtual ~Motion();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
  void calcTorque(float);
  void sendActuationContainer(int32_t, float);

 private:
   float m_steeringAngle;
   bool m_brakeEnabled;
   double m_deceleration;
   float m_speed;
   MatrixXd m_vehicleModelParameters;
   int32_t m_leftMotorID;
   int32_t m_rightMotorID;

};

}
}
}
}

#endif
