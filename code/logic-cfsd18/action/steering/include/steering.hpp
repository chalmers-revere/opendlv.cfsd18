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

#ifndef OPENDLV_LOGIC_CFSD18_ACTION_STEERING_HPP
#define OPENDLV_LOGIC_CFSD18_ACTION_STEERING_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

class Steering : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Steering(int32_t const &, char **);
  Steering(Steering const &) = delete;
  Steering &operator=(Steering const &) = delete;
  virtual ~Steering();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
  uint32_t calcSteering(float);

 private:
   uint32_t m_pwmId;
   int32_t m_stateId1;
   int32_t m_stateId2;
   int32_t m_stateId3;
   float m_Kp;
};

}
}
}
}

#endif
