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

#ifndef OPENDLV_LOGIC_CFSD18_ACTION_BRAKES_HPP
#define OPENDLV_LOGIC_CFSD18_ACTION_BRAKES_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>

//#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

class Brakes : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Brakes(int32_t const &, char **);
  Brakes(Brakes const &) = delete;
  Brakes &operator=(Brakes const &) = delete;
  virtual ~Brakes();
  virtual void nextContainer(odcore::data::Container &);
 
 public:
  float m_totalmass;
  

 private:
  void setUp();
  void tearDown();
};

}
}
}
}

#endif
