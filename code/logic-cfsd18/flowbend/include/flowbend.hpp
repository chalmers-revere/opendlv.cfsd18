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

#ifndef LOGIC_CFSD18_FLOWBEND_FLOWBEND_HPP
#define LOGIC_CFSD18_FLOWBEND_FLOWBEND_HPP

#include <map>

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/base/Mutex.h>
#include <opendavinci/odcore/data/Container.h>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {

class FlowBend : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  FlowBend(int32_t const &, char **);
  FlowBend(FlowBend const &) = delete;
  FlowBend &operator=(FlowBend const &) = delete;
  virtual ~FlowBend();
  virtual void nextContainer(odcore::data::Container &);

 private:
  odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
  void setUp();
  void tearDown();
};

}
}
}

#endif
