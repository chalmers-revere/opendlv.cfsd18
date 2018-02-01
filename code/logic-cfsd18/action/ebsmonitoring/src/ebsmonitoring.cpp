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

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include "ebsmonitoring.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

Ebsmonitoring::Ebsmonitoring(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-ebsmonitoring"),
  m_analogPin(),
  m_heartBeat(),
  m_analogReading()
{
  m_analogPin = 1;
  m_heartBeat = true;
}

Ebsmonitoring::~Ebsmonitoring()
{
}

void Ebsmonitoring::nextContainer(odcore::data::Container &a_container)
{/*
  if ((false) && a_container.getDataType() == opendlv::proxy::AnalogReading::ID()) { 
    auto analogReading = a_container.getData<opendlv::proxy::AnalogReading>();
    m_analogReading = analogReading.getVoltage();
  }*/
  std::cout << a_container;
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Ebsmonitoring::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

//      Eigen::AngleAxisd deltaRollAngle(deltaRoll, Eigen::Vector3d::UnitX());

  }

  m_heartBeat = !m_heartBeat;
  /*opendlv::proxy::ToggleRequest c;
  c.setToggleState = m_State;
  getConference().send(c);*/

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Ebsmonitoring::setUp()
{
  // std::string const exampleConfig =
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-ebsmonitoring.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Ebsmonitoring::tearDown()
{
}

}
}
}
}
