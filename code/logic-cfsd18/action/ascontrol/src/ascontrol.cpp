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

#include "ascontrol.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace action {

Ascontrol::Ascontrol(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-action-ascontrol")
{
  std::cout << getName() << ": I'm setting up v2" << '\n';
}

Ascontrol::~Ascontrol()
{
}

void Ascontrol::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::system::SignalStatusMessage::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();



  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Ascontrol::body()
{
    std::cout << "[" << getName() << "]: I sent a container" << std::endl;
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

          opendlv::system::SystemOperationState ASstatusMessage(1,"OKAY");
          odcore::data::Container output(ASstatusMessage);

          getConference().send(output);
          opendlv::proxy::GroundDecelerationRequest test;
          odcore::data::Container outTest(test);
          getConference().send(outTest);
          std::cout << "I sent a container" << std::endl;
  }


// Comment this out until PwmRequest has been added in our project
/*
  opendlv::proxy::PwmRequest LED_R;
  opendlv::proxy::PwmRequest LED_G;
  opendlv::proxy::PwmRequest LED_B;

  if (true) // Add logic to decide which colour to send
  {
    int32_t R = 50; // Assign the percentage of each colour (0-100)
    int32_t G = 50; // These values should be set in a config file
    int32_t B = 50;
    int32_t tot = fmax(B,fmax(R,G));
  }

  LED_R.setDutyCycleNs(R/tot*100);  // The fractions are normalized to make brightness more consistent
  LED_G.setDutyCycleNs(G/tot*100);
  LED_B.setDutyCycleNs(B/tot*100);

  LED_R.setPin(m_RedPin);
  LED_G.setPin(m_GreenPin);
  LED_B.setPin(m_BluePin);

  odcore::data::Container red(LED_R);
  odcore::data::Container green(LED_G);
  odcore::data::Container blue(LED_B);

  // It will be necessary to add some if statement controlling the frequency of the light to match rules

  getConference().send(red);
  getConference().send(green);
  getConference().send(blue);
*/

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Ascontrol::setUp()
{
  // std::string const exampleConfig =
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "logic-cfsd18-action-ascontrol.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
  std::cout << "[" << getName() << "]: I'm setting up!" << std::endl;
}

void Ascontrol::tearDown()
{
}

}
}
}
}
