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

#ifndef OPENDLV_LOGIC_CFSD18_COGNITION_TRACK_HPP
#define OPENDLV_LOGIC_CFSD18_COGNITION_TRACK_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <opendavinci/odcore/base/Lock.h>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <map>
#include <chrono>
namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace cognition {

class Track : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Track(int32_t const &, char **);
  Track(Track const &) = delete;
  Track &operator=(Track const &) = delete;
  virtual ~Track();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();

  void collectAndRun(std::map< double, std::vector<float> >);
  Eigen::RowVector2f traceBackToClosestPoint(Eigen::RowVector2f, Eigen::RowVector2f, Eigen::RowVector2f);
  Eigen::MatrixXf placeEquidistantPoints(Eigen::MatrixXf, bool, int, float);
  std::tuple<float, float> driverModelSteering(Eigen::MatrixXf, float, float);
  float driverModelSharp(Eigen::MatrixXf, float);
  float driverModelVelocity(Eigen::MatrixXf, float, float, float, float, float, float, float, float, bool);
  std::vector<float> curvatureTriCircle(Eigen::MatrixXf, int);
  std::vector<float> curvaturePolyFit(Eigen::MatrixXf);

  /* Member variables */
  float m_groundSpeed;
  bool m_newFrame;
  int m_objectId;
  odcore::base::Mutex m_groundSpeedMutex;
  odcore::base::Mutex m_surfaceMutex;
  std::map< double, std::vector<float> > m_surfaceFrame;
  std::map< double, std::vector<float> > m_surfaceFrameBuffer;
  uint64_t m_nSurfacesInframe;
  int m_surfaceId;
  std::chrono::time_point<std::chrono::system_clock> m_timeReceived;
  int m_lastObjectId;
  bool m_newId;
};

}
}
}
}

#endif
