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

#ifndef OPENDLV_LOGIC_CFSD18_SENSATION_SLAM_HPP
#define OPENDLV_LOGIC_CFSD18_SENSATION_SLAM_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/base/Mutex.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include <opendlv/data/environment/Point3.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
//#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {

class Slam : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Slam(int32_t const &, char **);
  Slam(Slam const &) = delete;
  Slam &operator=(Slam const &) = delete;
  virtual ~Slam();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
  bool CheckContainer(uint32_t objectId, odcore::data::TimeStamp timeStamp);
  bool isKeyframe(Eigen::MatrixXd Cones);
  void performSLAM(Eigen::MatrixXd Cones);
  Eigen::MatrixXd conesToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd Cones);
  Eigen::Vector3d Spherical2Cartesian(double azimuth, double zenimuth, double distance);
  void addConesToMap(Eigen::MatrixXd cones);
  bool newCone(Eigen::MatrixXd cone);
  uint32_t m_timeDiffMilliseconds;
  odcore::data::TimeStamp m_lastTimeStamp;
  Eigen::MatrixXd m_coneCollector;
  uint32_t m_lastObjectId;
  odcore::base::Mutex m_coneMutex;
  odcore::base::Mutex m_sensorMutex;
  odcore::base::Mutex m_mapMutex;
  Eigen::Vector3d m_odometryData;
  opendlv::data::environment::WGS84Coordinate m_gpsReference;
  Eigen::MatrixXd m_map;
  double m_newConeThreshold;

    // Constants for degree transformation
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
};

}
}
}
}

#endif
