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

#ifndef OPENDLV_LOGIC_CFSD18_PERCEPTION_DETECTCONE_HPP
#define OPENDLV_LOGIC_CFSD18_PERCEPTION_DETECTCONE_HPP

#include <memory>
#include <vector>
#include <cmath>

#include "opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/data/Container.h"
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
 

#include "opendavinci/generated/odcore/data/CompactPointCloud.h"
 
#include <opendavinci/odcore/wrapper/Eigen.h>

#include <opendavinci/odcore/data/TimeStamp.h>


namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

using namespace std;
using namespace Eigen;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::wrapper;


class DetectCone : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  DetectCone(int32_t const &, char **);
  DetectCone(DetectCone const &) = delete;
  DetectCone &operator=(DetectCone const &) = delete;
  virtual ~DetectCone();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void matchPoints(MatrixXd, MatrixXd);
  void findMatch(MatrixXd, MatrixXd);
  MatrixXd Spherical2Cartesian(double, double, double);
  opendlv::logic::sensation::Point Cartesian2Spherical(double, double, double);
  void SendCollectedCones(MatrixXd);
  void SendMatchedContainer(MatrixXd);
  void setUp();
  void tearDown();
  MatrixXd m_lastLidarData;
  MatrixXd m_lastCameraData;
  MatrixXd m_pointMatched;
  double m_diffVec;
  MatrixXd m_finalPointCloud;
  double m_threshold;
  int64_t m_timeDiffMilliseconds;
  odcore::data::TimeStamp m_lastTimeStamp;
  MatrixXd m_coneCollector;
  int m_coneCollected;
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
};

}
}
}
}

#endif
