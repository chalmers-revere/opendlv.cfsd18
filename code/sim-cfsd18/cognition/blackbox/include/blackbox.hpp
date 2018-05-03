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

#ifndef OPENDLV_SIM_CFSD18_COGNITION_BLACKBOX_HPP
#define OPENDLV_SIM_CFSD18_COGNITION_BLACKBOX_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <cmath>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/base/Lock.h>
//#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
//#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>


namespace opendlv {
namespace sim {
namespace cfsd18 {
namespace cognition {

class BlackBox : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  BlackBox(int32_t const &, char **);
  BlackBox(BlackBox const &) = delete;
  BlackBox &operator=(BlackBox const &) = delete;
  virtual ~BlackBox();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();

  void initializeCollection(int);
  void generateSurfaces(ArrayXXf, ArrayXXf, ArrayXXf);
  //void CheckContainer(uint32_t);
  Eigen::MatrixXd Spherical2Cartesian(double, double, double);


  odcore::data::TimeStamp m_lastTimeStamp;
  Eigen::MatrixXd m_coneCollector;
  odcore::base::Mutex m_coneMutex;
  bool m_newFrame;
  int m_timeDiffMilliseconds;
  int m_lastTypeId;
  int m_surfaceId;

  const double DEG2RAD = 0.017453292522222; // PI/180.0

  void findSafeLocalPath(ArrayXXf, ArrayXXf);
  ArrayXXf placeEquidistantPoints(ArrayXXf, bool, int, float);
  ArrayXXf traceBackToClosestPoint(ArrayXXf, ArrayXXf, ArrayXXf);
  ArrayXXf orderCones(ArrayXXf, ArrayXXf);
  ArrayXXf orderAndFilterCones(ArrayXXf, ArrayXXf);
  ArrayXXf insertNeededGuessedCones(ArrayXXf, ArrayXXf, ArrayXXf, float, float, bool);
  ArrayXXf guessCones(ArrayXXf, ArrayXXf, float, bool, bool, bool);
  float findTotalPathLength(ArrayXXf);
  float findFactorToClosestPoint(ArrayXXf, ArrayXXf, ArrayXXf);

  void sortIntoSideArrays(MatrixXd, int, int, int, int);
  void sendMatchedContainer(Eigen::ArrayXXf, Eigen::ArrayXXf);

};

}
}
}
}

#endif
