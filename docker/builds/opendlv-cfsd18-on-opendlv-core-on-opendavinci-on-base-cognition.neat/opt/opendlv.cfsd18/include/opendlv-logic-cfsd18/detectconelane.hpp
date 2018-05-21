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

#ifndef OPENDLV_LOGIC_CFSD18_PERCEPTION_DETECTCONELANE_HPP
#define OPENDLV_LOGIC_CFSD18_PERCEPTION_DETECTCONELANE_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <cmath>
#include <map>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/base/Lock.h>
//#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
//#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>


namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

class DetectConeLane : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  DetectConeLane(int32_t const &, char **);
  DetectConeLane(DetectConeLane const &) = delete;
  DetectConeLane &operator=(DetectConeLane const &) = delete;
  virtual ~DetectConeLane();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();

  void initializeCollection();
  void generateSurfaces(ArrayXXf, ArrayXXf, ArrayXXf);
  //void CheckContainer(uint32_t);
  Eigen::MatrixXd Spherical2Cartesian(double, double, double);


  bool m_newFrame;
  bool m_directionOK;
  bool m_distanceOK;
  bool m_runOK;
  std::map< double, float > m_directionFrame;
  std::map< double, float > m_distanceFrame;
  std::map< double, int > m_typeFrame;
  std::map< double, float > m_directionFrameBuffer;
  std::map< double, float > m_distanceFrameBuffer;
  std::map< double, int > m_typeFrameBuffer;
  int m_lastDirectionId;
  int m_lastDistanceId;
  int m_lastTypeId;
  bool m_newDirectionId;
  bool m_newDistanceId;
  bool m_newTypeId;
  std::chrono::time_point<std::chrono::system_clock> m_directionTimeReceived;
  std::chrono::time_point<std::chrono::system_clock> m_distanceTimeReceived;
  std::chrono::time_point<std::chrono::system_clock> m_typeTimeReceived;
  uint64_t m_nConesInFrame;
  int m_objectPropertyId;
  int m_directionId;
  int m_distanceId;
  int m_typeId;
  odcore::base::Mutex m_directionMutex = {};
  odcore::base::Mutex m_distanceMutex = {};
  odcore::base::Mutex m_typeMutex = {};
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
