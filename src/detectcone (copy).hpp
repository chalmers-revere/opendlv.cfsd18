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

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
#include "odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"
#include "opendavinci/odcore/wrapper/SharedMemory.h"
#include "opendavinci/generated/odcore/data/CompactPointCloud.h"
#include <opendavinci/odcore/base/Lock.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <tiny_dnn/tiny_dnn.h>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

class DetectCone : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  DetectCone(int32_t const &, char **);
  DetectCone(DetectCone const &) = delete;
  DetectCone &operator=(DetectCone const &) = delete;
  virtual ~DetectCone();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();

	bool CheckContainer(uint32_t objectId, odcore::data::TimeStamp timestamp);
  bool ExtractSharedImage(odcore::data::image::SharedImage *);
  void saveImg(double currentTime);
  void featureBased();
  void reconstruction(cv::Mat, cv::Mat &, cv::Mat &, cv::Mat &, cv::Mat &);
  void blockMatching(cv::Mat &, cv::Mat, cv::Mat);
  void xyz2xy(cv::Mat, cv::Vec3f, cv::Vec2f &);
  float_t depth2resizeRate(double, double);
  void convertImage(cv::Mat, int, int, tiny_dnn::vec_t &);
  void slidingWindow(const std::string &);
  int backwardDetection(cv::Mat, cv::Vec3f);
  void efficientSlidingWindow(const std::string &, int, int);
  void softmax(cv::Vec4d, cv::Vec4d &);
  std::vector <cv::Point> imRegionalMax(cv::Mat, int, double, int);
  void forwardDetection();

  // void matchPoints(Eigen::MatrixXd, Eigen::MatrixXd);
  // void findMatch(Eigen::MatrixXd, Eigen::MatrixXd);
  Eigen::MatrixXd Spherical2Cartesian(double, double, double);
  void Cartesian2Spherical(double, double, double, opendlv::logic::sensation::Point &);
  void SendCollectedCones(Eigen::MatrixXd);
  void SendMatchedContainer(Eigen::MatrixXd);

  Eigen::MatrixXd m_lastLidarData;
  Eigen::MatrixXd m_lastCameraData;
  Eigen::MatrixXd m_pointMatched;
  double m_diffVec;
  Eigen::MatrixXd m_finalPointCloud;
  double m_threshold;
  int64_t m_timeDiffMilliseconds;
  odcore::data::TimeStamp m_lastTimeStamp;
  Eigen::MatrixXd m_coneCollector;
  uint32_t m_lastObjectId;

  odcore::base::Mutex m_coneMutex;
  bool m_recievedFirstImg;
  cv::Mat m_img;
  tiny_dnn::network<tiny_dnn::sequential> m_nn;
  bool m_lidarIsWorking;
  int64_t m_checkLiarMilliseconds;
  uint32_t m_senderStamp = 0;
  uint32_t m_attentionSenderStamp = 0;

  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
};

}
}
}
}

#endif
