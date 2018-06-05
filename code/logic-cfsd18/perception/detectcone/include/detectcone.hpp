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
#include <thread>
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

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tiny_dnn/tiny_dnn.h>

#include "point.hpp"

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

  bool ExtractSharedImage(odcore::data::image::SharedImage *); 
  void initializeCollection();
  void saveRecord(cv::Mat);
  void blockMatching(cv::Mat&, cv::Mat, cv::Mat);
  void reconstruction(cv::Mat, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);
  void convertImage(cv::Mat, int, int, tiny_dnn::vec_t&);
  void slidingWindow(const std::string&);
  std::vector <cv::Point> imRegionalMax(cv::Mat, int, double, int);
  float median(std::vector<float>);
  float mean(std::vector<float>);
  void whiteBalance(cv::Mat);
  void gather_points(cv::Mat, std::vector<float>, std::vector<int>&, std::vector<float>&);
  void filterKeypoints(std::vector<cv::Point3f>&);
  void xyz2xy(cv::Mat, cv::Point3f, cv::Point2f&, int&);
  void backwardDetection(cv::Mat, std::vector<cv::Point3f>, std::vector<int>&);
  void forwardDetectionORB();

  Eigen::MatrixXd Spherical2Cartesian(double, double, double);
  void Cartesian2Spherical(double, double, double, opendlv::logic::sensation::Point&);
  void SendCollectedCones(Eigen::MatrixXd);
  void SendMatchedContainer(Eigen::MatrixXd);

  Eigen::MatrixXd m_lastLidarData;
  Eigen::MatrixXd m_lastCameraData;
  Eigen::MatrixXd m_pointMatched;
  double m_diffVec;
  Eigen::MatrixXd m_finalPointCloud;
  double m_threshold;
  int64_t m_timeDiffMilliseconds;
  odcore::data::TimeStamp m_coneTimeStamp;
  odcore::data::TimeStamp m_imgTimeStamp;
  Eigen::MatrixXd m_coneCollector;
  uint32_t m_lastObjectId;
  bool m_newFrame;

  odcore::base::Mutex m_coneMutex;
  odcore::base::Mutex m_imgMutex;
  bool m_recievedFirstImg;
  cv::Mat m_img;
  tiny_dnn::network<tiny_dnn::sequential> m_slidingWindow;
  tiny_dnn::network<tiny_dnn::sequential> m_efficientSlidingWindow;
  bool m_lidarIsWorking;
  int64_t m_checkLiarMilliseconds;
  uint32_t m_senderStamp = 0;
  uint32_t m_attentionSenderStamp = 0;
  uint32_t m_count = 0;
  int m_patchSize = 64;
  int m_width = 672;
  int m_height = 376;
  double m_xShift;                //lateral distance between camera and LiDAR 
  double m_yShift = 0.833;            //Height difference between camera and LiDAR
  double m_zShift = 1.872;        //Distance between camera and LiDAR in forward distance

  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
};

}
}
}
}

#endif
