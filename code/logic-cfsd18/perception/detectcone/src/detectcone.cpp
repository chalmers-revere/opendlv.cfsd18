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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "detectcone.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

DetectCone::DetectCone(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-perception-detectcone")
, m_img()
{
}

DetectCone::~DetectCone()
{
  m_img.release();
}

void DetectCone::setUp()
{
  rectify();
}

void DetectCone::tearDown()
{
}

void DetectCone::nextContainer(odcore::data::Container &a_c)
{
  if (a_c.getDataType() == odcore::data::image::SharedImage::ID()) {
    odcore::data::image::SharedImage sharedImg =
        a_c.getData<odcore::data::image::SharedImage>();
    if (!ExtractSharedImage(&sharedImg)) {
      std::cout << "[" << getName() << "] " << "[Unable to extract shared image."
          << std::endl;
      return;
    }
  }

}

bool DetectCone::ExtractSharedImage(
      odcore::data::image::SharedImage *a_sharedImage)
{
  bool isExtracted = false;
  std::shared_ptr<odcore::wrapper::SharedMemory> sharedMem(
      odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
          a_sharedImage->getName()));
  if (sharedMem->isValid()) {
    const uint32_t nrChannels = a_sharedImage->getBytesPerPixel();
    const uint32_t imgWidth = a_sharedImage->getWidth();
    const uint32_t imgHeight = a_sharedImage->getHeight();
    IplImage* myIplImage = 
        cvCreateImage(cvSize(imgWidth,imgHeight), IPL_DEPTH_8U, nrChannels);
    cv::Mat bufferImage = cv::Mat(myIplImage);
    {
      sharedMem->lock();
      memcpy(bufferImage.data, sharedMem->getSharedMemory()
        , imgWidth*imgHeight*nrChannels);
      sharedMem->unlock();
    }
    m_img.release();
    m_img = bufferImage.clone();
    bufferImage.release();
    cvReleaseImage(&myIplImage);
    isExtracted = true;
  } else {
    std::cout << "[" << getName() << "] " << "Sharedmem is not valid." << std::endl;
  }
  return isExtracted;
}

void DetectCone::featureBased() {
  cv::Mat m_img_hsv, channel[3];
  cv::cvtColor(m_img, m_img_hsv, CV_RGB2HSV);
  cv::split(m_img_hsv, channel);
  //cv::adaptiveThreshold(adapThreshImg, channel[1], 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, m_adapThreshKernelSize, m_adapThreshConst);
  cv::imshow("img", channel[0]);
  cv::waitKey(0);
}

void DetectCone::rectify(){
  cv::Mat mtxLeft = (cv::Mat_<double>(3, 3) << 
    699.8341, 0, 670.6991, 
    0, 699.9473, 327.9933, 
    0, 0, 1);
  std::vector<double> distLeft{-0.1708, 0.0267, 0, 0, 0};
  cv::Mat mtxRight = (cv::Mat_<double>(3, 3) << 
    702.2891, 0, 667.0359,
    0, 701.5237, 358.7018,
    0, 0, 1);
  cv::Mat distRight = (cv::Mat_<double>(5, 1) << 
    -0.1733, 0.0275, 0, 0, 0);
  cv::Mat R = (cv::Mat_<double>(3, 3) << 
    0.9998, -0.0016, -0.0215,
    0.0016, 1, -0.0021,
    0.0215, 0.0020, 0.9998);
  cv::transpose(R, R);
  cv::Mat T = (cv::Mat_<double>(3, 1) << 
    -119.1632, 0.2062, 0.0252);
  //double F = 700;
  //double d = 120;
  //double factor = F * d / 2;

  cv::Size stdSize = cv::Size(1280, 720);
  cv::Mat img = cv::imread("2.png");
  int width = img.cols;
  int height = img.rows;
  cv::Mat imgL(img, cv::Rect(0, 0, width/2, height));
  cv::Mat imgR(img, cv::Rect(width/2, 0, width/2, height));

  cv::resize(imgL, imgL, stdSize);
  cv::resize(imgR, imgR, stdSize);

  //std::cout << imgR.size() <<std::endl;
  
  cv::Mat R1, R2, P1, P2, Q;
  cv::Rect validRoI[2];
  cv::stereoRectify(mtxLeft, distLeft, mtxRight, distRight, stdSize, R, T, R1, R2, P1, P2, Q, 
    cv::CALIB_ZERO_DISPARITY, 0.0, stdSize, &validRoI[0], &validRoI[1]);

  cv::Mat rmap[2][2];
  cv::initUndistortRectifyMap(mtxLeft, distLeft, R1, P1, stdSize, CV_16SC2, rmap[0][0], rmap[0][1]);
  cv::initUndistortRectifyMap(mtxRight, distRight, R2, P2, stdSize, CV_16SC2, rmap[1][0], rmap[1][1]);
  cv::remap(imgL, imgL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
  cv::remap(imgR, imgR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);

  /*
  cv::imwrite("2_left.png", imgL);
  cv::imwrite("2_right.png", imgR);

  cv::Mat rectify;
  cv::hconcat(imgL, imgR, rectify);
  cv::namedWindow("rectify", cv::WINDOW_NORMAL);
  cv::imshow("rectify", rectify);
  cv::waitKey(0);
  cv::destroyAllWindows();
  */
  cv::Mat disp;
  blockMatching(imgL, imgR, disp);
  std::cout << disp.size() <<std::endl;
}

void DetectCone::blockMatching(cv::Mat imgL, cv::Mat imgR, cv::Mat disp){
  cv::Mat grayL, grayR;

  cv::cvtColor(imgL, grayL, CV_BGR2GRAY);
  cv::cvtColor(imgR, grayR, CV_BGR2GRAY);

  cv::StereoBM sbm;
  sbm.state->SADWindowSize = 9;
  sbm.state->numberOfDisparities = 112;
  sbm.state->preFilterSize = 5;
  sbm.state->preFilterCap = 61;
  sbm.state->minDisparity = -39;
  sbm.state->textureThreshold = 507;
  sbm.state->uniquenessRatio = 0;
  sbm.state->speckleWindowSize = 0;
  sbm.state->speckleRange = 8;
  sbm.state->disp12MaxDiff = 1;
/*
  cv::StereoSGBM sgbm;
  sgbm.SADWindowSize = 3;
  sgbm.numberOfDisparities = 112 - 16;
  sgbm.preFilterCap = 63;
  sgbm.minDisparity = 16;
  sgbm.uniquenessRatio = 10;
  sgbm.speckleWindowSize = 100;
  sgbm.speckleRange = 32;
  sgbm.disp12MaxDiff = 1;
  sgbm.fullDP = false;
  sgbm.P1 = 8*3*3^2;
  sgbm.P2 = 32*3*3^2;
*/
  sbm(grayL, grayR, disp);
  cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
}



}
}
}
}
