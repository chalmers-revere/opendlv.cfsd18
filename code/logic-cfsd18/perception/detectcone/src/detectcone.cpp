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
  //rectify();
  run_cnn("sliding_window", "test.png");
  /*
  tiny_dnn::vec_t data;
  std::string img_path;
  for( int a = 0; a < 2; a = a + 1 ) {
      img_path = "data/yellow/" + std::to_string(a) + ".png";
      convert_image(img_path, 0, 1, 25, 25, data);
   }*/
  
}

void DetectCone::tearDown()
{
}

void DetectCone::nextContainer(odcore::data::Container &a_container)
{
  odcore::data::TimeStamp incommingDataTime = a_container.getSampleTimeStamp();
  double currentTime = static_cast<double>(incommingDataTime.toMicroseconds())/1000000.0;
  std::cout << "Current time: " << currentTime << "s" << std::endl;

  if (a_container.getDataType() == odcore::data::image::SharedImage::ID()) {
    odcore::data::image::SharedImage sharedImg =
        a_container.getData<odcore::data::image::SharedImage>();
    if (!ExtractSharedImage(&sharedImg)) {
      std::cout << "[" << getName() << "] " << "[Unable to extract shared image."
          << std::endl;
      return;
    }
    
    // saveImg(currentTime);
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

void DetectCone::saveImg(double currentTime) {
  std::string img_name = std::to_string(currentTime);
  cv::imwrite("recording/"+img_name+".png", m_img);
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
    350.6847, 0, 332.4661, 
    0, 350.0606, 163.7461, 
    0, 0, 1);
  cv::Mat distLeft = (cv::Mat_<double>(5, 1) << -0.1674, 0.0158, 0.0057, 0, 0);
  cv::Mat mtxRight = (cv::Mat_<double>(3, 3) << 
    351.9498, 0, 329.4456,
    0, 351.0426, 179.0179,
    0, 0, 1);
  cv::Mat distRight = (cv::Mat_<double>(5, 1) << -0.1700, 0.0185, 0.0048, 0, 0);
  cv::Mat R = (cv::Mat_<double>(3, 3) << 
    0.9997, 0.0015, 0.0215,
    -0.0015, 1, -0.00008,
    -0.0215, 0.00004, 0.9997);
  //cv::transpose(R, R);
  cv::Mat T = (cv::Mat_<double>(3, 1) << -119.1807, 0.1532, 1.1225);
  //double F = 350;
  //double d = 120;
  //double factor = F * d;

  cv::Size stdSize = cv::Size(640, 360);
  cv::Mat img = cv::imread("1.png");
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

  //cv::imwrite("2_left.png", imgL);
  //cv::imwrite("2_right.png", imgR);

  cv::Mat rectify, disp, result;
  rectify = imgL + imgR;
  disp = blockMatching(imgL, imgR);

  cv::namedWindow("disp", cv::WINDOW_NORMAL);
  cv::imshow("disp", disp);
  cv::waitKey(0);
  cv::destroyAllWindows();
}

cv::Mat DetectCone::blockMatching(cv::Mat imgL, cv::Mat imgR){
  cv::Mat grayL, grayR, disp;

  cv::cvtColor(imgL, grayL, CV_BGR2GRAY);
  cv::cvtColor(imgR, grayR, CV_BGR2GRAY);

  cv::StereoBM sbm;
  sbm.state->SADWindowSize = 17;
  sbm.state->numberOfDisparities = 32;
  
  sbm(grayL, grayR, disp);
  cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
  return disp;
}

//run cnn starts
void DetectCone::convert_image(const std::string &imagefilename,
                   double minv,
                   double maxv,
                   int w,
                   int h,
                   tiny_dnn::vec_t &data) {
  tiny_dnn::image<> img(imagefilename, tiny_dnn::image_type::rgb);
  tiny_dnn::image<> resized = tiny_dnn::resize_image(img, w, h);
  data.resize(resized.width() * resized.height() * resized.depth());
  for (size_t c = 0; c < resized.depth(); ++c) {
    for (size_t y = 0; y < resized.height(); ++y) {
      for (size_t x = 0; x < resized.width(); ++x) {
        data[c * resized.width() * resized.height() + y * resized.width() + x] =
          (maxv - minv) * (resized[y * resized.width() + x + c]) / 255.0 + minv;
      }
    }
  }
}

// double DetectCone::rescale(double x) {
//   return 100.0 * (x + 1) / 2;
// }

void DetectCone::run_cnn(const std::string &dictionary, const std::string &src_filename) {
  using conv    = tiny_dnn::convolutional_layer;
  // using dropout = tiny_dnn::dropout_layer;
  using pool    = tiny_dnn::max_pooling_layer;
  using fc      = tiny_dnn::fully_connected_layer;
  using relu    = tiny_dnn::relu_layer;
  using softmax = tiny_dnn::softmax_layer;

  const size_t n_fmaps  = 32;  ///< number of feature maps for upper layer
  const size_t n_fmaps2 = 64;  ///< number of feature maps for lower layer
  const size_t n_fc = 64;  ///< number of hidden units in fully-connected layer

  tiny_dnn::network<tiny_dnn::sequential> nn;

  nn << conv(32, 32, 5, 3, n_fmaps, tiny_dnn::padding::same)  // C1
     << pool(32, 32, n_fmaps, 2)                              // P2
     << relu(16, 16, n_fmaps)                                 // activation
     << conv(16, 16, 5, n_fmaps, n_fmaps, tiny_dnn::padding::same)  // C3
     << pool(16, 16, n_fmaps, 2)                                    // P4
     << relu(8, 8, n_fmaps)                                        // activation
     << conv(8, 8, 5, n_fmaps, n_fmaps2, tiny_dnn::padding::same)  // C5
     << pool(8, 8, n_fmaps2, 2)                                    // P6
     << relu(4, 4, n_fmaps2)                                       // activation
     << fc(4 * 4 * n_fmaps2, n_fc)                                 // FC7
     << fc(n_fc, 3) << softmax(3);                               // FC10

  // load nets
  std::ifstream ifs(dictionary.c_str());
  ifs >> nn;

  // convert imagefile to vec_t
  tiny_dnn::vec_t data;
  convert_image(src_filename, 0, 1.0, 32, 32, data);

  // recognize
  auto res = nn.predict(data);
  for(uint i=0; i<res.size(); i++){
    std::cout << res[i] << std::endl;
  }
}
//run cnn ends



}
}
}
}
