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

#include "detectcone.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace perception {

DetectCone::DetectCone(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "logic-cfsd18-perception-detectcone")
, m_lastLidarData()
, m_lastCameraData()
, m_pointMatched()
, m_diffVec()
, m_finalPointCloud()
, m_threshold()
, m_timeDiffMilliseconds()
, m_lastTimeStamp()
, m_coneCollector()
, m_lastObjectId()
, m_img()
, m_dictionary("slidingWindow")
, m_patchSize(32)
{
  m_diffVec = 0;
  m_pointMatched = Eigen::MatrixXd::Zero(4,1);
  m_lastCameraData = Eigen::MatrixXd::Zero(4,1);
  m_lastLidarData = Eigen::MatrixXd::Zero(4,1);
  m_coneCollector = Eigen::MatrixXd::Zero(4,10);
  m_lastObjectId = 0;
}

DetectCone::~DetectCone()
{
  m_img.release();
}

void DetectCone::setUp()
{
  auto kv = getKeyValueConfiguration();
  m_threshold = kv.getValue<double>("logic-cfsd18-perception-detectcone.threshold");
  m_timeDiffMilliseconds = kv.getValue<double>("logic-cfsd18-perception-detectcone.timeDiffMilliseconds");
  std::cout << "setup OK " << std::endl;

  // testSlidingWindow("0.png");
}

void DetectCone::tearDown()
{
}

////save image with time stamp
// void DetectCone::nextContainer(odcore::data::Container &a_container)
// {
//   odcore::data::TimeStamp incommingDataTime = a_container.getSampleTimeStamp();
//   double currentTime = static_cast<double>(incommingDataTime.toMicroseconds())/1000000.0;
//   std::cout << "Current time: " << currentTime << "s" << std::endl;
//
//   if (a_container.getDataType() == odcore::data::image::SharedImage::ID()) {
//     odcore::data::image::SharedImage sharedImg =
//         a_container.getData<odcore::data::image::SharedImage>();
//     if (!ExtractSharedImage(&sharedImg)) {
//       std::cout << "[" << getName() << "] " << "[Unable to extract shared image."
//           << std::endl;
//       return;
//     }
//     saveImg(currentTime);
//   }
// }

void DetectCone::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == odcore::data::image::SharedImage::ID()) {
    odcore::data::image::SharedImage sharedImg =
        a_container.getData<odcore::data::image::SharedImage>();
    if (!ExtractSharedImage(&sharedImg)) {
      std::cout << "[" << getName() << "] " << "[Unable to extract shared image." << std::endl;
      return;
    }
  }
  //Marcus
  if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID()) {
    //Retrive data and timestamp
    odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
		uint32_t objectId = coneDirection.getObjectId();

    //Check last timestamp if they are from same message
    std::cout << "Message Recieved " << std::endl;
    if (CheckContainer(objectId,timeStamp)){
      std::cout << "Test 2 " << std::endl;
      m_coneCollector(objectId,0) = coneDirection.getAzimuthAngle();
			m_coneCollector(objectId,1) = coneDirection.getZenithAngle();
    }
  }
	if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()){
		odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
		uint32_t objectId = coneDistance.getObjectId();

    //Check last timestamp if they are from same message
    std::cout << "Message Recieved " << std::endl;
    if (CheckContainer(objectId,timeStamp)){
      m_coneCollector(objectId,2) = coneDistance.getDistance();
			m_coneCollector(objectId,3) = 0;
    }
  }
  //Marcus
}

bool DetectCone::CheckContainer(uint32_t objectId, odcore::data::TimeStamp timeStamp){
		if (((timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds()) < m_timeDiffMilliseconds*1000)){
      std::cout << "Test 2 " << std::endl;
      m_lastObjectId = (m_lastObjectId<objectId)?(objectId):(m_lastObjectId);
      m_lastTimeStamp = timeStamp;
      std::cout << "FoundCone: " << std::endl;
      std::cout << m_coneCollector << std::endl;

    }
    else{
      //All object candidates collected, to sensor fusion
      std::cout << "Extracted Cones " << std::endl;
      Eigen::MatrixXd extractedCones;
      extractedCones = m_coneCollector.leftCols(m_lastObjectId);
      std::cout << extractedCones << std::endl;
      SendCollectedCones(extractedCones);

      //Initialize for next collection
      m_lastTimeStamp = timeStamp;
      m_lastObjectId = 0;
      m_coneCollector = Eigen::MatrixXd::Zero(4,10);
    }
		return true;
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
  std::string imgName = std::to_string(currentTime);
  cv::imwrite("recording/"+imgName+".png", m_img);
}

void DetectCone::featureBased() {
  cv::Mat m_hsv, channel[3];
  cv::cvtColor(m_img, m_hsv, CV_RGB2HSV);
  cv::split(m_hsv, channel);
  //cv::adaptiveThreshold(adapThreshImg, channel[1], 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, m_adapThreshKernelSize, m_adapThreshConst);
  cv::imshow("img", channel[0]);
  cv::waitKey(0);
}

void DetectCone::blockMatching(cv::Mat &disp, cv::Mat imgL, cv::Mat imgR){
  cv::Mat grayL, grayR;

  cv::cvtColor(imgL, grayL, CV_BGR2GRAY);
  cv::cvtColor(imgR, grayR, CV_BGR2GRAY);

  cv::StereoBM sbm;
  sbm.state->SADWindowSize = 17;
  sbm.state->numberOfDisparities = 32;

  sbm(grayL, grayR, disp);
  cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
}

void DetectCone::reconstruction(cv::Mat img, cv::Mat &Q, cv::Mat &disp, cv::Mat &rectified, cv::Mat &XYZ){
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

  cv::Size stdSize = cv::Size(640, 360);
  int width = img.cols;
  int height = img.rows;
  cv::Mat imgL(img, cv::Rect(0, 0, width/2, height));
  cv::Mat imgR(img, cv::Rect(width/2, 0, width/2, height));

  cv::resize(imgL, imgL, stdSize);
  cv::resize(imgR, imgR, stdSize);

  //std::cout << imgR.size() <<std::endl;

  cv::Mat R1, R2, P1, P2;
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

  blockMatching(disp, imgL, imgR);
  rectified = imgL;

  cv::reprojectImageTo3D(disp, XYZ, Q);
}

void DetectCone::xyz2xy(cv::Mat Q, cv::Vec3f xyz, cv::Vec2f &xy){
  double X = xyz[0];
  double Y = xyz[1];
  double Z = xyz[2];
  double Cx = -Q.at<double>(0,3);
  double Cy = -Q.at<double>(1,3);
  double f = Q.at<double>(2,3);
  double a = Q.at<double>(3,2);
  double b = Q.at<double>(3,3);
  double d = (f - Z * b ) / ( Z * a);
  xy[0] = X * ( d * a + b ) + Cx;
  xy[1] = Y * ( d * a + b ) + Cy;
}

float_t DetectCone::depth2resizeRate(double x, double y){
  return 1.6078-0.6767*std::sqrt(std::sqrt(x*x+y*y)/1000);
}


void DetectCone::convertImage(cv::Mat img, int w, int h, tiny_dnn::vec_t &data){
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(w, h));
  data.resize(w * h * 3);
  for (int c = 0; c < 3; ++c) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
       data[c * w * h + y * w + x] =
         resized.at<cv::Vec3b>(y, x)[c] / 255.0;
      }
    }
  }
}

// void DetectCone::testSlidingWindow(const std::string &imgPath){
//   cv::Mat img = cv::imread(imgPath);
//   // cv::Vec3f point3D;
//   // point3D << 1182.67, 166.36, 1750.78;
//   // backwardDetection(img, point3D);
//   forwardDetection(img);
// }

void DetectCone::loadCNN(const std::string &dictionary, tiny_dnn::network<tiny_dnn::sequential> &nn) {
  using conv    = tiny_dnn::convolutional_layer;
  using pool    = tiny_dnn::max_pooling_layer;
  using fc      = tiny_dnn::fully_connected_layer;
  using relu    = tiny_dnn::relu_layer;
  using softmax = tiny_dnn::softmax_layer;

  const size_t n_fmaps  = 32;  // number of feature maps for upper layer
  const size_t n_fmaps2 = 64;  // number of feature maps for lower layer
  const size_t n_fc     = 64;  // number of hidden units in fc layer
  tiny_dnn::core::backend_t backend_type = tiny_dnn::core::default_engine();

  nn << conv(m_patchSize, m_patchSize, 5, 3, n_fmaps, tiny_dnn::padding::same, true, 1, 1,
             backend_type)                      // C1
     << pool(32, 32, n_fmaps, 2, backend_type)  // P2
     << relu()                                  // activation
     << conv(16, 16, 5, n_fmaps, n_fmaps, tiny_dnn::padding::same, true, 1, 1,
             backend_type)                      // C3
     << pool(16, 16, n_fmaps, 2, backend_type)  // P4
     << relu()                                  // activation
     << conv(8, 8, 5, n_fmaps, n_fmaps2, tiny_dnn::padding::same, true, 1, 1,
             backend_type)                                // C5
     << pool(8, 8, n_fmaps2, 2, backend_type)             // P6
     << relu()                                            // activation
     << fc(4 * 4 * n_fmaps2, n_fc, true, backend_type)    // FC7
     << relu()                                            // activation
     << fc(n_fc, 4, true, backend_type) << softmax(4);  // FC10

  // load nets
  std::ifstream ifs(dictionary.c_str());
  ifs >> nn;
}

int DetectCone::forwardDetection(cv::Mat img){
  //Detect cone in camera frame and then project to 3D world

  // manual roi
  // (453, 237,	0.96875,	"orange", 319.105, 172.883, 1033.59);
  // (585, 211,	0.625,	"orange", 1182.67, 166.36, 1750.78);
  // (343, 185,	0.25,	"yellow", 10.183, 75.1519, 3299.55);
  // (521, 198,	0.375,	"yellow", 1219.86, 144.365, 2451.1);
  // (625,	191,	0.34375,	"blue", 2499.47, 125.34, 3177.35);
  // (396,	183,	0.34375,	"blue", 586.295, 67.1457, 3899.47);

  // convert imagefile to vec_t
  std::cout << "image size: " << img.size() << std::endl;
  cv::Mat Q, disp, rectified, XYZ;
  reconstruction(img, Q, disp, rectified, XYZ);

  tiny_dnn::network<tiny_dnn::sequential> nn;
  loadCNN(m_dictionary, nn);

  // manual roi
  // (453, 237, 0.96875,  "orange");
  // (585, 211, 0.625,  "orange");
  // (343, 185, 0.25, "yellow");
  // (521, 198, 0.375,  "yellow");
  // (634,  202,  0.375,  "yellow");
  // (625,  191,  0.34375,  "blue");
  // (396,  183,  0.34375,  "blue");

  int x = 453;
  int y = 237;
  float_t ratio = depth2resizeRate(319.105, 1033.59);
  int length = ratio * m_patchSize;
  int radius = (length-1)/2;

  cv::Vec2f point2D;
  point2D << x, y;
  cv::Vec3f point3D = XYZ.at<cv::Vec3f>(y,x) * 2;

  cv::Rect roi;
  roi.x = std::max(x - radius, 0);
  roi.y = std::max(y - radius, 0);
  roi.width = std::min(x + radius, img.cols) - roi.x;
  roi.height = std::min(y + radius, img.rows) - roi.y;
  auto patchImg = rectified(roi);

  tiny_dnn::vec_t data;
  convertImage(patchImg, m_patchSize, m_patchSize, data);
  auto prob = nn.predict(data);
  float_t threshold = 0.5;
  // std::cout << prob[0] << " " << prob[1] << " " << prob[2] << " " << prob[3] << std::endl;
  int maxIndex = 1;
  float_t maxProb = prob[1];
  for(int i=2;i<4;i++){
    if(prob[i]>prob[maxIndex]){
      maxIndex = i;
      maxProb = prob[i];
    }
  }

  std::string labels[] = {"yellow", "blue", "orange"};
  if (maxProb < threshold)
    std::cout << "No cone detected" << std::endl;
  else
    std::cout << "Find one " << labels[maxIndex-1] << " cone, XYZ positon: "
    << point3D << "mm, xy position: " << point2D << "pixel, certainty: " << maxProb << std::endl;

  // cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,0,0));
  // // cv::circle(disp, cv::Point (x,y), 3, 0, CV_FILLED);
  // cv::namedWindow("disp", cv::WINDOW_NORMAL);
  // cv::imshow("disp", rectified);
  // cv::waitKey(0);
  // cv::destroyAllWindows();

  return maxIndex;
}

int DetectCone::backwardDetection(cv::Mat img, cv::Vec3f point3D){
  //Given RoI in 3D world, project back to the camera frame and then detect

  // manual roi
  // (453, 237,	0.96875,	"orange", 319.105, 172.883, 1033.59);
  // (585, 211,	0.625,	"orange", 1182.67, 166.36, 1750.78);
  // (343, 185,	0.25,	"yellow", 10.183, 75.1519, 3299.55);
  // (521, 198,	0.375,	"yellow", 1219.86, 144.365, 2451.1);
  // (625,	191,	0.34375,	"blue", 2499.47, 125.34, 3177.35);
  // (396,	183,	0.34375,	"blue", 586.295, 67.1457, 3899.47);

  // convert imagefile to vec_t
  std::cout << "image size: " << img.size() << std::endl;
  cv::Mat Q, disp, rectified, XYZ;
  reconstruction(img, Q, disp, rectified, XYZ);

  tiny_dnn::network<tiny_dnn::sequential> nn;
  loadCNN(m_dictionary, nn);
  cv::Vec2f point2D;
  xyz2xy(Q, point3D, point2D);

  int x = point2D[0];
  int y = point2D[1];
  float_t ratio = depth2resizeRate(point3D[0], point3D[2]);
  int length = ratio * m_patchSize;
  int radius = (length-1)/2;

  cv::Rect roi;
  roi.x = std::max(x - radius, 0);
  roi.y = std::max(y - radius, 0);
  roi.width = std::min(x + radius, img.cols) - roi.x;
  roi.height = std::min(y + radius, img.rows) - roi.y;
  auto patchImg = rectified(roi);

  tiny_dnn::vec_t data;
  convertImage(patchImg, m_patchSize, m_patchSize, data);
  auto prob = nn.predict(data);
  float_t threshold = 0.5;
  // std::cout << prob[0] << " " << prob[1] << " " << prob[2] << " " << prob[3] << std::endl;
  int maxIndex = 1;
  float_t maxProb = prob[1];
  for(int i=2;i<4;i++){
    if(prob[i]>prob[maxIndex]){
      maxIndex = i;
      maxProb = prob[i];
    }
  }

  std::string labels[] = {"yellow", "blue", "orange"};
  if (maxProb < threshold)
    std::cout << "No cone detected" << std::endl;
  else
    std::cout << "Find one " << labels[maxIndex-1] << " cone, XYZ positon: " << point3D << "mm, xy position: " << point2D << "pixel" << std::endl;
  // cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,0,0), CV_FILLED);
  // // cv::circle(disp, cv::Point (x,y), 3, 0, CV_FILLED);
  // cv::namedWindow("disp", cv::WINDOW_NORMAL);
  // cv::imshow("disp", rectified);
  // cv::waitKey(0);
  // cv::destroyAllWindows();
  return maxIndex;
}
//run cnn ends

//Marcus
// void DetectCone::matchPoints(Eigen::MatrixXd lidar, Eigen::MatrixXd camera)
// {
//   //Index vars
//   int colFinalPoints = 1;
//   int index = 0;
//   //Initialize zero matricies
//   Eigen::MatrixXd tempPointLidar = Eigen::MatrixXd::Zero(4,1);
//   Eigen::MatrixXd tempPointCamera = Eigen::MatrixXd::Zero(4,1);
//   m_finalPointCloud = Eigen::MatrixXd::Zero(4,lidar.cols());
//   Eigen::MatrixXd diffVec = Eigen::MatrixXd::Zero(1,lidar.cols());
//
//   //Pick i:th found lidar object
//   for (int i = 0; i < lidar.cols(); i++){
//     //Reset match check for each lidar point
//     bool matchFound = false;
//     //Loop through all found camera objects
//     for (int j = 0; j < camera.cols(); j++){
//       //store in temporary variables as input in findMatch
//       tempPointLidar.col(0) = lidar.col(i);
//       tempPointCamera.col(0) = camera.col(j);
//
//       findMatch(tempPointLidar, tempPointCamera);
//       //Store range difference of i lidar point, j camera point
//       diffVec(0,j) = m_diffVec;
//
//     }
//     //Reset
//     m_diffVec=1000000;
//
//     //Iterate through all points to find the closest camera object j to lidar object to current i
//     for (int k = 0; k < diffVec.cols(); k++){
//
//       if (m_diffVec > diffVec(0,k) && diffVec(0,k) > 0 ){
//         m_diffVec = diffVec(0,k);
//         index = k;
//         matchFound = true;
//       }
//
//     }
//
//     //If no match is found, store object as a cone without classification, else use index found to classify cone
//     if(!matchFound) {
//       m_finalPointCloud(0,colFinalPoints-1) = lidar(0,i);
//       m_finalPointCloud(1,colFinalPoints-1) = lidar(1,i);
//       m_finalPointCloud(2,colFinalPoints-1) = lidar(2,i);
//       m_finalPointCloud(3,colFinalPoints-1) = 0;
//       colFinalPoints++;
//     }
//     else{
//       m_finalPointCloud(0,colFinalPoints-1) = lidar(0,i);
//       m_finalPointCloud(1,colFinalPoints-1) = lidar(1,i);
//       m_finalPointCloud(2,colFinalPoints-1) = lidar(2,i);
//       m_finalPointCloud(3,colFinalPoints-1) = camera(3,index);
//       colFinalPoints++;
//     }
//   }
// }

// void DetectCone::findMatch(Eigen::MatrixXd lidarPoint, Eigen::MatrixXd cameraPoint)
// {
//   //Calculate distance between lidar and camera points
//   Eigen::MatrixXd tempNorm = Eigen::MatrixXd::Zero(2,1);
//   tempNorm(0,0) = lidarPoint(0,0)-cameraPoint(0,0);
//   tempNorm(1,0) = lidarPoint(1,0)-cameraPoint(1,0);
//   m_diffVec = tempNorm.norm();
//
//   //below threshold results in point added
//   if(m_diffVec < m_threshold){
//       m_pointMatched(0,0) = lidarPoint(0,0);
//       m_pointMatched(1,0) = lidarPoint(1,0);
//       m_pointMatched(2,0) = lidarPoint(2,0);
//       m_pointMatched(3,0) = cameraPoint(3,0);
//   }
//   else{
//     m_pointMatched = Eigen::MatrixXd::Zero(4,1);
//     m_diffVec = 0;
//   }
// }

void DetectCone::Spherical2Cartesian(double azimuth, double zenimuth, double distance, Eigen::MatrixXd &recievedPoint)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  double xData = distance * sin(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * sin(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * cos(zenimuth * static_cast<double>(DEG2RAD));
  recievedPoint << xData,
                    yData,
                    zData,
                    0;
}

void DetectCone::Cartesian2Spherical(double x, double y, double z, opendlv::logic::sensation::Point &pointInSpherical)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan(y/x)*static_cast<double>(RAD2DEG);
  double zenithAngle = atan(sqrt(x*x+y*y)/z)*static_cast<double>(RAD2DEG);
  pointInSpherical.setDistance(distance);
  pointInSpherical.setAzimuthAngle(azimuthAngle);
  pointInSpherical.setZenithAngle(zenithAngle);
}

void DetectCone::SendCollectedCones(Eigen::MatrixXd lidarCones)
{
  //Convert to cartesian
  Eigen::MatrixXd cone;
  for(int p = 0; p < lidarCones.cols(); p++){
    Spherical2Cartesian(lidarCones(0,p), lidarCones(1,p), lidarCones(2,p), cone);
    lidarCones.col(p) = cone;
  }
  std::cout << "lidarCones " << std::endl;
  std::cout << lidarCones << std::endl;
  m_finalPointCloud = lidarCones;
  cv::Vec3f point3D;
  double xShift = 0;//1872mm
  for (int i = 0; i < m_finalPointCloud.cols(); i++){
    point3D << -m_finalPointCloud(1,i), -m_finalPointCloud(2,i), xShift+m_finalPointCloud(0,i);
    m_finalPointCloud(3,i) = backwardDetection(m_img, point3D);
  }

  std::cout << "matched: " << std::endl;
  std::cout << m_finalPointCloud << std::endl;
  SendMatchedContainer(m_finalPointCloud);
}

// void DetectCone::SendCollectedCones(Eigen::MatrixXd lidarCones)
// {
//   //Convert to cartesian
//   for(int p = 0; p < lidarCones.cols(); p++){
//     lidarCones.col(p) = Spherical2Cartesian(lidarCones(0,p), lidarCones(1,p), lidarCones(2,p));
//   }
//   std::cout << "lidarCones " << std::endl;
//   std::cout << lidarCones << std::endl;
//   Eigen::MatrixXd cameraCones = Eigen::MatrixXd::Zero(4,2);
//   cameraCones << 1.7,  5.6,
//                  1.8,  2.4,
//                  0.1, 0,
//                  1, 2;
//
//   std::cout << "CameraCones " << std::endl;
//   std::cout << cameraCones << std::endl;
//   matchPoints(lidarCones, cameraCones);
//   std::cout << "matched: " << std::endl;
//   std::cout << m_finalPointCloud << std::endl;
//
//   SendMatchedContainer(m_finalPointCloud);
// }

void DetectCone::SendMatchedContainer(Eigen::MatrixXd cones)
{
  for(int n = 0; n < cones.cols(); n++){
    opendlv::logic::sensation::Point conePoint;
    Cartesian2Spherical(cones(0,n), cones(1,n), cones(2,n), conePoint);
    odcore::data::Container c1(conePoint);
    getConference().send(c1);
    std::cout << "a point sent out with distance: " <<conePoint.getDistance() <<"; azimuthAngle: " << conePoint.getAzimuthAngle() << "; and zenithAngle: " << conePoint.getZenithAngle() << std::endl;
  }
}
//Marcus

}
}
}
}
