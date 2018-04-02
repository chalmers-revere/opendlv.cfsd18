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
, m_coneMutex()
, m_recievedFirstImg(false)
, m_img()
, m_nn()
, m_lidarIsWorking(false)
, m_checkLiarMilliseconds()
{
  m_diffVec = 0;
  m_pointMatched = Eigen::MatrixXd::Zero(4,1);
  m_lastCameraData = Eigen::MatrixXd::Zero(4,1);
  m_lastLidarData = Eigen::MatrixXd::Zero(4,1);
  m_coneCollector = Eigen::MatrixXd::Zero(4,20);
  m_lastObjectId = 0;
}

DetectCone::~DetectCone()
{
  m_img.release();
  cv::destroyAllWindows();
}

void DetectCone::setUp()
{
  auto kv = getKeyValueConfiguration();
  m_threshold = kv.getValue<double>("logic-cfsd18-perception-detectcone.threshold");
  m_timeDiffMilliseconds = kv.getValue<double>("logic-cfsd18-perception-detectcone.timeDiffMilliseconds");
  m_checkLiarMilliseconds = kv.getValue<double>("logic-cfsd18-perception-detectcone.checkLidarMilliseconds");
  m_senderStamp = kv.getValue<int>("logic-cfsd18-perception-detectcone.senderStamp");
  m_attentionSenderStamp = kv.getValue<int>("logic-cfsd18-perception-detectcone.attentionSenderStamp");
  std::cout << "setup OK " << std::endl;

  // slidingWindow("slidingWindow");

  efficientSlidingWindow("efficientSlidingWindow", 320, 60);
}

void DetectCone::tearDown()
{
}

void DetectCone::nextContainer(odcore::data::Container &a_container)
{
  odcore::data::TimeStamp startTime;
  forwardDetection();
  odcore::data::TimeStamp endTime;
  double timeElapsed = abs(static_cast<double>(endTime.toMicroseconds()-startTime.toMicroseconds())/1000000.0);
  std::cout << "Time elapsed for camera detection: " << timeElapsed << std::endl;

  if (a_container.getDataType() == odcore::data::image::SharedImage::ID()) {
    odcore::data::image::SharedImage sharedImg =
    a_container.getData<odcore::data::image::SharedImage>();
    odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    if (!ExtractSharedImage(&sharedImg)) {
      std::cout << "[" << getName() << "] " << "[Unable to extract shared image." << std::endl;
      return;
    }else if(!m_recievedFirstImg){
      m_recievedFirstImg = true;
    }
    if (((timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds()) > m_checkLiarMilliseconds*1000)){
      std::cout << "Lidar fails! Camera detection only! " << std::endl;
      m_lidarIsWorking = false;
    }
    else{
      m_lidarIsWorking = true;
      std::cout << "Lidar is working!" << std::endl;
    }
    // if(!m_lidarIsWorking){
    //   forwardDetection(m_img);
    // }
  }
  bool correctSenderStamp = a_container.getSenderStamp() == m_attentionSenderStamp;
  if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID() && correctSenderStamp) {
    odcore::base::Lock lockCone(m_coneMutex);
    std::cout << "Recieved Direction" << std::endl;
    //Retrive data and timestamp
    odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
		uint32_t objectId = coneDirection.getObjectId();

    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (CheckContainer(objectId,timeStamp)){
      //std::cout << "Test 2 " << std::endl;
      m_coneCollector(0,objectId) = coneDirection.getAzimuthAngle();
			m_coneCollector(1,objectId) = coneDirection.getZenithAngle();
    }

  }
  else if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID() && correctSenderStamp){
    odcore::base::Lock lockCone(m_coneMutex);
    std::cout << "Recieved Distance" << std::endl;
		odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
		uint32_t objectId = coneDistance.getObjectId();

    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (CheckContainer(objectId,timeStamp)){
      m_coneCollector(2,objectId) = coneDistance.getDistance();
			m_coneCollector(3,objectId) = 0;
    }

  }
}

bool DetectCone::CheckContainer(uint32_t objectId, odcore::data::TimeStamp timeStamp){
		if (((timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds()) < m_timeDiffMilliseconds*1000)){
      //std::cout << "Test 2 " << std::endl;
      m_lastObjectId = (m_lastObjectId<objectId)?(objectId):(m_lastObjectId);
      m_lastTimeStamp = timeStamp;
      //std::cout << "FoundCone: " << std::endl;
      //std::cout << m_coneCollector << std::endl;

    }
    else {
      //All object candidates collected, to sensor fusion
      Eigen::MatrixXd extractedCones;
      extractedCones = m_coneCollector.leftCols(m_lastObjectId+1);
      if(extractedCones.cols() > 1){
      std::cout << "Extracted Cones " << std::endl;
      std::cout << extractedCones << std::endl;
        if(m_recievedFirstImg){
          SendCollectedCones(extractedCones);
        }
      }
      //Initialize for next collection
      m_lastTimeStamp = timeStamp;
      m_lastObjectId = 0;
      m_coneCollector = Eigen::MatrixXd::Zero(4,20);
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
  // cv::namedWindow("disp", cv::WINDOW_NORMAL);
  // cv::imshow("disp", disp);
  // cv::waitKey(0);

  rectified = imgL;

  cv::reprojectImageTo3D(disp, XYZ, Q);
  XYZ *= 2;
}

void DetectCone::xyz2xy(cv::Mat Q, cv::Vec3f xyz, cv::Vec2f &xy){
  double X = xyz[0] / 2;
  double Y = xyz[1] / 2;
  double Z = xyz[2] / 2;
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
  return 2*(1.6078-0.4785*std::sqrt(std::sqrt(x*x+y*y)/1000));
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

void DetectCone::slidingWindow(const std::string &dictionary) {
  using conv    = tiny_dnn::convolutional_layer;
  using pool    = tiny_dnn::max_pooling_layer;
  using fc      = tiny_dnn::fully_connected_layer;
  using relu    = tiny_dnn::relu_layer;
  using softmax = tiny_dnn::softmax_layer;

  const int n_fmaps  = 32;  // number of feature maps for upper layer
  const int n_fmaps2 = 64;  // number of feature maps for lower layer
  const int n_fc     = 64;  // number of hidden units in fc layer
  tiny_dnn::core::backend_t backend_type = tiny_dnn::core::default_engine();

  m_nn << conv(32, 32, 5, 3, n_fmaps, tiny_dnn::padding::same, true, 1, 1,
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
  ifs >> m_nn;
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
  // std::cout << "image size: " << img.size() << std::endl;
  cv::Mat Q, disp, rectified, XYZ;
  reconstruction(img, Q, disp, rectified, XYZ);

  cv::Vec2f point2D;
  xyz2xy(Q, point3D, point2D);

  int x = point2D[0];
  int y = point2D[1];

  // std::cout << "Camera region center: " << x << ", " << y << std::endl;
  float_t ratio = depth2resizeRate(point3D[0], point3D[2]);
  int maxIndex = 1;
  if (ratio > 0) {
    int length = ratio * 32;
    int radius = (length-1)/2;
    // std::cout << "radius: " << radius << std::endl;

    cv::Rect roi;
    roi.x = std::max(x - radius, 0);
    roi.y = std::max(y - radius, 0);
    roi.width = std::min(x + radius, rectified.cols) - roi.x;
    roi.height = std::min(y + radius, rectified.rows) - roi.y;

    //cv::circle(img, cv::Point (x,y), radius, cv::Scalar (0,0,0));
    // // cv::circle(disp, cv::Point (x,y), 3, 0, CV_FILLED);
    //cv::namedWindow("roi", cv::WINDOW_NORMAL);
    //cv::imshow("roi", img);
    //cv::waitKey(0);
    //cv::destroyAllWindows();
    if (0 > roi.x || 0 > roi.width || roi.x + roi.width > rectified.cols || 0 > roi.y || 0 > roi.height || roi.y + roi.height > rectified.rows){
      std::cout << "Wrong roi!" << std::endl;
      return 0;
    }
    auto patchImg = rectified(roi);

    tiny_dnn::vec_t data;
    convertImage(patchImg, 32, 32, data);
    auto prob = m_nn.predict(data);
    float_t threshold = 0.1;
    // std::cout << prob[0] << " " << prob[1] << " " << prob[2] << " " << prob[3] << std::endl;
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
    else{
      std::cout << "Find one " << labels[maxIndex-1] << " cone, XYZ positon: " << point3D << "mm, xy position: " << point2D << "pixel, certainty: " << prob[maxIndex] << std::endl;
      if (labels[maxIndex-1] == "yellow")
        cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,255,255), CV_FILLED);
      else if (labels[maxIndex-1] == "blue")
        cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (255,0,0), CV_FILLED);
      else if (labels[maxIndex-1] == "orange")
        cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,165,255), CV_FILLED);
      else
        cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,0,0), CV_FILLED);
    }
       // cv::circle(disp, cv::Point (x,y), 3, 0, CV_FILLED);
       cv::namedWindow("disp", cv::WINDOW_NORMAL);
       // cv::setWindowProperty("result", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
       cv::imshow("disp", rectified);
       cv::waitKey(10);
       //cv::destroyAllWindows();
  }
  else{
    std::cout << "No cone detected" << std::endl;
  }
  return maxIndex;
}

void DetectCone::efficientSlidingWindow(const std::string &dictionary, int width, int height) {
  using conv    = tiny_dnn::convolutional_layer;
  using relu    = tiny_dnn::relu_layer;
  tiny_dnn::core::backend_t backend_type = tiny_dnn::core::backend_t::avx;

  // m_nn << conv(width, height, 7, 3, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    << conv(width-6, height-6, 7, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    // << dropout((input_size-12)*(input_size-12)*16, 0.25)
  //    << conv(width-12, height-12, 7, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    << conv(width-18, height-18, 7, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    // << dropout((input_size-24)*(input_size-24)*16, 0.25)
  //    << conv(width-24, height-24, 5, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    << conv(width-28, height-28, 5, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    // << dropout((input_size-32)*(input_size-32)*16, 0.25)
  //    << conv(width-32, height-32, 5, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    << conv(width-36, height-36, 5, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    // << dropout((input_size-40)*(input_size-40)*16, 0.25)
  //    << conv(width-40, height-40, 3, 16, 128, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
  //    << conv(width-42, height-42, 3, 128, 4, tiny_dnn::padding::valid, true, 1, 1, backend_type);

  m_nn << conv(width, height, 7, 3, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-6, height-6, 7, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-12, height-12, 5, 16, 32, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-16, height-16, 5, 32, 32, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-20, height-20, 3, 32, 64, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-22, height-22, 3, 64, 4, tiny_dnn::padding::valid, true, 1, 1, backend_type);

  // load nets
  std::ifstream ifs(dictionary.c_str());
  ifs >> m_nn;
}

void DetectCone::softmax(cv::Vec4d x, cv::Vec4d &y) {
  double min, max, denominator = 0;
  cv::minMaxLoc(x, &min, &max);
  for (int j = 0; j < 4; j++) {
    y[j] = std::exp(x[j] - max);
    denominator += y[j];
  }
  for (int j = 0; j < 4; j++) {
    y[j] /= denominator;
  }
}

std::vector <cv::Point> DetectCone::imRegionalMax(cv::Mat input, int nLocMax, double threshold, int minDistBtwLocMax)
{
    cv::Mat scratch = input.clone();
    // std::cout<<scratch<<std::endl;
    cv::GaussianBlur(scratch, scratch, cv::Size(3,3), 0, 0);
    std::vector <cv::Point> locations(0);
    locations.reserve(nLocMax); // Reserve place for fast access
    for (int i = 0; i < nLocMax; i++) {
        cv::Point location;
        double maxVal;
        cv::minMaxLoc(scratch, NULL, &maxVal, NULL, &location);
        if (maxVal > threshold) {
            int row = location.y;
            int col = location.x;
            locations.push_back(cv::Point(col, row));
            int r0 = (row-minDistBtwLocMax > -1 ? row-minDistBtwLocMax : 0);
            int r1 = (row+minDistBtwLocMax < scratch.rows ? row+minDistBtwLocMax : scratch.rows-1);
            int c0 = (col-minDistBtwLocMax > -1 ? col-minDistBtwLocMax : 0);
            int c1 = (col+minDistBtwLocMax < scratch.cols ? col+minDistBtwLocMax : scratch.cols-1);
            for (int r = r0; r <= r1; r++) {
                for (int c = c0; c <= c1; c++) {
                    if (sqrt((r-row)*(r-row)+(c-col)*(c-col)) <= minDistBtwLocMax) {
                      scratch.at<double>(r,c) = 0.0;
                    }
                }
            }
        } else {
            break;
        }
    }
    return locations;
}

void DetectCone::forwardDetection() {
  double threshold = 0.7;
  int patchSize = 25;
  int patchRadius = int((patchSize-1)/2);
  int inputWidth = 320;
  int heightUp = 85;
  int heightDown = 145;
  int inputHeight = heightDown-heightUp;

  int outputWidth  = inputWidth - (patchSize - 1);
  int outputHeight  = inputHeight - (patchSize - 1);

  cv::Rect roi;
  roi.x = 0;
  roi.y = heightUp;
  roi.width = inputWidth;
  roi.height = inputHeight;
  cv::Mat imgSource = cv::imread("test.png");

  cv::Mat Q, disp, rectified, XYZ;
  reconstruction(imgSource, Q, disp, rectified, XYZ);
  cv::resize(rectified, rectified, cv::Size (320, 180));

  auto patchImg = rectified(roi);
  // cv::namedWindow("roi", cv::WINDOW_NORMAL);
  // cv::imshow("roi", patchImg);
  // cv::waitKey(0);

  // convert imagefile to vec_t
  tiny_dnn::vec_t data;
  convertImage(patchImg, inputWidth, inputHeight, data);

  // recognize
  auto prob = m_nn.predict(data);

  cv::Mat probMap = cv::Mat::zeros(outputHeight, outputWidth, CV_64FC4);
  for (int c = 0; c < 4; ++c)
    for (int y = 0; y < outputHeight; ++y)
      for (int x = 0; x < outputWidth; ++x)
         probMap.at<cv::Vec4d>(y, x)[c] = prob[c * outputWidth * outputHeight + y * outputWidth + x];

  cv::Vec4d probSoftmax(4);
  cv::Mat probMapSoftmax = cv::Mat::zeros(outputHeight, outputWidth, CV_64F);
  cv::Mat probMapIndex = cv::Mat::zeros(outputHeight, outputWidth, CV_16S);
  for (int y = 0; y < outputHeight; ++y){
    for (int x = 0; x < outputWidth; ++x){
      softmax(probMap.at<cv::Vec4d>(y, x), probSoftmax);
      for (int c = 0; c < 3; ++c)
        if(probSoftmax[c+1] > threshold){
          probMapSoftmax.at<double>(y, x) = probSoftmax[c+1];
          probMapIndex.at<int>(y, x) = c+1;
        }
    }
  }

  std::vector <cv::Point> cone;
  cv::Point position, positionShift = cv::Point(patchRadius, patchRadius+heightUp);
  int label;
  cone = imRegionalMax(probMapSoftmax, 10, threshold, patchSize);
  if (cone.size()>0){
    for(size_t i=0; i<cone.size(); i++){
      position = cone[i] + positionShift;
      label = probMapIndex.at<int>(cone[i]);
      cv::Vec3f point3D = XYZ.at<cv::Vec3f>(position * 2) * 2;
      if (point3D[2] > 0 && point3D[2] < 20000){
        if (label == 1){
          cv::circle(rectified, position, 1, {0, 255, 255}, -1);
          std::cout << "Find one yellow cone, XYZ positon: "
          << point3D << "mm, xy position: " << position << "pixel, certainty: " 
          << probMapSoftmax.at<double>(cone[i]) << std::endl;
        }
        if (label == 2){
          cv::circle(rectified, position, 1, {255, 0, 0}, -1);
          std::cout << "Find one blue cone, XYZ positon: "
          << point3D << "mm, xy position: " << position << "pixel, certainty: " 
          << probMapSoftmax.at<double>(cone[i]) << std::endl;
        }
        if (label == 3){
          cv::circle(rectified, position, 1, {0, 0, 255}, -1);
          std::cout << "Find one orange cone, XYZ positon: "
          << point3D << "mm, xy position: " << position << "pixel, certainty: " 
          << probMapSoftmax.at<double>(cone[i]) << std::endl;
        }

      }
    }
  }

  // cv::namedWindow("result", cv::WINDOW_NORMAL);
  // cv::imshow("result", rectified);
  // cv::waitKey(0);

  // cv::Vec4d probSoftmax(4);
  // cv::Mat probMapSoftmax = cv::Mat::zeros(outputHeight, outputWidth, CV_64FC3);
  // for (int y = 0; y < outputHeight; ++y){
  //   for (int x = 0; x < outputWidth; ++x){
  //     softmax(probMap.at<cv::Vec4d>(y, x), probSoftmax);
  //     for (int c = 0; c < 3; ++c)
  //       if(probSoftmax[c+1] > threshold)
  //         probMapSoftmax.at<cv::Vec3d>(y, x)[c] = probSoftmax[c+1];
  //   }
  // }

  // cv::Mat probMapSplit[3];
  // cv::split(probMapSoftmax, probMapSplit);

  // for (int c = 0; c < 3; ++c){
  //   cv::namedWindow("probMap", cv::WINDOW_NORMAL);
  //   cv::imshow("probMap", probMapSplit[c]);
  //   cv::waitKey(0);
  //   // cv::destroyAllWindows();
  // }

  // std::vector <cv::Point> yellow, blue, orange;

  // int minDistBtwLocMax = patchSize;
  // yellow = imRegionalMax(probMapSplit[0], 4, threshold, minDistBtwLocMax);
  // blue = imRegionalMax(probMapSplit[1], 4, threshold, minDistBtwLocMax);
  // orange = imRegionalMax(probMapSplit[2], 2, threshold, minDistBtwLocMax);

  // cv::Point position, positionShift = cv::Point(patchRadius, patchRadius+heightUp);

  // if (yellow.size()>0){
  //   for(size_t i=0; i<yellow.size(); i++){
  //     position = yellow[i] + positionShift;
  //     cv::Vec3f point3D = XYZ.at<cv::Vec3f>(position * 2) * 2;
  //     if (point3D[2] > 0 && point3D[2] < 20000){
  //       cv::circle(rectified, position, 1, {0, 255, 255}, -1);
  //       std::cout << "Find one yellow cone, XYZ positon: "
  //       << point3D << "mm, xy position: " << position << "pixel, certainty: " 
  //       << probMapSoftmax.at<cv::Vec3d>(yellow[i])[0] << std::endl;
  //     }
  //   }
  // }
  // if (blue.size()>0){
  //   for(size_t i=0; i<blue.size(); i++){
  //     position = blue[i] + positionShift;
  //     cv::Vec3f point3D = XYZ.at<cv::Vec3f>(position * 2) * 2;
  //     if (point3D[2] > 0 && point3D[2] < 20000){
  //       cv::circle(rectified, position, 1, {255, 0, 0}, -1);
  //       std::cout << "Find one blue cone, XYZ positon: "
  //       << point3D << "mm, xy position: " << position << "pixel, certainty: " 
  //       << probMapSoftmax.at<cv::Vec3d>(blue[i])[1] << std::endl;
  //     }
  //   }
  // }
  // if (orange.size()>0){
  //   for(size_t i=0; i<orange.size(); i++){
  //     position = orange[i] + positionShift;
  //     cv::Vec3f point3D = XYZ.at<cv::Vec3f>(position * 2) * 2;
  //     if (point3D[2] > 0 && point3D[2] < 20000){
  //       cv::circle(rectified, position, 1, {0, 0, 255}, -1);
  //       std::cout << "Find one orange cone, XYZ positon: "
  //       << point3D << "mm, xy position: " << position << "pixel, certainty: " 
  //       << probMapSoftmax.at<cv::Vec3d>(orange[i])[2] << std::endl;
  //     }
  //   }
  // }

  
  // cv::namedWindow("result", cv::WINDOW_NORMAL);
  // // cv::setWindowProperty("result", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  // cv::imshow("result", rectified);
  // cv::waitKey(0);

  // // int index = imgPath.find_last_of('/');
  // // std::string savePath(imgPath.substr(index+1));
  // // cv::imwrite("result/"+savePath, rectified);
}

Eigen::MatrixXd DetectCone::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = MatrixXd::Zero(4,1);
  recievedPoint << xData * 1000,
                   yData * 1000,
                   zData * 1000,
                    0;
  return recievedPoint;
}

void DetectCone::Cartesian2Spherical(double x, double y, double z, opendlv::logic::sensation::Point &pointInSpherical)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan(x/y)*static_cast<double>(RAD2DEG);
  double zenithAngle = atan(z/sqrt(x*x+y*y))*static_cast<double>(RAD2DEG);
  pointInSpherical.setDistance(distance);
  pointInSpherical.setAzimuthAngle(azimuthAngle);
  pointInSpherical.setZenithAngle(zenithAngle);
}

void DetectCone::SendCollectedCones(Eigen::MatrixXd lidarCones)
{
  //Convert to cartesian
  Eigen::MatrixXd cone;
  for(int p = 0; p < lidarCones.cols(); p++){
    cone = Spherical2Cartesian(lidarCones(0,p), lidarCones(1,p), lidarCones(2,p));
    lidarCones.col(p) = cone;
  }
  std::cout << "lidarCones " << std::endl;
  std::cout << lidarCones << std::endl;
  m_finalPointCloud = lidarCones;
  cv::Vec3f point3D;
  double yShift = 0;//1872mm
  double zShift = 0;
  for (int i = 0; i < m_finalPointCloud.cols(); i++){
    point3D << m_finalPointCloud(0,i), -zShift-m_finalPointCloud(2,i), yShift+m_finalPointCloud(1,i);
    m_finalPointCloud(3,i) = backwardDetection(m_img, point3D);
  }

  //std::cout << "matched: " << std::endl;
  //std::cout << m_finalPointCloud << std::endl;
  SendMatchedContainer(m_finalPointCloud);
}

void DetectCone::SendMatchedContainer(Eigen::MatrixXd cones)
{
  for(int n = 0; n < cones.cols(); n++){

    opendlv::logic::sensation::Point conePoint;
    Cartesian2Spherical(cones(0,n), cones(1,n), cones(2,n), conePoint);

    opendlv::logic::perception::ObjectDirection coneDirection;
    coneDirection.setObjectId(n);
    coneDirection.setAzimuthAngle(conePoint.getAzimuthAngle());
    coneDirection.setZenithAngle(conePoint.getZenithAngle());
    odcore::data::Container c2(coneDirection);
    c2.setSenderStamp(m_senderStamp);
    getConference().send(c2);

    opendlv::logic::perception::ObjectDistance coneDistance;
    coneDistance.setObjectId(n);
    coneDistance.setDistance(conePoint.getDistance());
    odcore::data::Container c3(coneDistance);
    c3.setSenderStamp(m_senderStamp);
    getConference().send(c3);

    opendlv::logic::perception::ObjectType coneType;
    coneType.setObjectId(n);
    coneType.setType(cones(3,n));
    odcore::data::Container c4(coneType);
    c3.setSenderStamp(m_senderStamp);
    getConference().send(c4);
/*
    opendlv::logic::sensation::Point conePoint;
    Cartesian2Spherical(cones(0,n), cones(1,n), cones(2,n), conePoint);
    odcore::data::Container c1(conePoint);
    getConference().send(c1);
    std::cout << "a point sent out with distance: " <<conePoint.getDistance() <<"; azimuthAngle: " << conePoint.getAzimuthAngle() << "; and zenithAngle: " << conePoint.getZenithAngle() << std::endl;
*/
  }
}

}
}
}
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

// void DetectCone::testSlidingWindow(const std::string &imgPath){
//   cv::Mat img = cv::imread(imgPath);
//   // cv::Vec3f point3D;
//   // point3D << 1182.67, 166.36, 1750.78;
//   // backwardDetection(img, point3D);
//   forwardDetection(img);
// }


// int DetectCone::forwardDetection(cv::Mat img){
//   //Detect cone in camera frame and then project to 3D world
//
//   // manual roi
//   // (453, 237,  0.96875,  "orange", 319.105, 172.883, 1033.59);
//   // (585, 211,  0.625,  "orange", 1182.67, 166.36, 1750.78);
//   // (343, 185,  0.25, "yellow", 10.183, 75.1519, 3299.55);
//   // (521, 198,  0.375,  "yellow", 1219.86, 144.365, 2451.1);
//   // (625, 191,  0.34375,  "blue", 2499.47, 125.34, 3177.35);
//   // (396, 183,  0.34375,  "blue", 586.295, 67.1457, 3899.47);
//
//   // convert imagefile to vec_t
//   std::cout << "image size: " << img.size() << std::endl;
//   cv::Mat Q, disp, rectified, XYZ;
//   reconstruction(img, Q, disp, rectified, XYZ);
//
//   int x = 250;
//   int y = 170;
//   float_t ratio = depth2resizeRate(319.105, 1033.59);
//   int length = ratio * 32;
//   int radius = (length-1)/2;
//
//   cv::Vec2f point2D;
//   point2D << x, y;
//   cv::Vec3f point3D = XYZ.at<cv::Vec3f>(y,x) * 2;
//   std::cout << "Find one cone, XYZ positon: "
//     << point3D << "mm, xy position: " << point2D << "pixel" << std::endl;
//
//   cv::Rect roi;
//   roi.x = std::max(x - radius, 0);
//   roi.y = std::max(y - radius, 0);
//   roi.width = std::min(x + radius, img.cols) - roi.x;
//   roi.height = std::min(y + radius, img.rows) - roi.y;
//   auto patchImg = rectified(roi);
//
//   tiny_dnn::vec_t data;
//   convertImage(patchImg, 32, 32, data);
//   auto prob = m_nn.predict(data);
//   float_t threshold = 0.5;
//   // std::cout << prob[0] << " " << prob[1] << " " << prob[2] << " " << prob[3] << std::endl;
//   int maxIndex = 1;
//   float_t maxProb = prob[1];
//   for(int i=2;i<4;i++){
//     if(prob[i]>prob[maxIndex]){
//       maxIndex = i;
//       maxProb = prob[i];
//     }
//   }
//
//   std::string labels[] = {"yellow", "blue", "orange"};
//   if (maxProb < threshold)
//     std::cout << "No cone detected" << std::endl;
//   else
//     std::cout << "Find one " << labels[maxIndex-1] << " cone, XYZ positon: "
//     << point3D << "mm, xy position: " << point2D << "pixel, certainty: " << maxProb << std::endl;
//
//   // cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,0,0));
//   // // cv::circle(disp, cv::Point (x,y), 3, 0, CV_FILLED);
//   // cv::namedWindow("disp", cv::WINDOW_NORMAL);
//   // cv::imshow("disp", rectified);
//   // cv::waitKey(0);
//   // cv::destroyAllWindows();
//    //cv::imwrite("test.png",rectified);
//
//   return maxIndex;
// }

//run cnn ends

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

