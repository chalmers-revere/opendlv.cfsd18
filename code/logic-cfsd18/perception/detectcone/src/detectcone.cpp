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
, m_newFrame(true)
, m_coneMutex()
, m_recievedFirstImg(false)
, m_img()
, m_slidingWindow()
, m_efficientSlidingWindow()
, m_lidarIsWorking(false)
, m_checkLiarMilliseconds()
, m_count(0)
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

  efficientSlidingWindow("efficientSlidingWindow", 336, 60);
  slidingWindow("slidingWindow");
}

void DetectCone::tearDown()
{
}

void DetectCone::nextContainer(odcore::data::Container &a_container)
{
  //Just for my testing
  // odcore::data::TimeStamp startTime;
  // cv::Mat img = cv::imread("test.png");
  // forwardDetectionRoI(img, m_slidingWindow);

  // forwardDetection(img);
  // std::vector<cv::Point3f> pts;
  // pts.push_back(cv::Point3f(-0.319045, 0.164448, 0.787049)*2);
  // pts.push_back(cv::Point3f(-0.656261, 0.133473, 1.90641)*2);
  // pts.push_back(cv::Point3f(0.435741, 0.154018, 0.824888)*2);
  // pts.push_back(cv::Point3f(0.477722, 0.0988543, 3.17735)*2);
  // pts.push_back(cv::Point3f(-0.369824, 0.0811971, 3.17735)*2);
  // pts.push_back(cv::Point3f(0.715716, 0.158176, 1.94974)*2);
  
  // std::vector<int> outputs;
  // backwardDetection(img, pts, outputs);
  
  // odcore::data::TimeStamp endTime;
  // double timeElapsed = abs(static_cast<double>(endTime.toMicroseconds()-startTime.toMicroseconds())/1000.0);
  // std::cout << "Time elapsed for camera detection: " << timeElapsed << std::endl;


  if (a_container.getDataType() == odcore::data::image::SharedImage::ID()) {
    odcore::data::image::SharedImage sharedImg =
    a_container.getData<odcore::data::image::SharedImage>();
    odcore::data::TimeStamp timeStamp = a_container.getReceivedTimeStamp();
    std::cout << timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds() << std::endl;
    m_lastTimeStamp = timeStamp;
    if (!ExtractSharedImage(&sharedImg)) {
      std::cout << "[" << getName() << "] " << "[Unable to extract shared image." << std::endl;
      return;
    }else if(!m_recievedFirstImg){
      m_recievedFirstImg = true;
    }

    forwardDetectionRoI(m_img, m_slidingWindow);

    // if (((timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds()) > m_checkLiarMilliseconds*1000)){
    //   std::cout << "Lidar fails! Camera detection only! " << std::endl;
    //   m_lidarIsWorking = false;
    // }
    // else{
    //   m_lidarIsWorking = true;
    //   std::cout << "Lidar is working!" << std::endl;
    // }
    // if(!m_lidarIsWorking){
    //   forwardDetection(m_img);
    // }
  }

  bool correctSenderStamp = a_container.getSenderStamp() == m_attentionSenderStamp;
  if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID() && correctSenderStamp) {
    std::cout << "Recieved Direction" << std::endl;
    m_lastTimeStamp = a_container.getReceivedTimeStamp();
    auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
    uint32_t objectId = coneDirection.getObjectId();
    bool newFrameDist = false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      m_coneCollector(0,objectId) = -coneDirection.getAzimuthAngle();  //Negative for conversion from car to LIDAR frame
      m_coneCollector(1,objectId) = coneDirection.getZenithAngle();
      m_lastObjectId = (m_lastObjectId<objectId)?(objectId):(m_lastObjectId);
      newFrameDist = m_newFrame;
      m_newFrame = false;
    }
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (newFrameDist){
       std::thread coneCollector(&DetectCone::initializeCollection, this);
       coneCollector.detach();
       //initializeCollection();
    }
  }

  else if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID() && correctSenderStamp){
    std::cout << "Recieved Distance" << std::endl;
    m_lastTimeStamp = a_container.getReceivedTimeStamp();
    auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
    uint32_t objectId = coneDistance.getObjectId();
    bool newFrameDist = false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      m_coneCollector(2,objectId) = coneDistance.getDistance();
      m_coneCollector(3,objectId) = 0;
      m_lastObjectId = (m_lastObjectId<objectId)?(objectId):(m_lastObjectId);
      newFrameDist = m_newFrame;
      m_newFrame = false;
    }
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (newFrameDist){
       std::thread coneCollector(&DetectCone::initializeCollection, this);
       coneCollector.detach();
       //initializeCollection();
    }

  }
}
/*OLD CONE COLLECTOR
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
}*/

void DetectCone::initializeCollection(){
  //std::this_thread::sleep_for(std::chrono::duration 1s); //std::chrono::milliseconds(m_timeDiffMilliseconds)

  bool sleep = true;
  auto start = std::chrono::system_clock::now();

  while(sleep)
  {
    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
    if ( elapsed.count() > m_timeDiffMilliseconds*1000 )
        sleep = false;
  }


  Eigen::MatrixXd extractedCones;
  {
    odcore::base::Lock lockCone(m_coneMutex);
	  std::cout << "FRAME IN LOCK: " << m_newFrame << std::endl;
    extractedCones = m_coneCollector.leftCols(m_lastObjectId+1);
    m_newFrame = true;
    m_lastObjectId = 0;
    m_coneCollector = Eigen::MatrixXd::Zero(4,20);
  }
  //Initialize for next collection
  std::cout << "Collection done " << extractedCones.cols() << std::endl;
  if(extractedCones.cols() > 0){
    //std::cout << "Extracted Cones " << std::endl;
    //std::cout << extractedCones << std::endl;
    std::cout << "Extracted Cones " << std::endl;
    std::cout << extractedCones << std::endl;
    if(m_recievedFirstImg){
      SendCollectedCones(extractedCones);
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

  // cv::imwrite("images/"+std::to_string(m_count++)+".png", m_img);
  // cv::namedWindow("img", cv::WINDOW_NORMAL);
  // cv::imshow("img", m_img);
  // cv::waitKey(2);
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

  sbm(grayL, grayR, disp, CV_32F);
  // cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
}

void DetectCone::reconstruction(cv::Mat img, cv::Mat &Q, cv::Mat &disp, cv::Mat &rectified, cv::Mat &XYZ){
  cv::Mat mtxLeft = (cv::Mat_<double>(3, 3) <<
    349.891, 0, 334.352,
    0, 349.891, 187.937,
    0, 0, 1);
  cv::Mat distLeft = (cv::Mat_<double>(5, 1) << -0.173042, 0.0258831, 0, 0, 0);
  cv::Mat mtxRight = (cv::Mat_<double>(3, 3) <<
    350.112, 0, 345.88,
    0, 350.112, 189.891,
    0, 0, 1);
  cv::Mat distRight = (cv::Mat_<double>(5, 1) << -0.174209, 0.026726, 0, 0, 0);
  cv::Mat rodrigues = (cv::Mat_<double>(3, 1) << -0.0132397, 0.021005, -0.00121284);
  cv::Mat R;
  cv::Rodrigues(rodrigues, R);
  cv::Mat T = (cv::Mat_<double>(3, 1) << -0.12, 0, 0);
  cv::Size stdSize = cv::Size(672, 376);

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
}

void DetectCone::xyz2xy(cv::Mat Q, cv::Point3f xyz, cv::Point2f &xy){
  double X = xyz.x;
  double Y = xyz.y;
  double Z = xyz.z;
  double Cx = -Q.at<double>(0,3);
  double Cy = -Q.at<double>(1,3);
  double f = Q.at<double>(2,3);
  double a = Q.at<double>(3,2);
  double b = Q.at<double>(3,3);
  double d = (f - Z * b ) / ( Z * a);
  xy.x = X * ( d * a + b ) + Cx;
  xy.y = Y * ( d * a + b ) + Cy;
}

float_t DetectCone::depth2resizeRate(double x, double y){
  return 2*(1.6078-0.4785*std::sqrt(std::sqrt(x*x+y*y)));
}

void DetectCone::convertImage(cv::Mat img, int w, int h, tiny_dnn::vec_t &data){
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(w, h));
  data.resize(w * h * 3);
  for (int c = 0; c < 3; ++c) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
       data[c * w * h + y * w + x] =
         float(resized.at<cv::Vec3b>(y, x)[c] / 255.0);
      }
    }
  }
}


void DetectCone::slidingWindow(const std::string &dictionary) {
  using conv    = tiny_dnn::convolutional_layer;
  using pool    = tiny_dnn::max_pooling_layer;
  using fc      = tiny_dnn::fully_connected_layer;
  using tanh    = tiny_dnn::tanh_layer;
  using leaky_relu    = tiny_dnn::leaky_relu_layer;
  using softmax = tiny_dnn::softmax_layer;

  tiny_dnn::core::backend_t backend_type = tiny_dnn::core::default_engine();

  m_slidingWindow << conv(25, 25, 4, 3, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh() 
     // << dropout(22*22*16, 0.25)                    
     << pool(22, 22, 16, 2, backend_type)                               
     << conv(11, 11, 4, 16, 32, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh() 
     // << dropout(8*8*32, 0.25)                    
     << pool(8, 8, 32, 2, backend_type) 
     << fc(4 * 4 * 32, 128, true, backend_type) << leaky_relu()  
     << fc(128, 5, true, backend_type) << softmax(5);

  // load nets
  std::ifstream ifs(dictionary.c_str());
  ifs >> m_slidingWindow;
}


void DetectCone::backwardDetection(cv::Mat img, std::vector<cv::Point3f> pts, std::vector<int>& outputs){
  //Given RoI in 3D world, project back to the camera frame and then detect
  float_t threshold = 0.7;
  cv::Mat disp, Q, rectified, XYZ;
  reconstruction(img, Q, disp, rectified, XYZ);
  std::vector<tiny_dnn::tensor_t> inputs;
  std::vector<int> verifiedIndex;
  std::vector<cv::Vec3i> porperty;
  outputs.clear();

  for(size_t i = 0; i < pts.size(); i++){
    cv::Point2f point2D;
    xyz2xy(Q, pts[i], point2D);

    int x = point2D.x;
    int y = point2D.y;

    // std::cout << "Camera region center: " << x << ", " << y << std::endl;
    float_t ratio = depth2resizeRate(pts[i].x, pts[i].z);
    if (ratio > 0) {
      int length = ratio * 25;
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
        outputs.push_back(-1);
      }
      else{
        auto patchImg = rectified(roi);
        tiny_dnn::vec_t data;
        convertImage(patchImg, 25, 25, data);
        inputs.push_back({data});
        outputs.push_back(0);
        verifiedIndex.push_back(i);
        porperty.push_back(cv::Vec3i(x,y,radius));
      }
    }
  }
  
  if(inputs.size()>0){
    auto prob = m_slidingWindow.predict(inputs);
    for(size_t i = 0; i < inputs.size(); i++){
      size_t maxIndex = 0;
      float_t maxProb = prob[i][0][0];
      for(size_t j = 1; j < 5; j++){
        if(prob[i][0][j] > maxProb){
          maxIndex = j;
          maxProb = prob[i][0][j];
        }
      }
      outputs[verifiedIndex[i]] = maxIndex;
      int x = int(porperty[i][0]);
      int y = int(porperty[i][1]);
      float_t radius = porperty[i][2];

      std::string labels[] = {"blue", "yellow", "orange", "big orange"};
      if (maxIndex == 0 || maxProb < threshold){
        std::cout << "No cone detected" << std::endl;
        cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,0,0));
      } 
      else{
        std::cout << "Find one " << labels[maxIndex-1] << " cone"<< std::endl;
        if (labels[maxIndex-1] == "blue")
          cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (255,0,0));
        else if (labels[maxIndex-1] == "yellow")
          cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,255,255));
        else if (labels[maxIndex-1] == "orange")
          cv::circle(rectified, cv::Point (x,y), radius, cv::Scalar (0,165,255));
        else if (labels[maxIndex-1] == "big orange")
          cv::circle(rectified, cv::Point (x,y), radius*2, cv::Scalar (0,0,255));
      }
    }
  }

  // cv::namedWindow("disp", cv::WINDOW_NORMAL);
  // // cv::setWindowProperty("result", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  // cv::imshow("disp", rectified);
  // cv::waitKey(0);
  // cv::destroyAllWindows();

  // for(size_t i = 0; i < pts.size(); i++)
  //   std::cout << i << ": " << outputs[i] << std::endl;
}


void DetectCone::efficientSlidingWindow(const std::string &dictionary, int width, int height) {
  using conv    = tiny_dnn::convolutional_layer;
  using tanh    = tiny_dnn::tanh_layer;
  tiny_dnn::core::backend_t backend_type = tiny_dnn::core::backend_t::internal;

  m_efficientSlidingWindow << conv(width, height, 3, 3, 8, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     << conv(width-2, height-2, 3, 8, 8, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     // << dropout((width-4)*(height-4)*8, 0.25)
     << conv(width-4, height-4, 3, 8, 8, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     << conv(width-6, height-6, 3, 8, 8, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     // << dropout((width-8)*(height-8)*8, 0.25)
     << conv(width-8, height-8, 3, 8, 8, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     << conv(width-10, height-10, 3, 8, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     // << dropout((width-12)*(height-12)*16, 0.25)
     << conv(width-12, height-12, 3, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     << conv(width-14, height-14, 3, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     // << dropout((width-16)*(height-16)*16, 0.25)
     << conv(width-16, height-16, 3, 16, 32, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     << conv(width-18, height-18, 3, 32, 32, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     // << dropout((width-20)*(height-20)*32, 0.25)
     << conv(width-20, height-20, 3, 32, 64, tiny_dnn::padding::valid, true, 1, 1, backend_type) << tanh()
     << conv(width-22, height-22, 3, 64, 5, tiny_dnn::padding::valid, true, 1, 1, backend_type);

  // load nets
  std::ifstream ifs(dictionary.c_str());
  ifs >> m_efficientSlidingWindow;
}

void DetectCone::softmax(cv::Vec<double,5> x, cv::Vec<double,5> &y) {
  double min, max, denominator = 0;
  cv::minMaxLoc(x, &min, &max);
  for (int j = 0; j < 5; j++) {
    y[j] = std::exp(x[j] - max);
    denominator += y[j];
  }
  for (int j = 0; j < 5; j++) {
    y[j] /= denominator;
  }
}

// float DetectCone::medianVector(std::vector<float> input){    
//   std::nth_element(input.begin(), input.begin() + input.size() / 2, input.end());
//   return input[input.size() / 2];
// }
int DetectCone::medianVector(std::vector<std::pair<float, int>> input){
  int n = input.size()/2;
  std::nth_element(input.begin(), input.begin()+n, input.end());

  // int median = input[n].first;
  return input[n].second;
}

std::vector <cv::Point> DetectCone::imRegionalMax(cv::Mat input, int nLocMax, double threshold, int minDistBtwLocMax)
{
    cv::Mat scratch = input.clone();
    // std::cout<<scratch<<std::endl;
    // cv::GaussianBlur(scratch, scratch, cv::Size(3,3), 0, 0);
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

void DetectCone::forwardDetection(cv::Mat imgSource) {
  double threshold = 0.9;
  int patchSize = 25;
  int patchRadius = int((patchSize-1)/2);
  int col = 336;
  int row = 188;
  int heightUp = 70;
  int heightDown = 130;
  int inputWidth = col;
  int inputHeight = heightDown-heightUp;

  int outputWidth  = inputWidth - (patchSize - 1);
  int outputHeight  = inputHeight - (patchSize - 1);

  cv::Rect roi;
  roi.x = 0;
  roi.y = heightUp;
  roi.width = inputWidth;
  roi.height = inputHeight;

  cv::Mat Q, disp, rectified, XYZ, resize_img;
  reconstruction(imgSource, Q, disp, rectified, XYZ);
  cv::resize(rectified, resize_img, cv::Size (col, row));

  auto patchImg = resize_img(roi);

  // convert imagefile to vec_t
  tiny_dnn::vec_t data;
  convertImage(patchImg, inputWidth, inputHeight, data);

  // recognize
  auto prob = m_efficientSlidingWindow.predict(data);

  cv::Mat probMap = cv::Mat::zeros(outputHeight, outputWidth, CV_64FC(5));
  for (int c = 0; c < 5; ++c)
    for (int y = 0; y < outputHeight; ++y)
      for (int x = 0; x < outputWidth; ++x)
         probMap.at<cv::Vec<double,5>>(y, x)[c] = prob[c * outputWidth * outputHeight + y * outputWidth + x];

  cv::Vec<double,5> probSoftmax(5);
  cv::Mat probMapSoftmax = cv::Mat::zeros(outputHeight, outputWidth, CV_64F);
  cv::Mat probMapIndex = cv::Mat::zeros(outputHeight, outputWidth, CV_32S);
  for (int y = 0; y < outputHeight; ++y){
    for (int x = 0; x < outputWidth; ++x){
      softmax(probMap.at<cv::Vec<double,5>>(y, x), probSoftmax);
      for (int c = 0; c < 4; ++c)
        if(probSoftmax[c+1] > threshold){
          probMapSoftmax.at<double>(y, x) = probSoftmax[c+1];
          probMapIndex.at<int>(y, x) = c+1;
        }
    }
  }

  std::vector <cv::Point> cone;
  cv::Point position, positionShift = cv::Point(patchRadius, patchRadius+heightUp);
  int label;
  cone = imRegionalMax(probMapSoftmax, 10, threshold, 20);
  // std::vector<std::pair<float, int>> coneRegion;
  // std::vector<cv::Point> coneRegionXY;

  if (cone.size()>0){
    for(size_t i = 0; i<cone.size(); i++){
      position = (cone[i] + positionShift)*2;
      if(position.x>320 && position.x<400 && position.y>250) continue;

      label = probMapIndex.at<int>(cone[i]);
      
      //median rectify
      // int ith = 0;
      // for(int x = positionResize.x-5; x <= positionResize.x+5; x++){
      //   for(int y = positionResize.y-5; y <= positionResize.y+5; y++){
      //     if (x >= 0 && x < 2*col && y >= 0 && y < 2*row){
      //       float depth = XYZ.at<cv::Point3f>(x, y)[2] * 2;
      //       if(depth > 0 && depth < 20000){
      //         coneRegion.push_back(std::pair<float, int>(depth, ith++));
      //         coneRegionXY.push_back(cv::Point(x, y));
      //       }
      //     }
      //   }  
      // }
      // if(coneRegion.size()>0){
      //   int medianIndex = medianVector(coneRegion);

      cv::Point3f point3D = XYZ.at<cv::Point3f>(position);
      // cv::Point3f point3D2 = XYZ.at<cv::Point3f>(coneRegionXY[medianIndex]) * 2;
      // std::cout << point3D2 << " " << point3D << std::endl;
      if (point3D.z > 0 && point3D.z < 10){
        if (label == 1){
          cv::circle(rectified, position, 3, {255, 0, 0}, -1);
          std::cout << "Find one blue cone, XYZ positon: "
          << point3D << "mm, xy position: " << position << "pixel, certainty: " 
          << probMapSoftmax.at<double>(cone[i]) << std::endl;
        }
        if (label == 2){
          cv::circle(rectified, position, 3, {0, 255, 255}, -1);
          std::cout << "Find one yellow cone, XYZ positon: "
          << point3D << "mm, xy position: " << position << "pixel, certainty: " 
          << probMapSoftmax.at<double>(cone[i]) << std::endl;
        }
        if (label == 3){
          cv::circle(rectified, position, 3, {0, 165, 255}, -1);
          std::cout << "Find one orange cone, XYZ positon: "
          << point3D << "mm, xy position: " << position << "pixel, certainty: " 
          << probMapSoftmax.at<double>(cone[i]) << std::endl;
        }
        if (label == 4){
          cv::circle(rectified, position, 6, {0, 0, 255}, -1);
          std::cout << "Find one big orange cone, XYZ positon: "
          << point3D << "mm, xy position: " << position << "pixel, certainty: " 
          << probMapSoftmax.at<double>(cone[i]) << std::endl;
        }
      }
    }
  }
}

void DetectCone::forwardDetectionRoI(cv::Mat imgSource, tiny_dnn::network<tiny_dnn::sequential> nn){
  //Given RoI by SIFT detector and detected by CNN
  float_t threshold = 0.5f;
  int radius = 12;
  
  std::vector<tiny_dnn::tensor_t> inputs;
  std::vector<int> verifiedIndex;
  std::vector<cv::Point> candidates;
  // std::vector<int> outputs;

  cv::Mat Q, disp, XYZ, img;
  cv::resize(imgSource, imgSource, cv::Size(1344,376));
  reconstruction(imgSource, Q, disp, imgSource, XYZ);
  // img = img.rowRange(176, 376);
  img = imgSource.rowRange(180, 320);
  // img.rowRange(0,24) = 0;
  // img.rowRange(126,150) = 0;
  // cv::Mat img_hsv;
  // cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

  cv::ORB detector(20);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(img, keypoints);

  // cv::Mat Match;
  // cv::drawKeypoints(img, keypoints, Match);
  // cv::namedWindow("cv::Match", cv::WINDOW_NORMAL);
  // cv::imshow("cv::Match", Match);
  // cv::waitKey(0);

  cv::resize(img, img, cv::Size(336, 70));
  cv::Mat probMap = cv::Mat::zeros(70, 336, CV_64F);
  cv::Mat indexMap = cv::Mat::zeros(70, 336, CV_32S);

  for(size_t i = 0; i < keypoints.size(); i++){
    int x = int(keypoints[i].pt.x/2);
    int y = int(keypoints[i].pt.y/2);

    cv::Rect roi;
    roi.x = std::max(x - radius, 0);
    roi.y = std::max(y - radius, 0);
    roi.width = std::min(x + radius, img.cols) - roi.x;
    roi.height = std::min(y + radius, img.rows) - roi.y;

    //cv::circle(img, cv::Point (x,y), radius, cv::Scalar (0,0,0));
    // // cv::circle(disp, cv::Point (x,y), 3, 0, CV_FILLED);
    //cv::namedWindow("roi", cv::WINDOW_NORMAL);
    //cv::imshow("roi", img);
    //cv::waitKey(0);
    //cv::destroyAllWindows();
    if (0 > roi.x || 0 > roi.width || roi.x + roi.width > img.cols || 0 > roi.y || 0 > roi.height || roi.y + roi.height > img.rows){
      std::cout << "Wrong roi!" << std::endl;
      // outputs.push_back(-1);
    }
    else{
      auto patchImg = img(roi);
      tiny_dnn::vec_t data;
      convertImage(patchImg, 25, 25, data);
      inputs.push_back({data});
      // outputs.push_back(0);
      verifiedIndex.push_back(i);
      candidates.push_back(cv::Point(x,y));
    }
  }
  
  std::ofstream savefile;
  savefile.open("results/"+std::to_string(m_count)+".csv");

  int resultWidth = 672;
  int resultHeight = 600;
  double resultResize = 30;
  cv::Mat result[2] = cv::Mat::zeros(resultHeight, resultWidth, CV_8UC3), coResult;

  if(inputs.size()>0){
    auto prob = nn.predict(inputs);
    for(size_t i = 0; i < inputs.size(); i++){
      size_t maxIndex = 1;
      float_t maxProb = prob[i][0][1];
      for(size_t j = 2; j < 5; j++){
        if(prob[i][0][j] > maxProb){
          maxIndex = j;
          maxProb = prob[i][0][j];
        }
      }
      // outputs[verifiedIndex[i]] = maxIndex;
      int x = candidates[i].x;
      int y = candidates[i].y;
      probMap.at<double>(y,x) = maxProb;
      indexMap.at<int>(y,x) = maxIndex;
    }
    std::vector <cv::Point> cones = imRegionalMax(probMap, 15, threshold, 15);

    std::string labels[] = {"background", "blue", "yellow", "orange", "big orange"};
    for(size_t i = 0; i < cones.size(); i++){
      int x = cones[i].x;
      int y = cones[i].y;
      int maxIndex = indexMap.at<int>(y,x);
      cv::Point position(x*2, y*2+180);
      cv::Point3f point3D = XYZ.at<cv::Point3f>(position);
      std::string labelName = labels[maxIndex];

      if (labelName == "background"){
        std::cout << "No cone detected" << std::endl;
        cv::circle(imgSource, position, 3, cv::Scalar (0,0,0), -1);
      } 
      else{
        std::cout << "Find one " << labelName << " cone"<< std::endl;
        if (labelName == "blue")
          cv::circle(imgSource, position, 3, cv::Scalar (175,238,238), -1);
        else if (labelName == "yellow")
          cv::circle(imgSource, position, 3, cv::Scalar (0,255,255), -1);
        else if (labelName == "orange")
          cv::circle(imgSource, position, 3, cv::Scalar (0,165,255), -1);
        else if (labelName == "big orange")
          cv::circle(imgSource, position, 6, cv::Scalar (0,0,255), -1);

        int xt = int(point3D.x * float(resultResize) + resultWidth/2);
        int yt = int((point3D.z-1.872f) * float(resultResize));
        if (xt >= 0 && xt <= resultWidth && yt >= 0 && yt <= resultHeight){
          if (labelName == "blue")
            cv::circle(result[0], cv::Point (xt,yt), 5, cv::Scalar (255,0,0), -1);
          else if (labelName == "yellow")
            cv::circle(result[0], cv::Point (xt,yt), 5, cv::Scalar (0,255,255), -1);
          else if (labelName == "orange")
            cv::circle(result[0], cv::Point (xt,yt), 5, cv::Scalar (0,165,255), -1);
          else if (labelName == "big orange")
            cv::circle(result[0], cv::Point (xt,yt), 10, cv::Scalar (0,0,255), -1);
        }

        std::cout << position << " " << labelName << " " << point3D << std::endl;
        savefile << std::to_string(position.x)+","+std::to_string(position.y)+","
          +labelName+","+std::to_string(point3D.x)+","+std::to_string(point3D.y)+","+std::to_string(point3D.z)+"\n";
      }
    }
  }

  for(int i = 0; i < m_finalPointCloud.cols(); i++){
    savefile << std::to_string(m_finalPointCloud(0,i))+","+std::to_string(m_finalPointCloud(1,i))+","+std::to_string(m_finalPointCloud(2,i))+"\n";
    int x = int(m_finalPointCloud(0,i) * resultResize + resultWidth/2);
    int y = int(m_finalPointCloud(1,i) * resultResize);
    if (x >= 0 && x <= resultWidth && y >= 0 && y <= resultHeight){
      cv::circle(result[0], cv::Point (x,y), 5, cv::Scalar (255, 255, 255), -1);
    }
  }

  cv::circle(result[0], cv::Point (int(resultWidth/2),0), 5, cv::Scalar (0, 0, 255), -1);
  cv::flip(result[0], result[0], 0);
  imgSource.copyTo(result[1].rowRange(resultHeight-376,resultHeight));
  cv::hconcat(result[1], result[0], coResult);
  cv::imwrite("results/"+std::to_string(m_count++)+".png", coResult);

  // cv::namedWindow("img", cv::WINDOW_NORMAL);
  // cv::imshow("img", img);
  // cv::waitKey(0);
  // cv::destroyAllWindows();

  // for(size_t i = 0; i < pts.size(); i++)
  //   std::cout << i << ": " << outputs[i] << std::endl;
}

Eigen::MatrixXd DetectCone::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  //double xyDistance = distance * cos(azimuth * static_cast<double>(DEG2RAD));
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double zData = distance * sin(zenimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = MatrixXd::Zero(4,1);
  recievedPoint << xData,
                   yData,
                   zData,
                    0;
  return recievedPoint;
}

void DetectCone::Cartesian2Spherical(double x, double y, double z, opendlv::logic::sensation::Point &pointInSpherical)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan2(x,y)*static_cast<double>(RAD2DEG);
  double zenithAngle = atan2(z,sqrt(x*x+y*y))*static_cast<double>(RAD2DEG);
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
  double yShift = 0;//1872mm
  double zShift = 0;
  std::vector<cv::Point3f> pts;
  std::vector<int> outputs;

  // int width = 640;
  // int height = 1000;
  // double resultResize = 50;
  // cv::Mat result[2] = cv::Mat::zeros(height, width, CV_8UC3), coResult;

  for (int i = 0; i < m_finalPointCloud.cols(); i++){
    pts.push_back(cv::Point3d(m_finalPointCloud(0,i), -zShift-m_finalPointCloud(2,i), yShift+m_finalPointCloud(1,i)));
  //   int x = int(m_finalPointCloud(0,i) * resultResize + width/2);
  //   int y = int(m_finalPointCloud(1,i) * resultResize);
  //   if (x >= 0 && x <= width && y >= 0 && y <= height){
  //     cv::circle(result[0], cv::Point (x,y), 5, cv::Scalar (255, 255, 255), -1);
  //   }
  }

  // cv::circle(result[0], cv::Point (int(width/2),0), 5, cv::Scalar (0, 0, 255), -1);
  // cv::flip(result[0], result[0], 0);
  // cv::flip(result[0], result[0], 1);
  // cv::Mat rectified = m_img.colRange(0,1280);
  // cv::resize(rectified, rectified, cv::Size(640, 360));
  // // rectified.convertTo(rectified, CV_8UC3);
  // rectified.copyTo(result[1].rowRange(320,680));
  // // result[1].rowRange(320,680) = rectified;
  // cv::hconcat(result[0], result[1], coResult);

  // cv::imwrite("results/"+std::to_string(m_count++)+".png", coResult);


  // forwardDetectionRoI(m_img, m_slidingWindow);
  // backwardDetection(m_img, pts, outputs);
  // for (int i = 0; i < m_finalPointCloud.cols(); i++){
  //   m_finalPointCloud(3,i) = outputs[i];
  // }

  //std::cout << "matched: " << std::endl;
  //std::cout << m_finalPointCloud << std::endl;
  SendMatchedContainer(m_finalPointCloud);
}

void DetectCone::SendMatchedContainer(Eigen::MatrixXd cones)
{
  opendlv::logic::perception::Object object;
  object.setObjectId(cones.cols());
  odcore::data::Container c1(object);
  c1.setSenderStamp(m_senderStamp);
  getConference().send(c1);

  for(int n = 0; n < cones.cols(); n++){

    opendlv::logic::sensation::Point conePoint;
    Cartesian2Spherical(cones(0,n), cones(1,n), cones(2,n), conePoint);

    opendlv::logic::perception::ObjectDirection coneDirection;
    coneDirection.setObjectId(n);
    coneDirection.setAzimuthAngle(-conePoint.getAzimuthAngle());  //Negative to convert to car frame from LIDAR
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
