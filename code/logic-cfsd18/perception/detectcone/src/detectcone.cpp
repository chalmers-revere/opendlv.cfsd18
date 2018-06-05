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
, m_patchSize(64)
, m_width(672)
, m_height(376)
, m_yShift(0)
, m_zShift(1.872)
{
  m_diffVec = 0;
  m_pointMatched = Eigen::MatrixXd::Zero(4,1);
  m_lastCameraData = Eigen::MatrixXd::Zero(4,1);
  m_lastLidarData = Eigen::MatrixXd::Zero(4,1);
  m_coneCollector = Eigen::MatrixXd::Zero(4,2000);
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
    odcore::data::TimeStamp timeStamp = a_container.getSampleTimeStamp();
    // std::cout << timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds() << std::endl;
    // m_lastTimeStamp = timeStamp;
    if (!ExtractSharedImage(&sharedImg)) {
      std::cout << "[" << getName() << "] " << "[Unable to extract shared image." << std::endl;
      return;
    }else if(!m_recievedFirstImg){
      m_recievedFirstImg = true;
    }
    // std::thread coneCollector(&DetectCone::forwardDetectionORB, this);
    // coneCollector.detach();
    // forwardDetectionORB();
    // saveRecord(m_img);
      double timeDiff = timeStamp.toMicroseconds() - m_lastTimeStamp.toMicroseconds();
      std::cout << "Difference between image and cone data " << timeDiff << std::endl;
    if ((timeDiff > m_checkLiarMilliseconds*1000)){
      std::cout << "Lidar fails! Camera detection only! " << std::endl;
      m_lidarIsWorking = false;
      forwardDetectionORB();
    }
    else{
      m_lidarIsWorking = true;
      // std::cout << "Lidar is working!" << std::endl;
      std::vector<cv::Point3f> pts;
      std::vector<int> outputs;

      for (int i = 0; i < m_finalPointCloud.cols(); i++){
        pts.push_back(cv::Point3d(m_finalPointCloud(0,i), -m_yShift-m_finalPointCloud(2,i), m_zShift+m_finalPointCloud(1,i)));
      }
      if(!m_img.empty()){
        backwardDetection(m_img, pts, outputs);
        // cv::Mat rectified = m_img.colRange(0,672);
        // cv::resize(rectified, rectified, cv::Size(672, 376));
        // // rectified.convertTo(rectified, CV_8UC3);
        // // rectified.copyTo(result[1]);
        // // result[1].rowRange(320,680) = rectified;
        // cv::hconcat(result[0], m_img, coResult);
        // cv::imwrite("results/"+std::to_string(m_count++)+".png", coResult);
      }
      
      for (int i = 0; i < m_finalPointCloud.cols(); i++){
        m_finalPointCloud(3,i) = outputs[i];
      }
      SendMatchedContainer(m_finalPointCloud);
    }
    // if(!m_lidarIsWorking){
    //   forwardDetectionORB();
    // }
  }

  bool correctSenderStamp = a_container.getSenderStamp() == m_attentionSenderStamp;
  if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID() && correctSenderStamp) {
    // std::cout << "Recieved Direction" << std::endl;
    m_lastTimeStamp = a_container.getSampleTimeStamp();
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
    // std::cout << "Message Recieved " << std::endl;
    if (newFrameDist){
       std::thread coneCollector(&DetectCone::initializeCollection, this);
       coneCollector.detach();
       //initializeCollection();
    }
  }

  else if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID() && correctSenderStamp){
    // std::cout << "Recieved Distance" << std::endl;
    m_lastTimeStamp = a_container.getSampleTimeStamp();
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
    // std::cout << "Message Recieved " << std::endl;
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
    m_coneCollector = Eigen::MatrixXd::Zero(4,2000);
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
  return isExtracted;
}

void DetectCone::blockMatching(cv::Mat &disp, cv::Mat imgL, cv::Mat imgR){
  cv::Mat grayL, grayR;

  cv::cvtColor(imgL, grayL, CV_BGR2GRAY);
  cv::cvtColor(imgR, grayR, CV_BGR2GRAY);

  cv::StereoBM sbm;
  sbm.state->SADWindowSize = 17;
  sbm.state->numberOfDisparities = 32;

  sbm(grayL, grayR, disp, CV_32F);
  // disp /= 16;
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
  cv::Size stdSize = cv::Size(m_width, m_height);

  int width = img.cols;
  int height = img.rows;
  cv::Mat imgL(img, cv::Rect(0, 0, width/2, height));
  cv::Mat imgR(img, cv::Rect(width/2, 0, width/2, height));

  if(height != m_height){
    cv::resize(imgL, imgL, stdSize);
    cv::resize(imgR, imgR, stdSize);
  }

  cv::Mat R1, R2, P1, P2;
  cv::Rect validRoI[2];
  cv::stereoRectify(mtxLeft, distLeft, mtxRight, distRight, stdSize, R, T, R1, R2, P1, P2, Q,
    cv::CALIB_ZERO_DISPARITY, 0.0, stdSize, &validRoI[0], &validRoI[1]);

  cv::Mat rmap[2][2];
  cv::initUndistortRectifyMap(mtxLeft, distLeft, R1, P1, stdSize, CV_16SC2, rmap[0][0], rmap[0][1]);
  cv::initUndistortRectifyMap(mtxRight, distRight, R2, P2, stdSize, CV_16SC2, rmap[1][0], rmap[1][1]);
  cv::remap(imgL, imgL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
  cv::remap(imgR, imgR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);

  blockMatching(disp, imgL, imgR);

  rectified = imgL;

  cv::reprojectImageTo3D(disp, XYZ, Q);
}

void DetectCone::saveRecord(cv::Mat img){
  cv::imwrite("images/"+std::to_string(m_count++)+".png", img);
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


void DetectCone::slidingWindow(const std::string& dictionary) {
  using conv    = tiny_dnn::convolutional_layer;
  using fc      = tiny_dnn::fully_connected_layer;
  using tanh    = tiny_dnn::tanh_layer;
  using relu    = tiny_dnn::relu_layer;
  using softmax = tiny_dnn::softmax_layer;

  tiny_dnn::core::backend_t backend_type = tiny_dnn::core::default_engine();

  m_slidingWindow << conv(64, 64, 4, 3, 16, tiny_dnn::padding::valid, true, 2, 2, backend_type) << tanh()                                                   
     << conv(31, 31, 3, 16, 16, tiny_dnn::padding::valid, true, 2, 2, backend_type) << tanh() 
     // << dropout(15*15*16, 0.25)
     << conv(15, 15, 3, 16, 32, tiny_dnn::padding::valid, true, 2, 2, backend_type) << tanh() 
     << conv(7, 7, 3, 32, 32, tiny_dnn::padding::valid, true, 2, 2, backend_type) << tanh() 
     // << dropout(3*3*32, 0.25)                     
     << fc(3 * 3 * 32, 128, true, backend_type) << relu()  
     << fc(128, 5, true, backend_type) << softmax(5); 

  // load nets
  std::ifstream ifs(dictionary.c_str());
  ifs >> m_slidingWindow;
}



void DetectCone::backwardDetection(cv::Mat img, std::vector<cv::Point3f> pts, std::vector<int>& outputs){
  //Given RoI in 3D world, project back to the camera frame and then detect
  float_t threshold = 0.7f;
  cv::Mat disp, Q, rectified, XYZ;
  reconstruction(img, Q, disp, rectified, XYZ);
  std::vector<tiny_dnn::tensor_t> inputs;
  std::vector<int> verifiedIndex;
  std::vector<cv::Vec3i> porperty;
  outputs.clear();

  for(size_t i = 0; i < pts.size(); i++){
    cv::Point2f point2D;
    int radius;
    xyz2xy(Q, pts[i], point2D, radius);

    int x = point2D.x;
    int y = point2D.y;

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
      convertImage(patchImg, m_patchSize, m_patchSize, data);
      inputs.push_back({data});
      outputs.push_back(0);
      verifiedIndex.push_back(i);
      porperty.push_back(cv::Vec3i(x,y,radius));
    }
  }

  if(inputs.size()>0){
    auto prob = m_slidingWindow.predict(inputs);
    std::cout << "good4" << std::endl;
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
      std::cout << "good5" << std::endl;
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

  cv::namedWindow("img", cv::WINDOW_NORMAL);
  // cv::setWindowProperty("result", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  cv::imshow("img", rectified);
  cv::waitKey(10);

  cv::imwrite("results/"+std::to_string(m_count++)+".png", rectified);

  // for(size_t i = 0; i < pts.size(); i++)
  //   std::cout << i << ": " << outputs[i] << std::endl;
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

float DetectCone::median(std::vector<float> vec) {
  int size = vec.size();
  float tvecan;
  if (size % 2 == 0) { // even
    tvecan = (vec[vec.size() / 2 - 1] + vec[vec.size() / 2]) / 2;
  }

  else //odd
    tvecan = vec[vec.size() / 2];
  return tvecan;
}

float DetectCone::mean(std::vector<float> vec) {
  float result = 0;
  size_t size = vec.size();
  for(size_t i = 0; i < size; i++){
    result += vec[i];
  }
  result /= size;
  return result;
}

void DetectCone::gather_points(//初始化
  cv::Mat source,
  std::vector<float> vecQuery,
  std::vector<int>& vecIndex,
  std::vector<float>& vecDist
  )
{  
  double radius = 1;
  unsigned int max_neighbours = 100;
  cv::flann::KDTreeIndexParams indexParams(2);
  cv::flann::Index kdtree(source, indexParams); //此部分建立kd-tree索引同上例，故不做详细叙述
  cv::flann::SearchParams params(1024);//设置knnSearch搜索参数
  kdtree.radiusSearch(vecQuery, vecIndex, vecDist, radius, max_neighbours, params);
}

void DetectCone::filterKeypoints(std::vector<cv::Point3f>& point3Ds){
  std::vector<Pt> data;
  
  for(size_t i = 0; i < point3Ds.size(); i++){
    // if(point3Ds[i].y > 0.5 && point3Ds[i].y < 2){
      data.push_back(Pt{point3Ds[i],-1});
    // }
  }
  
  cv::Mat source = cv::Mat(point3Ds).reshape(1);
  point3Ds.clear();
  cv::Point3f point3D;
  int groupId = 0;

  for(size_t j = 0; j < data.size()-1; j++)
  {   
    if(data[j].group == -1){
      std::vector<float> vecQuery(3);//存放 查询点 的容器（本例都是vector类型）
      vecQuery[0] = data[j].pt.x;
      vecQuery[1] = data[j].pt.y;
      vecQuery[2] = data[j].pt.z;
      std::vector<int> vecIndex;
      std::vector<float> vecDist;

      gather_points(source, vecQuery, vecIndex, vecDist);//kd tree finish; find the points in the circle with point center vecQuery and radius, return index in vecIndex
      int num = 0;
      for(size_t i = 0; i < vecIndex.size(); i++){
        if(vecIndex[i]!=0)
          num++;
      }
      for (size_t i = 1; i < vecIndex.size(); i++){
        if (vecIndex[i] == 0 && vecIndex[i+1] != 0){
          num++;
        }
      }
      if (num == 0){
        if (data[j].group == -1){ 
          data[j].group = groupId++;
          point3D = data[j].pt;
          // std::cout<<j<<" type 1"<<" "<<data[j].pt.x<<","<<data[j].pt.y<<" group "<<data[j].group<<std::endl;
        }
      }
      else{   
        std::vector<Pt> groupAll;
        std::vector<int> filteredIndex;
        std::vector<float> centerPointX;
        std::vector<float> centerPointY;
        std::vector<float> centerPointZ;
        for (int v = 0; v < num; v++){
          groupAll.push_back(data[vecIndex[v]]);
          filteredIndex.push_back(vecIndex[v]);
        }
      
        int noGroup = 0;
        for(size_t i = 0; i < groupAll.size(); i++){
          if(groupAll[i].group == -1)
            noGroup++;
        }

        if (noGroup > 0){
          for (size_t k = 0; k < filteredIndex.size(); k++)
          { 
            if (data[filteredIndex[k]].group == -1)
            { 
              data[filteredIndex[k]].group = groupId;
              centerPointX.push_back(data[vecIndex[k]].pt.x);
              centerPointY.push_back(data[vecIndex[k]].pt.y);
              centerPointZ.push_back(data[vecIndex[k]].pt.z);
            }
          }
          groupId++;
          point3D.x = mean(centerPointX);
          point3D.y = mean(centerPointY);
          point3D.z = mean(centerPointZ);
        }
        else{
          data[j].group = data[vecIndex[0]].group;
          point3D = data[j].pt;
          // std::cout<<j<<" type 2"<<" "<<data[j].pt.x<<","<<data[j].pt.y<<" group "<<data[j].group<<std::endl;
        }
      }
      if(std::isnan(point3D.x)||std::isnan(point3D.y)||std::isnan(point3D.y))
        continue;
      point3Ds.push_back(point3D);
    }
  }
}

void DetectCone::xyz2xy(cv::Mat Q, cv::Point3f xyz, cv::Point2f& xy, int& radius){
  float X = xyz.x;
  float Y = xyz.y;
  float Z = xyz.z;
  float Cx = float(-Q.at<double>(0,3));
  float Cy = float(-Q.at<double>(1,3));
  float f = float(Q.at<double>(2,3));
  float a = float(Q.at<double>(3,2));
  float b = float(Q.at<double>(3,3));
  float d = (f - Z * b ) / ( Z * a);
  xy.x = X * ( d * a + b ) + Cx;
  xy.y = Y * ( d * a + b ) + Cy;
  radius = int(0.4f * ( d * a + b ));
}

void DetectCone::whiteBalance(cv::Mat img){
  auto meanValue = cv::mean(img);
  std::cout << "mean value: " << meanValue << std::endl;
}

void DetectCone::forwardDetectionORB(){
  //Given RoI by SIFT detector and detected by CNN
  if(m_img.empty()){
    return;
  }
  cv::Mat img;
  m_img.copyTo(img);
  double threshold = 0.1;

  std::vector<tiny_dnn::tensor_t> inputs;
  std::vector<int> verifiedIndex;
  std::vector<cv::Point> candidates;
  // std::vector<int> outputs;

  cv::Mat Q, disp, XYZ, imgRoI, imgSource;
  reconstruction(img, Q, disp, img, XYZ);
  img.copyTo(imgSource);

  int rowT = 190;
  int rowB = 320;
  imgRoI = img.rowRange(rowT, rowB);
  // whiteBalance(imgRoI);

  cv::ORB detector(100);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(imgRoI, keypoints);

  cv::Mat probMap = cv::Mat::zeros(m_height, m_width, CV_64F);
  cv::Mat indexMap = cv::Mat::zeros(m_height, m_width, CV_32S);

  std::vector<cv::Point3f> point3Ds;
  cv::Point2f point2D;
  for(size_t i = 0; i < keypoints.size(); i++){
    cv::Point position(int(keypoints[i].pt.x), int(keypoints[i].pt.y)+rowT);
    cv::Point3f point3D = XYZ.at<cv::Point3f>(position);
    if(point3D.y>0.8f && point3D.y<1.1f){
      point3Ds.push_back(point3D);
    }
    // std::cout << "position" << position << " " << XYZ.at<cv::Point3f>(position) << std::endl;
  }
  filterKeypoints(point3Ds);
  for(size_t i = 0; i < point3Ds.size(); i++){
    int radius;
    xyz2xy(Q, point3Ds[i], point2D, radius);
    int x = int(point2D.x);
    int y = int(point2D.y);

    cv::Rect roi;
    roi.x = std::max(x - radius, 0);
    roi.y = std::max(y - radius, 0);
    roi.width = std::min(x + radius, img.cols) - roi.x;
    roi.height = std::min(y + radius, img.rows) - roi.y;

    // cv::circle(img, cv::Point (x,y), 2, cv::Scalar (0,0,0));
    // cv::namedWindow("roi", cv::WINDOW_NORMAL);
    // cv::imshow("roi", img);
    // cv::waitKey(0);
    //cv::destroyAllWindows();

    if (0 > roi.x || 0 >= roi.width || roi.x + roi.width > img.cols || 0 > roi.y || 0 >= roi.height || roi.y + roi.height > img.rows){
      std::cout << "Wrong roi!" << std::endl;
      continue;
    }
    else{
      auto patchImg = img(roi);
      // cv::namedWindow("roi", cv::WINDOW_NORMAL);
      // cv::imshow("roi", patchImg);
      // cv::waitKey(0);
      tiny_dnn::vec_t data;
      convertImage(patchImg, m_patchSize, m_patchSize, data);
      inputs.push_back({data});
      // outputs.push_back(0);
      verifiedIndex.push_back(i);
      candidates.push_back(cv::Point(x,y));
    }
  }

  // // show img result
  // int resultWidth = m_height;
  // int resultHeight = m_height;
  // double resultResize = 20;
  // cv::Mat result = cv::Mat::zeros(resultHeight, resultWidth, CV_8UC3);
  // std::string labels[] = {"background", "blue", "yellow", "orange", "big orange"};
  // if(inputs.size()>0){
  //   auto prob = m_slidingWindow.predict(inputs);
  //   for(size_t i = 0; i < inputs.size(); i++){
  //     size_t maxIndex = 0;
  //     float_t maxProb = prob[i][0][0];
  //     for(size_t j = 1; j < 5; j++){
  //       if(prob[i][0][j] > maxProb){
  //         maxIndex = j;
  //         maxProb = prob[i][0][j];
  //       }
  //     }
  //     // outputs[verifiedIndex[i]] = maxIndex;
  //     int x = candidates[i].x;
  //     int y = candidates[i].y;
  //     probMap.at<double>(y,x) = maxProb;
  //     indexMap.at<int>(y,x) = maxIndex;
  //   }
  //   std::vector <cv::Point> cones = imRegionalMax(probMap, 10, threshold, 10);
  //   m_finalPointCloud = Eigen::MatrixXd::Zero(4,cones.size());
  //   for(size_t i = 0; i < cones.size(); i++){
  //     int x = cones[i].x;
  //     int y = cones[i].y;
  //     double maxProb = probMap.at<double>(y,x);
  //     int maxIndex = indexMap.at<int>(y,x);
  //     cv::Point position(x, y);
  //     cv::Point3f point3D = XYZ.at<cv::Point3f>(position);
  //     m_finalPointCloud(0,i) = point3D.x;
  //     m_finalPointCloud(1,i) = point3D.z-m_zShift;
  //     m_finalPointCloud(2,i) = -(point3D.y-m_yShift);
  //     m_finalPointCloud(3,i) = maxIndex;
  //     std::string labelName = labels[maxIndex];
  //     // float_t ratio = depth2resizeRate(point3D.x, point3D.z);
  //     // int length = ratio * 25;
  //     // int radius = (length-1)/2;
  //     int radius;
  //     cv::Point2f position_tmp;
  //     xyz2xy(Q, point3D, position_tmp, radius);
  //     // std::cout << x << " " << y << " " << point3D << " " << radius << std::endl;

  //     if(radius>0){
  //       if (labelName == "background"){
  //         std::cout << "No cone detected" << std::endl;
  //         cv::circle(img, position, radius, cv::Scalar (0,0,0));
  //       } 
  //       else{
  //         if (labelName == "blue")
  //           cv::circle(img, position, radius, cv::Scalar (255,0,0), 2);
  //         else if (labelName == "yellow")
  //           cv::circle(img, position, radius, cv::Scalar (0,255,255), 2);
  //         else if (labelName == "orange")
  //           cv::circle(img, position, radius, cv::Scalar (0,165,255), 2);
  //         else if (labelName == "big orange")
  //           cv::circle(img, position, radius, cv::Scalar (0,0,255), 2);

  //         int xt = int(point3D.x * float(resultResize) + resultWidth/2);
  //         int yt = int(point3D.z * float(resultResize));
  //         if (xt >= 0 && xt <= resultWidth && yt >= 0 && yt <= resultHeight){
  //           if (labelName == "blue")
  //             cv::circle(result, cv::Point (xt,yt), 5, cv::Scalar (255,0,0), -1);
  //           else if (labelName == "yellow")
  //             cv::circle(result, cv::Point (xt,yt), 5, cv::Scalar (0,255,255), -1);
  //           else if (labelName == "orange")
  //             cv::circle(result, cv::Point (xt,yt), 5, cv::Scalar (0,165,255), -1);
  //           else if (labelName == "big orange")
  //             cv::circle(result, cv::Point (xt,yt), 10, cv::Scalar (0,0,255), -1);
  //         }

  //         std::cout << position << " " << labelName << " " << point3D << " " << maxProb << std::endl;
  //       }
  //     }
  //   }
  // }
      

  // for(size_t i = 0; i < keypoints.size(); i++){
  //   cv::circle(img, cv::Point(int(keypoints[i].pt.x),int(keypoints[i].pt.y)+rowT), 2, cv::Scalar (255,255,255), -1);
  // }

  // cv::line(img, cv::Point(0,rowT), cv::Point(m_width,rowT), cv::Scalar(0,0,255), 2);
  // cv::line(img, cv::Point(0,rowB), cv::Point(m_width,rowB), cv::Scalar(0,0,255), 2);


  // show lidar and camera result
  int resultWidth = 672;
  int resultHeight = 376;
  float resultResize = 20.0f;
  cv::Mat result[2] = cv::Mat::zeros(resultHeight, resultWidth, CV_8UC3), coResult;
  std::string labels[] = {"background", "blue", "yellow", "orange", "big orange"};
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
      // outputs[verifiedIndex[i]] = maxIndex;
      int x = candidates[i].x;
      int y = candidates[i].y;
      probMap.at<double>(y,x) = maxProb;
      indexMap.at<int>(y,x) = maxIndex;
    }
    std::vector <cv::Point> cones = imRegionalMax(probMap, 10, threshold, 10);
    // m_finalPointCloud = Eigen::MatrixXd::Zero(4,cones.size());
    for(size_t i = 0; i < cones.size(); i++){
      int x = cones[i].x;
      int y = cones[i].y;
      // double maxProb = probMap.at<double>(y,x);
      int maxIndex = indexMap.at<int>(y,x);
      cv::Point position(x, y);
      cv::Point3f point3D = XYZ.at<cv::Point3f>(position);
      // m_finalPointCloud(0,i) = point3D.x;
      // m_finalPointCloud(1,i) = point3D.z-m_zShift;
      // m_finalPointCloud(2,i) = -(point3D.y-m_yShift);
      // m_finalPointCloud(3,i) = maxIndex;
      std::string labelName = labels[maxIndex];
      // float_t ratio = depth2resizeRate(point3D.x, point3D.z);
      // int length = ratio * 25;
      // int radius = (length-1)/2;
      int radius;
      cv::Point2f position_tmp;
      xyz2xy(Q, point3D, position_tmp, radius);
      // std::cout << x << " " << y << " " << point3D << " " << radius << std::endl;  

      if(radius>0){
        if (labelName == "background"){
          std::cout << "No cone detected" << std::endl;
          cv::circle(img, position, radius, cv::Scalar (0,0,0));
        } 
        else{
          if (labelName == "blue")
            cv::circle(img, position, radius, cv::Scalar (255,0,0), 2);
          else if (labelName == "yellow")
            cv::circle(img, position, radius, cv::Scalar (0,255,255), 2);
          else if (labelName == "orange")
            cv::circle(img, position, radius, cv::Scalar (0,165,255), 2);
          else if (labelName == "big orange")
            cv::circle(img, position, radius, cv::Scalar (0,0,255), 2);

          int xt = int(point3D.x * resultResize + resultWidth/2);
          int yt = int((point3D.z-float(m_zShift)) * resultResize);
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
          // savefile << std::to_string(position.x)+","+std::to_string(position.y)+","+labelName+","+std::to_string(point3D.x)+","+std::to_string(point3D.y)+","+std::to_string(point3D.z)+"\n";
        }
      }
    }
  }

  for(int i = 0; i < m_finalPointCloud.cols(); i++){
    // savefile << std::to_string(m_finalPointCloud(0,i))+","+std::to_string(m_finalPointCloud(1,i))+","+std::to_string(m_finalPointCloud(2,i))+"\n";
    int x = int(m_finalPointCloud(0,i) * double(resultResize) + resultWidth/2);
    int y = int(m_finalPointCloud(1,i) * double(resultResize));
    if (x >= 0 && x <= resultWidth && y >= 0 && y <= resultHeight){
      cv::circle(result[0], cv::Point (x,y), 5, cv::Scalar (255, 255, 255), -1);
    }
  }

  cv::flip(result[0], result[0], 0);
  img.copyTo(result[1].rowRange(resultHeight-376,resultHeight));
  cv::hconcat(result[1], result[0], coResult);

  cv::imwrite("results/"+std::to_string(m_count++)+".png", coResult);

  // cv::namedWindow("coResult", cv::WINDOW_NORMAL); 
  // cv::imshow("coResult", coResult);
  // cv::waitKey(10);

  SendMatchedContainer(m_finalPointCloud);
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
  m_finalPointCloud = lidarCones;

  // std::vector<cv::Point3f> pts;
  // std::vector<int> outputs;

  // // int width = 672;
  // // int height = 376;
  // // double resultResize = 10;
  // // cv::Mat result[2] = cv::Mat::zeros(height, width, CV_8UC3), coResult;

  // for (int i = 0; i < m_finalPointCloud.cols(); i++){
  //   pts.push_back(cv::Point3d(m_finalPointCloud(0,i), -m_yShift-m_finalPointCloud(2,i), m_zShift+m_finalPointCloud(1,i)));
  // //   int x = int(m_finalPointCloud(0,i) * resultResize + width/2);
  // //   int y = int(m_finalPointCloud(1,i) * resultResize);
  // //   if (x >= 0 && x <= width && y >= 0 && y <= height){
  // //     cv::circle(result[0], cv::Point (x,y), 2, cv::Scalar (255, 255, 255), -1);
  // //   }
  // }

  // // cv::flip(result[0], result[0], 0);
  // if(!m_img.empty()){
  //   backwardDetection(m_img, pts, outputs);
  //   // cv::Mat rectified = m_img.colRange(0,672);
  //   // cv::resize(rectified, rectified, cv::Size(672, 376));
  //   // // rectified.convertTo(rectified, CV_8UC3);
  //   // // rectified.copyTo(result[1]);
  //   // // result[1].rowRange(320,680) = rectified;
  //   // cv::hconcat(result[0], m_img, coResult);
  //   // cv::imwrite("results/"+std::to_string(m_count++)+".png", coResult);
  // }
  
  // for (int i = 0; i < m_finalPointCloud.cols(); i++){
  //   m_finalPointCloud(3,i) = outputs[i];
  // }

  // //std::cout << "matched: " << std::endl;
  // //std::cout << m_finalPointCloud << std::endl;
  // SendMatchedContainer(m_finalPointCloud);
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
    c2.setSampleTimeStamp(m_lastTimeStamp);
    getConference().send(c2);

    opendlv::logic::perception::ObjectDistance coneDistance;
    coneDistance.setObjectId(n);
    coneDistance.setDistance(conePoint.getDistance());
    odcore::data::Container c3(coneDistance);
    c3.setSenderStamp(m_senderStamp);
    c3.setSampleTimeStamp(m_lastTimeStamp);
    getConference().send(c3);

    opendlv::logic::perception::ObjectType coneType;
    coneType.setObjectId(n);
    coneType.setType(cones(3,n));
    odcore::data::Container c4(coneType);
    c4.setSenderStamp(m_senderStamp);
    c4.setSampleTimeStamp(m_lastTimeStamp);
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
