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

#include "logic-detectcone.hpp"

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

void DetectCone::efficientSlidingWindow(const std::string &dictionary, tiny_dnn::network<tiny_dnn::sequential> &nn, int width, int height) {
  using conv    = tiny_dnn::convolutional_layer;
  using relu    = tiny_dnn::relu_layer;
  tiny_dnn::core::backend_t backend_type = tiny_dnn::core::backend_t::avx;

  // nn << conv(width, height, 7, 3, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
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

  nn << conv(width, height, 7, 3, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-6, height-6, 7, 16, 16, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-12, height-12, 5, 16, 32, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-16, height-16, 5, 32, 32, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-20, height-20, 3, 32, 64, tiny_dnn::padding::valid, true, 1, 1, backend_type) << relu()
     << conv(width-22, height-22, 3, 64, 4, tiny_dnn::padding::valid, true, 1, 1, backend_type);

  // load nets
  std::ifstream ifs(dictionary.c_str());
  ifs >> nn;
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

void DetectCone::forwardDetection(tiny_dnn::network<tiny_dnn::sequential> nn) {
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

  cv::Size stdSize = cv::Size(640, 360);
  int width = imgSource.cols;
  int height = imgSource.rows;
  cv::Mat imgL(imgSource, cv::Rect(0, 0, width/2, height));
  cv::resize(imgL, imgL, stdSize);

  cv::Mat rectified;
  cv::resize(imgL, rectified, cv::Size (320, 180));

  auto patchImg = rectified(roi);

  // convert imagefile to vec_t
  tiny_dnn::vec_t data;
  convertImage(patchImg, inputWidth, inputHeight, data);

  // recognize
  auto startTime = std::chrono::system_clock::now();
  auto prob = nn.predict(data);
  auto endTime = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = endTime-startTime;
    std::cout << "Time: " << diff.count() << " s\n";

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
      if (label == 1){
        cv::circle(rectified, position, 1, {0, 255, 255}, -1);
        std::cout << "Find one yellow cone, xy position: " << position << " pixel, certainty: " 
        << probMapSoftmax.at<double>(cone[i]) << std::endl;
      }
      if (label == 2){
        cv::circle(rectified, position, 1, {255, 0, 0}, -1);
        std::cout << "Find one blue cone, xy position: " << position << " pixel, certainty: " 
        << probMapSoftmax.at<double>(cone[i]) << std::endl;
      }
      if (label == 3){
        cv::circle(rectified, position, 1, {0, 0, 255}, -1);
        std::cout << "Find one orange cone, xy position: " << position << " pixel, certainty: " 
        << probMapSoftmax.at<double>(cone[i]) << std::endl;
      }
    }
  }
//   cv::namedWindow("result", cv::WINDOW_NORMAL);
//   cv::imshow("result", rectified);
//   cv::waitKey(0);
}
