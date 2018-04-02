/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DETECTCONE
#define DETECTCONE

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <string>
#include <utility>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <thread>
#include <ctime>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <tiny_dnn/tiny_dnn.h>

class DetectCone {
   private:
    DetectCone(const DetectCone &) = delete;
    DetectCone(DetectCone &&)      = delete;
    DetectCone &operator=(const DetectCone &) = delete;
    DetectCone &operator=(DetectCone &&) = delete;

   public:
    DetectCone() = default;
    ~DetectCone() = default;

   public:
    void convertImage(cv::Mat, int, int, tiny_dnn::vec_t &);
    void efficientSlidingWindow(const std::string &, tiny_dnn::network<tiny_dnn::sequential> &, int, int);
    void softmax(cv::Vec4d, cv::Vec4d &);
    std::vector <cv::Point> imRegionalMax(cv::Mat, int, double, int);
    void forwardDetection(tiny_dnn::network<tiny_dnn::sequential>);
};

#endif

