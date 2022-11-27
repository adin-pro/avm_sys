/*
 * @Author: ding.yin
 * @Date: 2022-11-27 14:22:24
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-27 14:34:23
 */

#include "camera/fisheye.hpp"
#include <iostream>

FishEye::FishEye(std::string config_file_path) {
  YAML::Node node = YAML::LoadFile(config_file_path);
  cv::Mat K = cv::Mat_<double>(3, 3);
  cv::Mat D = cv::Mat_<double>(4, 1);
  for (int i = 0; i < 9; ++i) {
    K.at<double>(i / 3, i % 3) = node["K"][i].as<double>();
  }
  for (int i = 0; i < 4; ++i) {
    D.at<double>(i) = node["D"][i].as<double>();
  }
  K_ = K.clone();
  D_ = D.clone();
  std::cout << K_ << std::endl;
  std::cout << D_ << std::endl;
}

void FishEye::GetUndistortImage(const cv::Mat &origin, cv::Mat &undist) {
  cv::Mat map1, map2;

  // std::cout << graysize.height << " " << graysize.width << std::endl;
  cv::fisheye::initUndistortRectifyMap(K_, D_, cv::Mat(), K_,
                                       origin.size(), CV_32FC1, map1, map2);
  // std::cout << "Get undistort matrix" << std::endl;
  cv::remap(origin, undist, map1, map2, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);
}