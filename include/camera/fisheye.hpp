/*
 * @Author: ding.yin
 * @Date: 2022-11-27 13:42:07
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-27 13:49:22
 */

#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

class FishEye {
public:
  FishEye(std::string config_file_path);

  void GetUndistortImage(const cv::Mat &origin, cv::Mat &undist);

private:
  cv::Mat K_;
  cv::Mat D_;
};