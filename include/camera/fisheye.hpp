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

  void WrapIPM(const cv::Mat &src, cv::Mat &dst, const cv::Size &dst_size);

  static void smooth(std::string type, int center_px, int center_py,
                     cv::Mat &IPM_dst, const cv::Mat &mid, const cv::Mat &side);
  
  static void GetAVM(cv::Mat &front, cv::Mat &back, cv::Mat &left,
                     cv::Mat &right, cv::Mat& IPM, YAML::Node &node);

private:
  cv::Mat K_;
  cv::Mat D_;
  cv::Mat Homo_;
};