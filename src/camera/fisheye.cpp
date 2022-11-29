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
  cv::Mat Homo = cv::Mat_<double>(3, 3);
  for (int i = 0; i < 9; ++i) {
    K.at<double>(i / 3, i % 3) = node["K"][i].as<double>();
  }
  for (int i = 0; i < 4; ++i) {
    D.at<double>(i) = node["D"][i].as<double>();
  }
  for (int i = 0; i < 9; ++i) {
    Homo.at<double>(i / 3, i % 3) = node["H"][i].as<double>();
  }
  K_ = K.clone();
  D_ = D.clone();
  Homo_ = Homo.clone();
  std::cout << "Load YAML Node from file: " << config_file_path << std::endl;
}

void FishEye::GetUndistortImage(const cv::Mat &origin, cv::Mat &undist) {
  cv::Mat map1, map2;

  // std::cout << graysize.height << " " << graysize.width << std::endl;
  cv::fisheye::initUndistortRectifyMap(
      K_, D_, cv::Mat(), K_, cv::Size(1600, 800), CV_32FC1, map1, map2);
  // std::cout << "Get undistort matrix" << std::endl;
  cv::remap(origin, undist, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void FishEye::WrapIPM(const cv::Mat &src, cv::Mat &dst,
                      const cv::Size &dst_size) {
  cv::warpPerspective(src, dst, Homo_, dst_size);
}

void FishEye::smooth(std::string type, int center_px, int center_py,
                     cv::Mat &IPM_dst, const cv::Mat &mid,
                     const cv::Mat &side) {

  if (type == "left_front") {
    for (int y = 0; y < center_py; ++y) {
      uchar *p = IPM_dst.ptr<uchar>(y);
      const uchar *p_mid = mid.ptr<uchar>(y);
      const uchar *p_side = side.ptr<uchar>(y);
      for (int x = 0; x < center_px; ++x) {
        double theta = atan((center_py - y + 1e-15) / (center_px - x + 1e-15));
        if (p_side[3 * x] == 0) {
          theta = CV_PI / 2.0;
        } else if (p_mid[3 * x] == 0) {
          theta = 0.0;
        }
        p[3 * x] = cos(theta) * cos(theta) * p_side[3 * x] +
                   sin(theta) * sin(theta) * p_mid[3 * x];
        p[3 * x + 1] = cos(theta) * cos(theta) * p_side[3 * x + 1] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 1];
        p[3 * x + 2] = cos(theta) * cos(theta) * p_side[3 * x + 2] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 2];
      }
    }
  } else if (type == "right_front") {
    for (int y = 0; y < center_py; ++y) {
      uchar *p = IPM_dst.ptr<uchar>(y);
      const uchar *p_mid = mid.ptr<uchar>(y);
      const uchar *p_side = side.ptr<uchar>(y);
      for (int x = center_px; x < IPM_dst.cols; ++x) {
        double theta = atan((center_py - y + 1e-15) / (x - center_px + 1e-15));
        if (p_side[3 * x] == 0) {
          theta = CV_PI / 2.0;
        } else if (p_mid[3 * x] == 0) {
          theta = 0.0;
        }
        p[3 * x] = cos(theta) * cos(theta) * p_side[3 * x] +
                   sin(theta) * sin(theta) * p_mid[3 * x];
        p[3 * x + 1] = cos(theta) * cos(theta) * p_side[3 * x + 1] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 1];
        p[3 * x + 2] = cos(theta) * cos(theta) * p_side[3 * x + 2] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 2];
      }
    }
  } else if (type == "left_back") {
    for (int y = center_py; y < IPM_dst.rows; ++y) {
      uchar *p = IPM_dst.ptr<uchar>(y);
      const uchar *p_mid = mid.ptr<uchar>(y);
      const uchar *p_side = side.ptr<uchar>(y);
      for (int x = 0; x < center_px; ++x) {
        double theta = atan((y - center_py + 1e-15) / (center_px - x + 1e-15));
        if (p_side[3 * x] == 0) {
          theta = CV_PI / 2.0;
        } else if (p_mid[3 * x] == 0) {
          theta = 0.0;
        }
        p[3 * x] = cos(theta) * cos(theta) * p_side[3 * x] +
                   sin(theta) * sin(theta) * p_mid[3 * x];
        p[3 * x + 1] = cos(theta) * cos(theta) * p_side[3 * x + 1] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 1];
        p[3 * x + 2] = cos(theta) * cos(theta) * p_side[3 * x + 2] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 2];
      }
    }
  } else if (type == "right_back") {
    for (int y = center_py; y < IPM_dst.rows; ++y) {
      uchar *p = IPM_dst.ptr<uchar>(y);
      const uchar *p_mid = mid.ptr<uchar>(y);
      const uchar *p_side = side.ptr<uchar>(y);
      for (int x = center_px; x < IPM_dst.cols; ++x) {
        double theta = atan((y - center_py + 1e-15) / (x - center_px + 1e-15));
        if (p_side[3 * x] == 0) {
          theta = CV_PI / 2.0;
        } else if (p_mid[3 * x] == 0) {
          theta = 0.0;
        }
        p[3 * x] = cos(theta) * cos(theta) * p_side[3 * x] +
                   sin(theta) * sin(theta) * p_mid[3 * x];
        p[3 * x + 1] = cos(theta) * cos(theta) * p_side[3 * x + 1] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 1];
        p[3 * x + 2] = cos(theta) * cos(theta) * p_side[3 * x + 2] +
                       sin(theta) * sin(theta) * p_mid[3 * x + 2];
      }
    }
  }
}

void FishEye::GetAVM(cv::Mat &front_all, cv::Mat &back_all, cv::Mat &left_all,
                     cv::Mat &right_all, cv::Mat &IPM, YAML::Node &node) {

  cv::Mat front =
      cv::Mat(front_all.size(), front_all.type(), cv::Scalar(0, 0, 0));
  cv::Mat back =
      cv::Mat(front_all.size(), front_all.type(), cv::Scalar(0, 0, 0));
  cv::Mat left =
      cv::Mat(front_all.size(), front_all.type(), cv::Scalar(0, 0, 0));
  cv::Mat right =
      cv::Mat(front_all.size(), front_all.type(), cv::Scalar(0, 0, 0));

  // color rectify
  static std::vector<int> color_bias =
      node["color_bias"].as<std::vector<int>>();
  static bool use_smooth = node["use_smooth"].as<bool>();
  for (int m = 0; m < front_all.rows; ++m) {
    uchar *p = front_all.ptr<uchar>(m);
    for (int n = 0; n < front_all.cols; ++n) {
      if (p[3 * n] == 0 || p[3 * n] >= 255 - color_bias[0] ||
          p[3 * n] <= -color_bias[0] || p[3 * n + 1] == 0 ||
          p[3 * n + 1] >= 255 - color_bias[1] ||
          p[3 * n + 1] <= -color_bias[1] || p[3 * n + 2] == 0 ||
          p[3 * n + 2] <= -color_bias[2] || p[3 * n + 2] >= 255 - color_bias[2])
        continue;
      p[3 * n] = color_bias[0] + p[3 * n];         // b
      p[3 * n + 1] = color_bias[1] + p[3 * n + 1]; // g
      p[3 * n + 2] = color_bias[2] + p[3 * n + 2]; // r
    }
  }

  cv::Mat front_mask, back_mask, left_mask, right_mask;
  // front
  cv::Mat front_less = front_all(cv::Rect(0, 0, 800, 400)).clone();

  cv::inRange(front_less, cv::Scalar(1, 1, 1), cv::Scalar(255, 255, 255),
              front_mask);
  cv::erode(front_mask, front_mask, cv::Mat());
  cv::Mat front_roi = front(cv::Rect(0, 0, 800, 400));
  cv::Mat front_roi_ipm = IPM(cv::Rect(0, 0, 800, 400));
  front_less.copyTo(front_roi, front_mask);
  front_less.copyTo(front_roi_ipm, front_mask);

  // back
  cv::Mat back_less = back_all(cv::Rect(0, 550, 800, 250)).clone();
  cv::inRange(back_less, cv::Scalar(1, 1, 1), cv::Scalar(255, 255, 255),
              back_mask);
  cv::erode(back_mask, back_mask, cv::Mat());
  cv::Mat back_roi = back(cv::Rect(0, 550, 800, 250));
  cv::Mat back_roi_ipm = IPM(cv::Rect(0, 550, 800, 250));
  back_less.copyTo(back_roi, back_mask);
  back_less.copyTo(back_roi_ipm, back_mask);

  // left
  cv::Mat left_less = left_all(cv::Rect(0, 0, 400, 800)).clone();
  cv::inRange(left_less, cv::Scalar(1, 1, 1), cv::Scalar(255, 255, 255),
              left_mask);
  cv::erode(left_mask, left_mask, cv::Mat());
  cv::Mat left_roi = left(cv::Rect(0, 0, 400, 800));
  cv::Mat left_roi_ipm = IPM(cv::Rect(0, 0, 400, 800));
  left_less.copyTo(left_roi, left_mask);
  left_less.copyTo(left_roi_ipm, left_mask);

  // right
  cv::Mat right_less = right_all(cv::Rect(400, 0, 400, 800)).clone();
  cv::inRange(right_less, cv::Scalar(1, 1, 1), cv::Scalar(255, 255, 255),
              right_mask);
  cv::erode(right_mask, right_mask, cv::Mat());
  cv::Mat right_roi = right(cv::Rect(400, 0, 400, 800));
  cv::Mat right_roi_ipm = IPM(cv::Rect(400, 0, 400, 800));
  right_less.copyTo(right_roi, right_mask);
  right_less.copyTo(right_roi_ipm, right_mask);

  if (use_smooth) {
    smooth("left_front", 400, 400, IPM, front, left);
    smooth("right_front", 400, 400, IPM, front, right);
    smooth("left_back", 400, 550, IPM, back, left);
    smooth("right_back", 400, 550, IPM, back, right);
  }

  cv::Point pt1(350, 350);
  // and its bottom right corner.
  cv::Point pt2(450, 560);
  // These two calls...
  cv::rectangle(IPM, pt1, pt2, cv::Scalar(0, 0, 0), -1);
}
