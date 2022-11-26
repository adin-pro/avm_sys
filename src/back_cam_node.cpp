/*
 * @Author: ding.yin
 * @Date: 2022-11-26 15:28:08
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-26 16:01:30
 */

#include <iostream>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "back_cam_node");
  ros::NodeHandle nh;
  ros::Rate r(30);

  cv::VideoCapture cap;
  cv::Mat frame;
  int device_ID = 0;
  if (argc > 1) {
    device_ID = std::atoi(argv[1]);
  }
  int apiID = cv::CAP_ANY;
  cap.open(device_ID + apiID);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  

  if (!cap.isOpened()) {
    LOG(ERROR) << "Cap " << device_ID << " failed to open";
    return -1;
  }


  while (ros::ok()) {
    ros::spinOnce();
    cap.read(frame);
    if (frame.empty()) {
      LOG(INFO) << "Empty Frame";
    } else {
      LOG(INFO) << ros::Time::now();
      cv::imshow("back cam", frame);
      cv::waitKey(1);
    }
    LOG(INFO) << "BACK CAM";
    r.sleep();
  }

  return 0;
}