/*
 * @Author: ding.yin
 * @Date: 2022-11-26 15:28:08
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-27 12:51:24
 */

#include <iostream>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "front_cam_node");
  ros::NodeHandle nh;
  ros::Rate r(30);

  int width;
  int height;
  nh.param<int>("width", width, 1280);
  nh.param<int>("height", height, 720);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher publisher = it.advertise("/front_cam", 1000);

  cv::VideoCapture cap;
  cv::Mat frame;
  int device_ID = 0;
  if (argc > 1) {
    device_ID = std::atoi(argv[1]);
  }
  int apiID = cv::CAP_ANY;
  cap.open(device_ID + apiID);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

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
      // cv::imshow("front cam", frame);
      // cv::waitKey(1);
      ros::Time time = ros::Time::now();
      std_msgs::Header header;
      header.stamp = time;
      header.frame_id = "/base_link";
      sensor_msgs::ImagePtr img_msg =
          cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      publisher.publish(img_msg);
    }
    // LOG(INFO) << "FRONT CAM";
    r.sleep();
  }

  return 0;
}