/*
 * @Author: ding.yin
 * @Date: 2022-11-27 14:19:14
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-27 14:52:57
 */

#include <deque>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "camera/fisheye.hpp"
#include "publisher/image_publisher.hpp"
#include "subscriber/image_subscriber.hpp"

using namespace avp_mapping;

void UndistortAndPublish(FishEye &cam, std::deque<ImageData> &data_buff,
                         std::shared_ptr<ImagePublisher> pub_ptr) {
  if (data_buff.size()) {
    ImageData curr_data = data_buff.front();
    ImageData undistort_data = curr_data;
    cam.GetUndistortImage(curr_data.image, undistort_data.image);
    pub_ptr->publish(undistort_data.image, undistort_data.time);
    data_buff.pop_front();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_calib_node");
  ros::NodeHandle nh;
  ros::Rate r(30);
  std::string work_dir = "";
  nh.param<std::string>("work_dir", work_dir,
                        "/home/adin/catkin_ws/src/avm_sys/");

  std::shared_ptr<ImageSubscriber> front_sub_ptr =
      std::make_shared<ImageSubscriber>(nh, "/front_cam", 1000);
  std::shared_ptr<ImageSubscriber> back_sub_ptr =
      std::make_shared<ImageSubscriber>(nh, "/back_cam", 1000);
  std::shared_ptr<ImageSubscriber> left_sub_ptr =
      std::make_shared<ImageSubscriber>(nh, "/left_cam", 1000);
  std::shared_ptr<ImageSubscriber> right_sub_ptr =
      std::make_shared<ImageSubscriber>(nh, "/right_cam", 1000);

  std::shared_ptr<ImagePublisher> front_pub_ptr =
      std::make_shared<ImagePublisher>(nh, "/un_front_cam", "/map", 100);
  std::shared_ptr<ImagePublisher> back_pub_ptr =
      std::make_shared<ImagePublisher>(nh, "/un_back_cam", "/map", 100);
  std::shared_ptr<ImagePublisher> left_pub_ptr =
      std::make_shared<ImagePublisher>(nh, "/un_left_cam", "/map", 100);
  std::shared_ptr<ImagePublisher> right_pub_ptr =
      std::make_shared<ImagePublisher>(nh, "/un_right_cam", "/map", 100);

  std::deque<ImageData> front_data_buff;
  std::deque<ImageData> back_data_buff;
  std::deque<ImageData> left_data_buff;
  std::deque<ImageData> right_data_buff;

  FishEye front_cam(work_dir + "config/front.yaml");
  FishEye back_cam(work_dir + "config/back.yaml");
  FishEye left_cam(work_dir + "config/left.yaml");
  FishEye right_cam(work_dir + "config/right.yaml");

  // main loop
  while (ros::ok()) {
    ros::spinOnce();
    front_sub_ptr->parseData(front_data_buff);

    back_sub_ptr->parseData(back_data_buff);

    left_sub_ptr->parseData(left_data_buff);

    right_sub_ptr->parseData(right_data_buff);

    LOG(INFO) << "Calib " << ros::Time::now();

    UndistortAndPublish(front_cam, front_data_buff, front_pub_ptr);

    UndistortAndPublish(back_cam, back_data_buff, back_pub_ptr);

    UndistortAndPublish(left_cam, left_data_buff, left_pub_ptr);

    UndistortAndPublish(right_cam, right_data_buff, right_pub_ptr);

    r.sleep();
  }
}