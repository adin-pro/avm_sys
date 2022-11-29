/*
 * @Author: ding.yin
 * @Date: 2022-11-27 14:19:14
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-27 15:56:25
 */

#include <deque>
#include <fstream>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "camera/fisheye.hpp"
#include "publisher/image_publisher.hpp"
#include "subscriber/image_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

using namespace avp_mapping;

bool UndistortAndPublish(FishEye &cam, std::deque<ImageData> &data_buff,
                         std::shared_ptr<ImagePublisher> &pub_ptr,
                         std::string tag) {
  if (data_buff.size()) {
    ImageData curr_data = data_buff.front();
    ImageData undistort_data = curr_data;
    undistort_data.image = curr_data.image.clone();
    LOG(INFO) << std::fixed << std::setprecision(15) << tag << " "
              << curr_data.time;
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
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr =
      std::make_shared<OdometrySubscriber>(nh, "/vins_estimator/odometry",
                                           1000);

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
  std::deque<PoseData> odom_data_buff;

  FishEye front_cam(work_dir + "config/front.yaml");
  FishEye back_cam(work_dir + "config/back.yaml");
  FishEye left_cam(work_dir + "config/left.yaml");
  FishEye right_cam(work_dir + "config/right.yaml");

  std::ofstream out("./odom.txt", std::ios::out);

  // main loop
  while (ros::ok()) {
    ros::spinOnce();
    front_sub_ptr->parseData(front_data_buff);

    back_sub_ptr->parseData(back_data_buff);

    left_sub_ptr->parseData(left_data_buff);

    right_sub_ptr->parseData(right_data_buff);

    odom_sub_ptr->parseData(odom_data_buff);

    if (odom_data_buff.size()) {
      PoseData curr_odom = odom_data_buff.front();
      LOG(INFO) << "(x,y) "
                << "(" << curr_odom.pose(0, 3) << ", " << curr_odom.pose(1, 3)
                << ")";
      Eigen::Quaterniond q = curr_odom.getQuaternion();
      out << curr_odom.time << " " << curr_odom.pose(0, 3) << " "
          << curr_odom.pose(1, 3) << " " << curr_odom.pose(2, 3) << " " << q.x()
          << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      out.flush();
      odom_data_buff.pop_front();
    }

    UndistortAndPublish(front_cam, front_data_buff, front_pub_ptr, "front");

    UndistortAndPublish(back_cam, back_data_buff, back_pub_ptr, "back");

    UndistortAndPublish(left_cam, left_data_buff, left_pub_ptr, "left");

    UndistortAndPublish(right_cam, right_data_buff, right_pub_ptr, "right");

    r.sleep();
  }
}