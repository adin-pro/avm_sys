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

bool GetAVMImage(const cv::Mat &front, const cv::Mat &back, const cv::Mat &left,
                 const cv::Mat &right) {
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_calib_node");
  ros::NodeHandle nh;
  ros::Rate r(30);
  std::string work_dir = "";
  nh.param<std::string>("work_dir", work_dir,
                        "/home/adin/catkin_ws/src/avm_sys/");
  YAML::Node node = YAML::LoadFile(work_dir + "config/config.yaml");
  std::string save_dir = node["save_dir"].as<std::string>();
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

  std::ofstream out(node["save_dir"].as<std::string>() + "/odom.txt",
                    std::ios::out);

  ImageData front_fisheye;
  ImageData back_fisheye;
  ImageData left_fisheye;
  ImageData right_fisheye;
  PoseData curr_odom;

  int frame_cnt = 0;

  // main loop
  while (ros::ok()) {
    ros::spinOnce();
    front_sub_ptr->parseData(front_data_buff);

    back_sub_ptr->parseData(back_data_buff);

    left_sub_ptr->parseData(left_data_buff);

    right_sub_ptr->parseData(right_data_buff);

    odom_sub_ptr->parseData(odom_data_buff);

    if (odom_data_buff.size()) {
      curr_odom = odom_data_buff.front();
      LOG(INFO) << "(x,y) "
                << "(" << curr_odom.pose(0, 3) << ", " << curr_odom.pose(1, 3)
                << ")";
      Eigen::Quaterniond q = curr_odom.getQuaternion();
      out << frame_cnt++ << std::fixed << std::setprecision(15) << " "
          << curr_odom.time << " " << curr_odom.pose(0, 3) << " "
          << curr_odom.pose(1, 3) << " " << curr_odom.pose(2, 3) << " " << q.x()
          << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      out.flush();
      odom_data_buff.pop_front();
    } else {
      continue;
    }

    // UndistortAndPublish(front_cam, front_data_buff, front_pub_ptr, "front");

    // UndistortAndPublish(back_cam, back_data_buff, back_pub_ptr, "back");

    // UndistortAndPublish(left_cam, left_data_buff, left_pub_ptr, "left");

    // UndistortAndPublish(right_cam, right_data_buff, right_pub_ptr, "right");

    ImageData::controlDuration(front_data_buff, 1.0);
    ImageData::controlDuration(back_data_buff, 1.0);
    ImageData::controlDuration(left_data_buff, 1.0);
    ImageData::controlDuration(right_data_buff, 1.0);

    double sync_time = curr_odom.time;

    if (front_data_buff.size()) {
      if (!ImageData::getImageDataByTS(front_data_buff, sync_time,
                                       front_fisheye))
        continue;
      if (!ImageData::getImageDataByTS(back_data_buff, sync_time, back_fisheye))
        continue;
      if (!ImageData::getImageDataByTS(left_data_buff, sync_time, left_fisheye))
        continue;
      if (!ImageData::getImageDataByTS(right_data_buff, sync_time,
                                       right_fisheye))
        continue;
    } else {
      continue;
    }

    cv::Mat front_undistort, back_undistort, left_undistort, right_undistort;

    front_cam.GetUndistortImage(front_fisheye.image, front_undistort);
    back_cam.GetUndistortImage(back_fisheye.image, back_undistort);
    left_cam.GetUndistortImage(left_fisheye.image, left_undistort);
    right_cam.GetUndistortImage(right_fisheye.image, right_undistort);

    cv::Mat front_ipm, back_ipm, left_ipm, right_ipm;

    cv::Mat IPM(cv::Size(800, 800), CV_8UC3, cv::Scalar(0, 0, 0));
    front_cam.WrapIPM(front_undistort, front_ipm, IPM.size());
    back_cam.WrapIPM(back_undistort, back_ipm, IPM.size());
    left_cam.WrapIPM(left_undistort, left_ipm, IPM.size());
    right_cam.WrapIPM(right_undistort, right_ipm, IPM.size());

    FishEye::GetAVM(front_ipm, back_ipm, left_ipm, right_ipm, IPM, node);
    cv::imshow("IPM", IPM);
    cv::imwrite(save_dir + "/" + std::to_string(frame_cnt) + ".png", IPM);
    char k = cv::waitKey(1);
    if (k == 'q') {
      break;
    } else if (k == 's') {
      cv::imwrite("/home/adin/extra/fisheye_samples/front_ipm.png", front_ipm);
      cv::imwrite("/home/adin/extra/fisheye_samples/back_ipm.png", back_ipm);
      cv::imwrite("/home/adin/extra/fisheye_samples/left_ipm.png", left_ipm);
      cv::imwrite("/home/adin/extra/fisheye_samples/right_ipm.png", right_ipm);
      cv::imwrite("/home/adin/extra/fisheye_samples/front.png", front_fisheye.image);
      cv::imwrite("/home/adin/extra/fisheye_samples/back.png", back_fisheye.image);
      cv::imwrite("/home/adin/extra/fisheye_samples/left.png", left_fisheye.image);
      cv::imwrite("/home/adin/extra/fisheye_samples/right.png", right_fisheye.image);
    }

    r.sleep();
  }
}