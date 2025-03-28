// Copyright 2024 MITSUBA Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

class SampleNode : public rclcpp::Node {
 public:
  SampleNode() : Node("sample_node") {
    param1_ = this->declare_parameter("param1", 0.394);
    param2_ = this->declare_parameter("param2", 0.204);
    RCLCPP_INFO(this->get_logger(), "param1_:%lf param2_:%lf", param1_, param2_);

    ros_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    param_handler_ = this->add_on_set_parameters_callback(
        std::bind(&SampleNode::param_callback, this, std::placeholders::_1));

    create_files();
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_marker_array_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);

    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&SampleNode::joy_callback, this, std::placeholders::_1));
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&SampleNode::scan_callback, this, std::placeholders::_1));
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud", rclcpp::SensorDataQoS(),
        std::bind(&SampleNode::cloud_callback, this, std::placeholders::_1));
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&SampleNode::cmd_vel_callback, this, std::placeholders::_1));
    sub_clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(&SampleNode::clicked_point_callback, this, std::placeholders::_1));
    sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&SampleNode::goal_pose_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&SampleNode::odom_callback, this, std::placeholders::_1));
    sub_can_sent_ = this->create_subscription<can_msgs::msg::Frame>(
        "/to_can_bus", 10,
        std::bind(&SampleNode::sent_messages_callback, this, std::placeholders::_1));
    sub_can_recv_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", 10,
        std::bind(&SampleNode::received_messages_callback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_50ms_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&SampleNode::timer_50ms_callback, this));
    timer_200ms_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                           std::bind(&SampleNode::timer_200ms_callback, this));
  }

 private:
  std::mutex mtx_;
  std::shared_ptr<rclcpp::Clock> ros_clock_;                //時刻取得用
  OnSetParametersCallbackHandle::SharedPtr param_handler_;  //パラメータハンドラ
  std::ofstream of_log_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_sent_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_recv_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_50ms_;
  rclcpp::TimerBase::SharedPtr timer_200ms_;
  double param1_, param2_;  // launchファイルで設定するパラメータ
  std::vector<can_msgs::msg::Frame> can_msg_array_;  //コールバック関数間で使用する配列
  visualization_msgs::msg::MarkerArray marker_array_;  //コールバック関数間で使用するROS配列
  geometry_msgs::msg::TransformStamped trans_form_;

  rcl_interfaces::msg::SetParametersResult param_callback(
      const std::vector<rclcpp::Parameter>& parameters)  //パラメータ変更コールバック関数
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto&& param : parameters) {
      std::string param_name = param.get_name();
      if (param_name == "param1")
        param1_ = param.as_double();
      else if (param_name == "param2")
        param2_ = param.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  void create_files(void) {
    time_t t = time(NULL);
    char strTime[128];
    strftime(strTime, sizeof(strTime), "20%y%m%d_%H%M%S", localtime(&t));
    char fileNameCmd[256];
    snprintf(fileNameCmd, sizeof(fileNameCmd), "%s/log/log_%s.txt", getenv("HOME"), strTime);
    of_log_.open(fileNameCmd);
    //			of_log_ << "time[s] " << "sample_label1[m/s] " << "sample_label2[m/s] " <<
    //"sample_label3[m/s] " << "sample_label4[m/s] " << std::endl;
  }

  int get_transform(std::string frame_from, std::string frame_to) {
    try {
      trans_form_ = tf_buffer_->lookupTransform(frame_from, frame_to, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", frame_to.c_str(),
                  frame_from.c_str(), ex.what());
      return -1;
    }
    std::cout << "trans_form_:" << trans_form_.transform.translation.x << " "
              << trans_form_.transform.translation.y << " " << trans_form_.transform.translation.z
              << std::endl;
    std::cout << trans_form_.transform.rotation.x << " " << trans_form_.transform.rotation.y << " "
              << trans_form_.transform.rotation.z << " " << trans_form_.transform.rotation.w
              << std::endl;
    return 0;
  }

  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();

    std::cout << std::fixed << now.seconds() << " /joy subscribed"
              << " axes[0] = " << msg->axes[0] << " buttons[0] = " << msg->buttons[0] << std::endl;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();
    double angle_min = msg->angle_min;
    double angle_max = msg->angle_max;
    double angle_increment = msg->angle_increment;
    auto ranges = msg->ranges;

    for (int i = 0; i < (int)ranges.size(); i++) {
      double angle = angle_min + angle_increment * (double)i;  //[rad]
      double range = ranges[i];                                //[m]
      std::cout << i << " " << angle << " " << range << std::endl;
    }

    std::cout << std::fixed << now.seconds() << " /scan subscribed"
              << " angle_min = " << angle_min << " angle_max = " << angle_max
              << " angle_increment = " << angle_increment << std::endl;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();
    std::cout << std::fixed << now.seconds() << " cloud subscribed point cloud size:" << msg->data.size()
              << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    now = ros_clock_->now();
    std::cout << std::fixed << now.seconds() << " convert cloud size:" << cloud.points.size()
              << std::endl;
    for (int i = 0; i < (int)cloud.points.size(); i++) {
      std::cout << i << " x:" << cloud.points[i].x << " y:" << cloud.points[i].y
                << " z:" << cloud.points[i].z << std::endl;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    pub_cmd_vel_->publish(cmd_vel);
/*
    // transformしたい場合
    if (get_transform("base_link", "base_scan") == -1) return;
    pcl::PointCloud<pcl::PointXYZ> cloud_tf;
    pcl_ros::transformPointCloud(cloud, cloud_tf, trans_form_);

    std::string targetFrame = "base_link";
    pcl::PointCloud<pcl::PointXYZ> cloud_tf2;
    pcl_ros::transformPointCloud(targetFrame, cloud, cloud_tf2, *tf_buffer_);
    //フレームが見つからないときの対応が必要

    sensor_msgs::msg::PointCloud2 msg_tf;
    pcl::PointCloud<pcl::PointXYZ> cloud_tf3;
    pcl_ros::transformPointCloud(targetFrame, *msg, msg_tf, *tf_buffer_);
    //フレームが見つからないときの対応が必要
    pcl::fromROSMsg(msg_tf, cloud_tf3);

    for (int i = 0; i < (int)cloud.points.size(); i++) {
      std::cout << i << " x:" << cloud.points[i].x << " y:" << cloud.points[i].y
                << " z:" << cloud.points[i].z << std::endl;
      std::cout << i << " x:" << cloud_tf.points[i].x << " y:" << cloud_tf.points[i].y
                << " z:" << cloud_tf.points[i].z << std::endl;
      std::cout << i << " x:" << cloud_tf2.points[i].x << " y:" << cloud_tf2.points[i].y
                << " z:" << cloud_tf2.points[i].z << std::endl;
      std::cout << i << " x:" << cloud_tf3.points[i].x << " y:" << cloud_tf3.points[i].y
                << " z:" << cloud_tf3.points[i].z << std::endl;
    }
*/
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();

    std::cout << std::fixed << now.seconds() << " /cmd_vel subscribed"
              << " linear.x = " << msg->linear.x << " angular.z = " << msg->angular.z << std::endl;
  }

  // msgを使用しない場合は、warning防止の為msgをコメント化する
  void clicked_point_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr /*msg*/) {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();
    std::cout << std::fixed << now.seconds() << " /clicked_point subscribed" << std::endl;

    if (marker_array_.markers.empty()) return;  // makerArrayが空なら何もしない
    // marker_array_の最後のmarkerを非表示に設定
    marker_array_.markers.back().action = visualization_msgs::msg::Marker::DELETE;

    pub_marker_array_->publish(marker_array_);
    // marker_array_の最後のmarkerを削除（これをやってパブリッシュしても既に表示されているmarkerは消えない）
    marker_array_.markers.pop_back();
  }

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/map";
    marker.scale.x = 0.5;                      //矢印の長さ[m]
    marker.scale.y = 0.2;                      //矢印の幅[m]
    marker.scale.z = 0.2;                      //矢印の高さ[m]
    marker.color.a = 1.0;                      //矢印の透明度（0〜1、0：透明）
    marker.color.r = 0.0;                      //矢印の色、R要素、0〜1
    marker.color.g = 1.0;                      //矢印の色、G要素、0〜1
    marker.color.b = 0.0;                      //矢印の色、B要素、0〜1
    marker.id = marker_array_.markers.size();  // id
    marker.pose = msg->pose;                   // position,orientation
    marker_array_.markers.push_back(marker);
    pub_marker_array_->publish(marker_array_);

    std::cout << std::fixed << now.seconds() << " /goal_pose subscribed"
              << " pose.position.x = " << msg->pose.position.x
              << " pose.orientation.w = " << msg->pose.orientation.w << std::endl;
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    // double z = msg->pose.pose.position.z;
    double roll, pitch, yaw;
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    std::cout << std::fixed << now.seconds() << " /odom subscribed"
              << " x = " << x << " y = " << y << " yaw = " << yaw << std::endl;
  }

  void sent_messages_callback(const can_msgs::msg::Frame::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    bool flg_push_back = true;
    for (auto it = can_msg_array_.begin(); it != can_msg_array_.end(); ++it) {
      if (it->id == msg->id) {
        it->data = msg->data;
        flg_push_back = false;
      }
    }
    if (flg_push_back == true) can_msg_array_.push_back(*msg);
  }

  void received_messages_callback(const can_msgs::msg::Frame::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    bool flg_push_back = true;
    for (auto it = can_msg_array_.begin(); it != can_msg_array_.end(); ++it) {
      if (it->id == msg->id) {
        it->data = msg->data;
        flg_push_back = false;
      }
    }
    if (flg_push_back == true) can_msg_array_.push_back(*msg);
  }

  void timer_200ms_callback() {
    std::cout << "\033[2J"
              << "\033[1;1H";  //画面クリア
    for (auto it = can_msg_array_.begin(); it != can_msg_array_.end(); ++it) {
      std::cout << std::hex << std::setfill('0') << std::right << std::setw(3)
                << it->id;  // 16進数で表示
      for (auto i = 0; i < 8; i++)
        std::cout << " " << std::setfill('0') << std::right << std::setw(2) << (int)it->data[i];
      std::cout << std::dec << std::endl;  // 10進数表示に戻す
    }
  }

  void timer_50ms_callback() {
    std::lock_guard<std::mutex> lock(mtx_);
    rclcpp::Time now = ros_clock_->now();
    of_log_ << std::fixed << now.seconds() << " 50ms callback" << std::endl;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNode>());
  rclcpp::shutdown();
  return 0;
}
