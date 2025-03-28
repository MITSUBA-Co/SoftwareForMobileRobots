#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fisheye.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

using sensor_msgs::msg::Image;
using namespace std;
using namespace cv;
using namespace std::chrono_literals;

class fisheye_projection : public rclcpp::Node {
 public:
  fisheye_projection(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~fisheye_projection() {}

 private:
  void onImageSubscribed(Image::SharedPtr img);
  rclcpp::Subscription<Image>::SharedPtr sub_img;
  void onTwistSubscribed(const geometry_msgs::msg::Twist::ConstSharedPtr twist);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
  std::string image_topic_name;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double pitch = 0.0;
  double yaw = 0;
  double zoom = 240.0;       
  double zoom_w = 0.0;       
  double zoom_h = 0.0;       
  double zoom_foot = 250.0;  
  rclcpp::Publisher<Image>::SharedPtr panorama_img;
  void onStringSubscribed(const std_msgs::msg::String::ConstSharedPtr string);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_string;
  bool panorama_flag = true;
};

fisheye_projection::fisheye_projection(rclcpp::NodeOptions options)
    : Node("fisheye_projection", options) {
  image_topic_name =
      this->declare_parameter<std::string>("image_topic_name", "/image_raw");
  sub_img = this->create_subscription<Image>(
      image_topic_name, 1,
      std::bind(&fisheye_projection::onImageSubscribed, this, std::placeholders::_1));
  sub_twist = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_camera", 1,
      std::bind(&fisheye_projection::onTwistSubscribed, this, std::placeholders::_1));
  panorama_img = this->create_publisher<Image>("/panorama", 1);           
  sub_string = this->create_subscription<std_msgs::msg::String>(
      "/panorama_flag", 1,
      std::bind(&fisheye_projection::onStringSubscribed, this, std::placeholders::_1));
}

void fisheye_projection::onStringSubscribed(const std_msgs::msg::String::ConstSharedPtr string) {
  if (panorama_flag) {
    panorama_flag = false;
  } else {
    panorama_flag = true;
  }
  cout << " panorama_flag:" << panorama_flag << endl;
  auto trash = string;
}

void fisheye_projection::onTwistSubscribed(const geometry_msgs::msg::Twist::ConstSharedPtr twist) {
  //変化量をπ/16の一定量に変更
  if (twist->linear.x > 0)
    yaw -= M_PI / 32;
  else if (twist->linear.x < 0)
    yaw += M_PI / 32;

  if (twist->angular.z > 0 && twist->linear.x >= 0)
    pitch += M_PI / 16;
  else if (twist->angular.z > 0 && twist->linear.x < 0)
    pitch -= M_PI / 16;
  else if (twist->angular.z < 0 && twist->linear.x >= 0)
    pitch -= M_PI / 16;
  else if (twist->angular.z < 0 && twist->linear.x < 0)
    pitch += M_PI / 16;

  if (yaw < -M_PI_4)
    yaw = -M_PI_4;
  else if (yaw > M_PI_4)
    yaw = +M_PI_4;
  if (pitch < -M_PI_4)
    pitch = -M_PI_4;
  else if (pitch > M_PI_4)
    pitch = +M_PI_4;

  if (twist->linear.z > 0) {
    zoom_w += 9;
    zoom_h += 6;
  } else if (twist->linear.z < 0) {
    zoom_w -= 9;
    zoom_h -= 6;
  }

  if (twist->linear.y > 0)
    zoom_foot *= 1.5;
  else if (twist->linear.y < 0)
    zoom_foot /= 1.5;

  if (twist->linear.x == 0 && twist->linear.y == 0 && twist->linear.z == 0 &&
      twist->angular.x == 0 && twist->angular.y == 0 && twist->angular.z == 0) {
    pitch = 0;
    yaw = 0;
    zoom_w = 0;
    zoom_h = 0;
    zoom_foot = 250;
  }

  cout << " pitch:" << pitch << " yaw:" << yaw << endl;
}

void fisheye_projection::onImageSubscribed(Image::SharedPtr msg) {
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  rclcpp::Time timeNow = ros_clock.now();

  auto img = cv_bridge::toCvShare(msg, msg->encoding);
  Mat front_im = img->image;  
  Mat rear_im = img->image;   
  cv::cvtColor(front_im, front_im, cv::COLOR_RGB2BGR);
  resize(front_im, front_im, cv::Size(), 640.0 / front_im.cols, 480.0 / front_im.rows);

    line(front_im, Point(600, 370), Point(440, 296), Scalar(0, 255, 255), 4, LINE_AA);
  line(front_im, Point(600, 150), Point(440, 201), Scalar(0, 255, 255), 4, LINE_AA);


  Mat p_im(480, 640, CV_8UC3);
  Mat f_im(640, 480, CV_8UC3);               
  Mat rotate_p_im(640, 480, CV_8UC3);        
  Mat p_im_left(480, 640, CV_8UC3);          
  Mat p_im_right(480, 640, CV_8UC3);         
  Mat rotate_p_im_left(640, 480, CV_8UC3);   
  Mat rotate_p_im_right(640, 480, CV_8UC3);  
  Mat temp;                                  
  Mat p_im_hconcat;                          
  Mat rotate_rear_im;                        

  double f_prm[8], p_prm[11];
  bool nn = false;  

  /* k1, k2, k3, k4, fx, fy, cx, cy */
  f_prm[0] = -0.0425717519946;                         // k1
  f_prm[1] = -0.00123937172291;                        // k2
  f_prm[2] = -0.00247584211717;                        // k3
  f_prm[3] = 0.0003834904042;                          // k4
  f_prm[4] = 919.420420196942 * ((double)640 / 2592);  // fx
  f_prm[5] = 919.580616965347 * ((double)640 / 2592);  // fy
  f_prm[6] = 1289.3182936353 * ((double)640 / 2592);   // cx
  f_prm[7] = 1035.0917201625 * ((double)480 / 1944);   // cy

  /* X0, Y0, Z0, e1X, e1Y, e1Z, e2X, e2Y, e2Z, i0, j0 */
  p_prm[0] = zoom * cos(pitch) * sin(yaw);  // X0
  p_prm[1] = zoom * sin(pitch);             // Y0
  p_prm[2] = zoom * cos(pitch) * cos(yaw);  // Z0
  p_prm[3] = cos(yaw);                      // e1X
  p_prm[4] = 0.0;                           // e1Y
  p_prm[5] = -sin(yaw);                     // e1Z
  p_prm[6] = -sin(pitch) * sin(yaw);        // e2X
  p_prm[7] = cos(pitch);                    // e2Y
  p_prm[8] = -sin(pitch) * cos(yaw);        // e2Z
  p_prm[9] = 639.0 / 2.0;                   // i0
  p_prm[10] = 479.0 / 2.0;                  // j0

  Mat publish_im(480, 640, CV_8UC3);

  //シリンダー変換
  double c_prm[10];
  int cy_height;
  int cy_width;
  if (panorama_flag) {
    cy_height = 480;
    cy_width = 640;
  } else {
    cy_height = 120;
    cy_width = 160;
  }
  Mat cylinder_im(cy_height, cy_width, CV_8UC3);
  c_prm[0] = 1.0;  // e1X
  c_prm[1] = 0.0;  // e1Y
  c_prm[2] = 0.0;  // e1Z
  c_prm[3] = 0.0;  // e2X
  c_prm[4] = 0.0;  // e2Y
  c_prm[5] = 1.0;  // e2Z
  c_prm[6] = (-60.0 - zoom_w) * M_PI / 180.0 - pitch;  // as
  c_prm[7] = (70.0 + zoom_w) * M_PI / 180.0 - pitch;   // ae
  c_prm[8] = (-30.0 - zoom_h) * M_PI / 180.0 + yaw;  // bs
  c_prm[9] = (50.0 + zoom_h) * M_PI / 180.0 + yaw;   // be
  fisheye_proj_cylinder(front_im, cylinder_im, f_prm, c_prm);

  //足元のみの変換画像
  double p_prm_foot[11];
  double yaw_foot = (M_PI / 2);
  double pitch_foot = 0;
  memcpy(p_prm_foot, p_prm, sizeof(p_prm));
  int foot_height;
  int foot_width;
  if (panorama_flag) {
    foot_height = 120;
    foot_width = 160;
  } else {
    foot_height = 480;
    foot_width = 640;
  }
  Mat p_im_foot(foot_width, foot_height, CV_8UC3);
  Mat rotate_p_im_foot(foot_height, foot_width, CV_8UC3);
  if (panorama_flag) {
    p_prm_foot[0] = zoom_foot / 4;  // X0
    p_prm_foot[1] = 160;            // X0
    p_prm_foot[2] = -240;           // Z0
  } else {
    p_prm_foot[0] = zoom_foot * cos(pitch_foot) * sin(yaw_foot);  // X0
    p_prm_foot[1] = zoom_foot * sin(pitch_foot) - 70;             // X0
    p_prm_foot[2] = zoom_foot * cos(pitch_foot) * cos(yaw_foot);  // Z0
  }
  p_prm_foot[3] = cos(yaw_foot);                     // e1X
  p_prm_foot[5] = -sin(yaw_foot);                    // e1Z
  p_prm_foot[6] = -sin(pitch_foot) * sin(yaw_foot);  // e2X
  p_prm_foot[7] = cos(pitch_foot);                   // e2Y
  p_prm_foot[8] = -sin(pitch_foot) * cos(yaw_foot);  // e2Z
  fisheye_proj_plane(front_im, p_im_foot, f_prm, p_prm_foot, nn);
  rotate(p_im_foot, rotate_p_im_foot, cv::ROTATE_90_CLOCKWISE);


  if (panorama_flag) {
    line(rotate_p_im_foot, Point(159, 119), Point(0, 119), Scalar(255, 255, 255), 2, LINE_4);
    line(rotate_p_im_foot, Point(0, 119), Point(0, 0), Scalar(255, 255, 255), 2, LINE_4);
    cv::Mat mat = (cv::Mat_<double>(2, 3) << 1.0, 0.0, 481, 0.0, 1.0, 0);
    warpAffine(rotate_p_im_foot, cylinder_im, mat, cylinder_im.size(), CV_INTER_LINEAR,
               cv::BORDER_TRANSPARENT);
    publish_im = cylinder_im;
  } else {
    line(cylinder_im, Point(159, 119), Point(0, 119), Scalar(255, 255, 255), 2, LINE_4);
    line(cylinder_im, Point(0, 119), Point(0, 0), Scalar(255, 255, 255), 2, LINE_4);
    cv::Mat mat = (cv::Mat_<double>(2, 3) << 1.0, 0.0, 481, 0.0, 1.0, 0);
    warpAffine(cylinder_im, rotate_p_im_foot, mat, rotate_p_im_foot.size(), CV_INTER_LINEAR,
               cv::BORDER_TRANSPARENT);
    publish_im = rotate_p_im_foot;
  }

  cv::waitKey(1);

  cout << "proc time:" << fixed << (ros_clock.now() - timeNow).seconds() << endl;

  //画像送信
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::msg::Image::SharedPtr msg_ =
      cv_bridge::CvImage(msg->header, "bgr8", publish_im).toImageMsg();
  panorama_img->publish(*msg_.get());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fisheye_projection>());
  rclcpp::shutdown();
  return 0;
}
