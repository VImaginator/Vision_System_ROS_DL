
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

// Custom message
#include <drv_msgs/recognized_target.h>

// STL
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"
#include "utilities.h"

using namespace std;
using namespace cv;

const int angle_step = 1;
const float x_to_angle = 0.02; // a reasonable speed
const float y_to_angle = 0.02;

// Wait 50 loops befor target lost
#define WAIT_LOOP 50
int delay_ = WAIT_LOOP;

// Publishers
image_transport::Publisher trackPubImage_;
ros::Publisher trackPubStatus_;
ros::Publisher trackPubServo_;
ros::Publisher trackPubTargetLocation_;

// Image temps
cv_bridge::CvImageConstPtr src_;
Mat src_img_;

cv_bridge::CvImagePtr track_ptr_(new cv_bridge::CvImage());
Mat track_img_;

// Run mode
enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;
int modeTypeTemp_ = m_wander;
string param_running_mode = "/status/running_mode";

bool isInTracking_ = true;

// Target infomation
std_msgs::String tgt_label_;
Rect roi_init_;

// Global params that record servo angle, 
// only used for initialize pitch_ and yaw_
string param_servo_pitch = "/status/servo/pitch";
string param_servo_yaw = "/status/servo/yaw";
int pitch_ = 70;
int yaw_ = 90;

// Initialize the tracker
KCFTracker tracker; // use default settings

void publishServo(int pitch_angle, int yaw_angle)
{
  std_msgs::UInt16MultiArray array;
  array.data.push_back(pitch_angle);
  array.data.push_back(yaw_angle);
  array.data.push_back(20); // Servo speed
  pitch_ = pitch_angle;
  yaw_ = yaw_angle;
  trackPubServo_.publish(array);
}

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
  // This callback should always active
  pitch_ = msg->data[0];
  yaw_ = msg->data[1];
}

void searchROICallback(const drv_msgs::recognized_targetConstPtr &msg)
{
  tgt_label_ = msg->label;
  int min_x = msg->tgt_bbox_array.data[0];
  int min_y = msg->tgt_bbox_array.data[1];
  int max_x = msg->tgt_bbox_array.data[2];
  int max_y = msg->tgt_bbox_array.data[3];
  
  ROS_INFO("Track: Received ROI %d %d %d %d.", min_x, min_y, max_x, max_y);

  // Pad search roi by 10 px to make the tracking more robust
  Utilities::tryExpandROI(min_x, min_y, max_x, max_y, 10);
  roi_init_ = Rect(min_x, min_y, max_x - min_x, max_y - min_y);
  tracker.initialized_ = false;
}

bool verifyDetection(Rect roi) {
  /* If the roi is out of image region, cut it to fit the image */
  if (roi.x < 0) {
    roi.x = 0;
  }
  if (roi.x >= 640) {
    roi.x = 639;
  }
  if (roi.y < 0) {
    roi.y = 0;
  }
  if (roi.y >= 480) {
    roi.y = 479;
  }
  if (roi.x + roi.width >= 640) {
    roi.width = 639 - roi.x;
  }
  if (roi.y + roi.height >= 480) {
    roi.height = 479 - roi.y;
  }
  if (roi.area() < MIN_OBJECT_AREA || roi.area() > MAX_OBJECT_AREA) {
    ROS_WARN("Track: Target ROI is unnormal.");
    return false;
  }
  return true;
}

void pubTarget(std_msgs::Header header, vector<unsigned int> mask_id, Rect roi) {
  // Publish new target info
  drv_msgs::recognized_target result;
  result.header = header;
  result.label = tgt_label_;
  result.tgt_pixels.data = mask_id; // the datatype is uint 32 aka unsigned int
  result.tgt_bbox_array.data.push_back(roi.x);
  result.tgt_bbox_array.data.push_back(roi.y);
  result.tgt_bbox_array.data.push_back(roi.x + roi.width);
  result.tgt_bbox_array.data.push_back(roi.y + roi.height);
  result.tgt_bbox_center.data.push_back(roi.x + roi.width / 2);
  result.tgt_bbox_center.data.push_back(roi.y + roi.height / 2);

  trackPubTargetLocation_.publish(result);
}

bool postProcess(Rect roi, Mat &track_img)
{
  if (!verifyDetection(roi))
    return false;

  Utilities::markImage(roi, track_img);
  return true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
  if (modeType_ != m_track)
    return;

  if (!verifyDetection(roi_init_)) {
    ROS_WARN("Track: Initial ROI can not be tracked.");
    isInTracking_ = false;
    tracker.initialized_ = false;
    return;
  }

  src_ = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  src_img_ = src_->image;
  src_img_.copyTo(track_img_);

  // cv::rectangle(track_image_, detection_, cv::Scalar(232,228,53), 2);
  // Adjust the camera view based on ROI center
  if (!tracker.initialized_) {
    tracker.init(roi_init_, src_img_);
    isInTracking_ = true;
    tracker.initialized_ = true;
  }
  else {
    Rect roi = tracker.update(src_img_);

    vector<unsigned int> mask_id; // store object pixels id in image
    if (!postProcess(roi, track_img_)) {
      delay_--;
      if (delay_ < 0) {
        isInTracking_ = false;
        tracker.initialized_ = false;
      }
      return;
    }

    // Publish image of target with bounding box
    track_ptr_->header = image_msg->header;
    track_ptr_->image = track_img_;
    track_ptr_->encoding = sensor_msgs::image_encodings::BGR8;
    trackPubImage_.publish(track_ptr_->toImageMsg());

    // Move the camera so that the center of the image captured is on object center
    int d_x = roi.x + roi.width / 2 - 320;
    int d_y = roi.y + roi.height / 2 - 240;
    int deg_x = int(d_x * x_to_angle); // offset the robot head
    int deg_y = int(d_y * y_to_angle);

    isInTracking_ = true;
    delay_ = WAIT_LOOP;
    pubTarget(image_msg->header, mask_id, roi);

    if (abs(deg_x) < angle_step && abs(deg_y) < angle_step) {
      // Target on image center, continue tracking..
      isInTracking_ = true;
      delay_ = WAIT_LOOP;
      pubTarget(image_msg->header, mask_id, roi);
      return;
    }
    if (abs(deg_x) >= angle_step && abs(deg_y) < angle_step) {
      // Need move in x direction
      if (deg_x < -angle_step) deg_x = -angle_step;
      if (deg_x > angle_step) deg_x = angle_step;
      deg_y = 0;
    }
    if (abs(deg_x) < angle_step && abs(deg_y) >= angle_step) {
      // Need move camera in y direction
      if (deg_y < -angle_step) deg_y = -angle_step;
      if (deg_y > angle_step) deg_y = angle_step;
      deg_x = 0;
    }
    if (abs(deg_x) >= angle_step && abs(deg_y) >= angle_step) {
      // Need move in both directions
      if (deg_x < -angle_step) deg_x = -angle_step;
      if (deg_x > angle_step) deg_x = angle_step;
      if (deg_y < -angle_step) deg_y = -angle_step;
      if (deg_y > angle_step) deg_y = angle_step;
    }
    int x_ang = - deg_x + yaw_;
    int y_ang = - deg_y + pitch_;

    if (!(x_ang >= 0 && x_ang <= 180 && y_ang >= 60 && y_ang <= 140)) {
      // Target center out of camera movable region
      pubTarget(image_msg->header, mask_id, roi);
      ROS_WARN_THROTTLE(31, "Track: Target out of camera movable area.");
      // Although out of movable area, still in tracking
      // isInTracking_ = false;
      // tracker.initialized_ = false;
    }
    else {
      // Target center is in camera movable region, so move the camera
      publishServo(y_ang, x_ang);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_track_kcf");

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::NodeHandle rgb_nh(nh, "rgb");
  ros::NodeHandle rgb_pnh(pnh, "rgb");

  image_transport::ImageTransport it_rgb_sub(rgb_nh);
  image_transport::TransportHints hints_rgb("compressed", ros::TransportHints(), rgb_pnh);

  image_transport::ImageTransport it_rgb_pub(nh);
  trackPubImage_ = it_rgb_pub.advertise("search/labeled_image", 1);
  trackPubServo_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1);
  trackPubStatus_ = nh.advertise<std_msgs::Bool>("status/track/feedback", 1);
  trackPubTargetLocation_ = nh.advertise<drv_msgs::recognized_target>("track/recognized_target" , 1);

  ros::Subscriber sub_res = nh.subscribe<drv_msgs::recognized_target>("search/recognized_target", 1, searchROICallback);
  image_transport::Subscriber sub_rgb = it_rgb_sub.subscribe("image_rect_color", 1, imageCallback, hints_rgb);
  ros::Subscriber sub_s = nh.subscribe<std_msgs::UInt16MultiArray>("servo", 1, servoCallback);

  if (ros::param::has(param_servo_pitch))
    ros::param::get(param_servo_pitch, pitch_);

  if (ros::param::has(param_servo_yaw))
    ros::param::get(param_servo_yaw, yaw_);

  ROS_INFO("KCF tracking function initialized.");

  while (ros::ok())
  {
    if (ros::param::has(param_running_mode)) {
      ros::param::get(param_running_mode, modeTypeTemp_);

      if (modeTypeTemp_ != m_track && modeType_ == m_track)
        tracker.initialized_ = false;

      modeType_ = modeTypeTemp_;
    }

    std_msgs::Bool flag;
    flag.data = true;

    ros::spinOnce();

    flag.data = isInTracking_;
    trackPubStatus_.publish(flag);
  }

  return 0;
}