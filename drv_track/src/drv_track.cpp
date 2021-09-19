/*
 * Make sure this line is in your ~/.bashrc:
 *
 * export DRV=/home/USER/catkin_ws/src/drv_package
 *
 * Change 'USER' according to your environment
*/

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

//custom message
#include <drv_msgs/recognized_target.h>

//stl
#include <cstdlib>
#include <math.h>

#include "goturn.h"

const int angle_step = 1;
const float x_to_angle = 0.02; // a reasonable speed
const float y_to_angle = 0.02;

using namespace std;

// wait loop befor target lost
#define WAIT_LOOP 50
int delay_ = WAIT_LOOP;

image_transport::Publisher trackPubImage_;
ros::Publisher trackPubServo_;
ros::Publisher trackPubStatus_;
ros::Publisher trackPubTarget_; // notice the topic is different from the subscribed one

cv_bridge::CvImageConstPtr src_;
cv::Mat src_img_;

cv_bridge::CvImagePtr track_ptr_(new cv_bridge::CvImage());
cv::Mat track_img_;

enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;
int modeTypeTemp_ = m_wander;

string param_running_mode = "/status/running_mode";
bool isInTracking_ = true;// cause the first call for tracking means have something to track

// Initialize tracking function class
char* drv_path_env = std::getenv("DR