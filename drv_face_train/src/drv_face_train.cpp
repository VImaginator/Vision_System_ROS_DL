#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <drv_msgs/face_train.h>

#include "facedetector.h"

#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <openssl/sha.h>
#include <rosauth/Authentication.h>
#include <sstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;
using namespace image_transport;

char* drv_path_env = std::getenv("DRV");

string drv_path_ = string(drv_path_env) + "/supplements/face_recognize/";

string image_path_ = drv_path_ + "images/";

ros::Publisher faceTrainPubStatus_;

enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;
string param_running_mode = "/status/running_mode";

string param_face_need_train_ = "/vision/face/need_train";
string param_face_train_name_ = "/vision/face/train/name";

string faceName_ = "";
Mat faceROI_;

int image_num_ = 100;
int imageCount_ = 0;

bool nameAdded_ = false;
int currNameId_ = 0;

vector<string> nameVec_;

cv_bridge::CvImagePtr imagePtr_;

FaceDetector fd_(drv_path_);

// Authority
ros::ServiceClient cl_auth_;
bool need_authority_ = false;
bool authenticated_ = false;
string param_password_ = "/password";
string password_ = ""; // default: admin


void resetStatus()
{
  faceName_ = "";
  imageCount_ = 0;
  
  nameAdded_ = false;
  currNameId_ = 0;
  
  nameVec_.clear();
  
  authenticated_ = false;
}

bool checkAuthority()
{
  string rand = "xyzabc";
  ros::Time now = ros::Time::now();
  string user_level = "admin";
  ros::Time end = ros::Time::now();
  end.sec += 120;
  
  // create the string to hash
  stringstream ss;
  ss << password_ << rand << now.sec << user_level << end.sec;
  string local_hash = ss.str();
  unsigned char sha512_hash[SHA512_DIGEST_LENGTH];
  SHA512((unsigned char *)local_hash.c_str(), local_hash.length(), sha512_hash);
  
  // convert to a hex string to compare
  char hex[SHA512_DIGEST_LENGTH * 2];
  // make the request
  rosauth::Authentication srv;
  srv.request.mac = string