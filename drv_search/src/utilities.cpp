#include "utilities.h"

using namespace cv;

Utilities::Utilities()
{
  ros::NodeHandle rgb_nh(nh_, "rgb");
  ros::NodeHandle depth_nh(nh_, "depth");
  ros::NodeHandle rgb_pnh(pnh_, "rgb");
  ros::NodeHandle depth_pnh(pnh_, "depth");
  image_transport::ImageTransport rgb_it(rgb_nh);
  image_transport::ImageTransport depth_it(depth_nh);
  // !Use compressed message to speed up -- necessary!
  image_transport::TransportHints hintsRgb("compressed", ros::TransportHints(), rgb_pnh);
  image_transport::TransportHints hintsDepth("compressedDepth", ros::TransportHints(), depth_pnh);

  subImage_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
  subDepth_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
  subCameraInfo_.subscribe(rgb_nh, "camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
  message_