#include <tf2/LinearMath/Quaternion.h>
#include "obstacledetect.h"


// Normal threshold, |z_norm_| > th_z_norm_ means the point is from plane
float th_z_norm_ = 0.7;

// Region growing threshold
float th_smooth_ = 8;

// Voxel grid threshold
float th_leaf_ = 0.015;
// th_deltaz_ must 2 times bigger than th_leaf_
float th_deltaz_ = 2 * th_leaf_;
float th_ratio_ = 5 * th_leaf_; // flatness ratio max value of plane

// Depth threshold
float th_max_depth_ = 1.3;

ObstacleDetect::ObstacleDetect(bool use_od, string base_frame, float base_to_ground, 
                               float table_height, float table_area) :
  use_od_(use_od),
  fi_(new FetchRGBD),
  pub_it_(nh_),
  src_cloud_(new PointCloudMono),
  src_z_inliers_(new pcl::PointIndices),
  m_tf_(new Transform),
  base_frame_(base_frame),
  base_link_above_ground_(base_to_ground),
  table_height_(table_height),
  th_height_(0.2),
  th_area_(table_area)
{
  param_running_mode_ = "/status/running_mode";
  
  // For store max hull id and area
  global_area_temp_ = 0;
  
  // Regist the callback if you want to use octomap
  // Obstacle info from point cloud
  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/vision/depth_registered/points", 1, 
                                                            &ObstacleDetect::cloudCallback, this);

  // Detect table obstacle
  pub_table_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/ctrl/vision/detect/table", 1);
  pub_table_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/table/points", 1);
  
  // Detect obstacle by octomap from point cloud
  pub_exp_obj_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/points/except_object", 1);
  
  // Detect obstacle by octomap from depth image
  pub_depth_cam_info_ = nh_.advertise<sensor_msgs::CameraInfo>("/vision/depth/camera_info", 3);
  pub_exp_obj_depth_ = pub_it_.advertise("/vision/depth/except_object", 1);
}

ObstacleDetect::ObstacleDetect(bool use_od, string base_frame, 
                               float base_to_ground, float table_height, float table_area,
                               float grasp_area_x, float grasp_area_y, float tolerance) :
  use_od_(use_od),
  fi_(new FetchRGBD),
  pub_it_(nh_),
  src_cloud_(new PointCloudMono),
  src_z_inliers_(new pcl::PointIndices), 
  m_tf_(new Transform),
  base_frame_(base_frame),
  base_link_above_ground_(base_to_ground),
  table_height_(table_height),
  th_height_(0.2),
  th_area_(table_area),
  grasp_area_x_(grasp_area_x),
  grasp_area_y_(grasp_area_y), 
  tolerance_(tolerance)
{
  param_running_mode_ = "/status/running_mode";
  
  // Store max hull id and area
  global_area_temp_ = 0;
  
  // Get table info from point cloud
  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/vision/depth_registered/points", 1, 
                                                            &ObstacleDetect::cloudCallback, this);
  
  pub_table_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/table/points", 1);
  pub_exp_obj_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/points_except_object", 1);
  pub_exp_obj_depth_ = pub_it_.advertise("/vision/depth/except_object", 1);
}

void ObstacleDetect::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!use_od_)
    return;
  
  if (ros::param::has(param_running_mode_)) {
    int mode_type;
    ros::param::get(param_running_mode_, mode_type);
    // 2 for tracking, 3 for putting
    if (mode_type == 2 || mode_type == 3) {
      if (msg->data.empty()) {
        ROS_WARN_THROTTLE(31, "ObstacleDetect: PointCloud is empty.");
        return;
      }
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*msg, pcl_pc2);
      PointCloudMono::Ptr temp(new PointCloudMono);
      pcl::fromPCLPointCloud2(pcl_pc2, *temp);
      
      PointCloudMono::Ptr temp_filtered(new PointCloudMono);
      Utilities::getCloudByZ(temp, src_z_inliers_, temp_filtered, 0.0, th_max_depth_);
      
      m_tf_->getTransform(base_frame_, msg->header.frame_id);
      m_tf_->doTransform(temp_filtered, src_cloud_);
    }
  }
}

bool ObstacleDetect::detectPutTable(geometry_msgs::PoseStamped &put_pose,
                                    geometry_msgs::PoseStamped &ref_pose, 
                                    bool &need_move)
{
  getSourceCloud();
  findMaxPlane();
  if (plane_max_hull_ == NULL) {
    ROS_INFO_THROTTLE(11, "ObstacleDetect: No put place detected.");
    return false;
  }
  else {
    need_move = analysePutPose(put_pose, ref_pose);
    return true;
  }
}

bool ObstacleDetect::getSourceCloud()
{
  while (ros::ok()) {
    if (!src_cloud_->po