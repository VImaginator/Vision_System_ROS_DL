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
    if (!src_cloud_->points.empty())
      return true;
    
    // Handle callbacks and sleep for a small amount of time
    // before looping again
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
}

void ObstacleDetect::detectObstacleTable()
{
  getSourceCloud();
  findMaxPlane();
  
  // Get table geometry from hull and publish the plane with max area
  analyseObstacle();
}

void ObstacleDetect::detectObstacleInCloud(int min_x, int min_y, 
                                           int max_x, int max_y)
{
  if (src_cloud_->points.empty())
    return;
  
  assert(src_cloud_->points.size() == src_z_inliers_->indices.size());
  
  PointCloudMono::Ptr cloud_except_obj(new PointCloudMono);
  pcl::PointIndices::Ptr idx_obj(new pcl::PointIndices);
  
  for (size_t i = 0; i < src_z_inliers_->indices.size(); ++i) {
    int c = src_z_inliers_->indices[i] % 640;
    int r = src_z_inliers_->indices[i] / 640;
    if (c > min_x && c < max_x && r > min_y && r < max_y)
      idx_obj->indices.push_back(i);
  }
  // Set negtive=true to get cloud id not equal to idx_obj
  Utilities::getCloudByInliers(src_cloud_, cloud_except_obj, idx_obj, 
                               true, false);
  publishCloud(cloud_except_obj, pub_exp_obj_cloud_);
}

void ObstacleDetect::detectObstacleInDepth(int min_x, int min_y, 
                                           int max_x, int max_y)
{
  cv_bridge::CvImagePtr rgb, depth;
  sensor_msgs::CameraInfo info;
  fi_->fetchRGBD(rgb, depth, info);
  Mat depth_except_obj = depth->image;
  for (size_t r = 0; r < depth_except_obj.rows; ++r) {
    for (size_t c = 0; c < depth_except_obj.cols; ++c) {
      if (c > min_x && c < max_x && r > min_y && r < max_y) {
        // The type of depth image is CV_32F
        depth_except_obj.at<float>(r, c) = 0.0;
      }
    }
  }
  cv_bridge::CvImage cv_img;
  cv_img.image = depth_except_obj;
  cv_img.header = info.header;
  cv_img.encoding = depth->encoding;
  
  pub_depth_cam_info_.publish(info);
  pub_exp_obj_depth_.publish(cv_img.toImageMsg());
}

void ObstacleDetect::findMaxPlane()
{
  if (src_cloud_->points.empty())
    return;
  
  // Clear temp
  planeZVector_.clear();
  plane_coeff_.clear();
  plane_hull_.clear();
  global_area_temp_ = 0.0;
  global_height_temp_ = 0.0;
  
  // Calculate the normal of source cloud
  PointCloudRGBN::Ptr src_norm(new PointCloudRGBN);
  Utilities::estimateNormCurv(src_cloud_, src_norm, 2*th_leaf_, th_leaf_, true);
  
  // Extract all points whose norm indicates that the point belongs to plane
  pcl::PointIndices::Ptr idx_norm_ok(new pcl::PointIndices);
  Utilities::getCloudByNormZ(src_norm, idx_norm_ok, th_z_norm_);
  
  if (idx_norm_ok->indices.empty()) {
    ROS_DEBUG("ObstacleDetect: No point have the right normal of plane.");
    return;
  }
  
  PointCloudRGBN::Ptr cloud_norm_ok(new PointCloudRGBN);
  Utilities::getCloudByInliers(src_norm, cloud_norm_ok, idx_norm_ok, false, false);
  
  ROS_DEBUG("Points may from plane: %d", cloud_norm_ok->points.size());
  
  // Prepare curv data for clustering
  pcl::PointCloud<pcl::Normal>::Ptr norm_plane_curv(new pcl::PointCloud<pcl::Normal>);
  norm_plane_curv->resize(cloud_norm_ok->size());
  
  size_t i = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_norm_ok->begin();
       pit != cloud_norm_ok->end(); ++pit) {
    norm_plane_curv->points[i].normal_x = pit->normal_x;
    norm_plane_curv->points[i].normal_y = pit->normal_y;
    norm_plane_curv->points[i].normal_z = pit->normal_z;
    ++i;
  }
  // Perform clustering, cause the scene may contain multiple planes
  calRegionGrowing(cloud_norm_ok, norm_plane_curv);
  
  // Extract each plane from the points having similar z value,
  // the planes are stored in vector plane_hull_
  extractPlaneForEachZ(cloud_norm_ok);
}

template <typename PointTPtr>
void ObstacleDetect::publishCloud(PointTPtr cloud, ros::Publisher pub)
{
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.frame_id = base_frame_;
  ros_cloud.header.stamp = ros::Time(0)