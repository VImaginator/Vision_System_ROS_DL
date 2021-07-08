#ifndef OBSTACLEDETECT_H
#define OBSTACLEDETECT_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

#include <math.h>
#include <vector>
#include <string>

#include "fetchrgbd.h"
#include "transform.h"
#include "utilities.h"

using namespace std;
using namespace cv;

class ObstacleDetect
{
public:
  /**
   * @brief ObstacleDetect
   * Used in grasp function
   * @param use_od
   * @param base_frame
   * @param base_to_ground
   * @param table_height
   * @param table_area
   */
  ObstacleDetect(bool use_od, string base_frame, float base_to_ground, 
                 float table_height, float table_area);
  
  /**
   * @brief ObstacleDetect
   * Used in put function, regist callback for pointcloud
   * @param use_od
   * @param base_frame
   * @param base_to_ground
   * @param table_height
   * @param table_area
   * @param grasp_area_x X value of grapable area center
   * @param grasp_area_y Y value of grapable area center
   * @param tolerance How far robot hand can be away from graspable center in x direction
   */
  ObstacleDetect(bool use_od, string base_frame, float base_to_ground, float table_height,
                 float table_area, float grasp_area_x, float grasp_area_y, float tolerance);
  /**
   * @brief ObstacleDetect::detectTableInCloud
   * Find out whether the source cloud contains table,
   * if true, publish table geometry for obstacle avoiding.
   * The msg type is geometry_msgs/PoseStamped, which
   * contains pose of the table centroid, assuming the table is a box,
   * the size of table can be pre-defined
   */
  void detectObstacleTable();
  
  /**
   * @brief detectObstacle
   * With the target bounding box given, publish pointcloud without
   * the object
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  void detectObstacleInCloud(int min_x, int min_y, int max_x, int max_y);
  
  void detectObstacleInDepth(int min_x, int min_y, int max_x, int max_y);
  
  /**
   * @brief detectPutTable
   * @param put_pose
   * @param ref_pose
   * @param need_move Whether need to move the robot to put down object
   * @return True if current scene contains table
   */
  bool detectPutTable(geometry_msgs::PoseStamped &put_pose, 
                      geometry_msgs::PoseStamped &ref_pose, 
                      bool &need_move);
  
  inline void setZOffset(float z_offset) {z_offset_ = z_offset;}
  
  // Whether perform obstacle detection
  bool use_od_;
  inline void setUseOD(bool use) {use_od_ = use;}
  inline bool getUseOD() {return use_od_;}
  
private:
  string param_running_mode_;
  
  ros::NodeHandle nh_;
  image_transport::ImageTransport pub_it_;
  
  // Source point cloud and its inliers after z filter
  PointCloudMono::Ptr src_cloud_;
  pcl::PointIndices::Ptr src_z_inliers_;
  ros::Subscriber sub_pointcloud_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  
  FetchRGBD *fi_;
  // The transform object can't be shared between Classes
  Transform *m_tf_;
  // Frame for point cloud to transfer
  string base_frame_;
  
  ros::Publisher pub_table_pose_;
  ros::Publisher pub_table_points_;
  ros::Publisher pub_exp_obj_cloud_;
  
  ros::Publisher pub_depth_cam_info_;
  image_transport::Publisher pub_exp_obj_depth_;
  
  // Target table approximate height
  float table_height_;
  
  // height max error 
  float th_height_;
  // Table area threshold
  float th_area_;
  
  // Distance between base_frame and ground
  fl