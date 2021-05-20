#include "makeplan.h"

MakePlan::MakePlan()
{
}

void MakePlan::removeOutliers(PointCloud::Ptr cloud_in, PointCloud::Ptr &cloud_out)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud_in);
  outrem.setRadiusSearch(0.01);
  outrem.setMinNeighborsInRadius (4);
  // apply filter
  outrem.filter (*cloud_out);
}

bool MakePlan::getAveragePoint(PointCloud::Ptr cloud_in, pcl::PointXYZ &avrPt)
{
  if (!cloud_in->points.size())
  {
    return false;
  }
  
  pcl::PointXYZ min_dis_point;
  float min_dis = 100.