#include "getsourcecloud.h"

GetSourceCloud::GetSourceCloud()
{
}

template<class T>
inline bool uIsFinite(const T & value)
{
#if _MSC_VER
  return _finite(value) != 0;
#else
  return std::isfinite(value);
#endif
}


float getDepth(
    const cv::Mat & depthImage,
    float x, float y,
    bool smoothing,
    float maxZError,
    bool estWithNeighborsIfNull)
{

  int u = int(x+0.5f);
  int v = int(y+0.5f);
  if(u == depthImage.cols && x<float(depthImage.cols))
  {
    u = depthImage.cols - 1;
  }
  if(v == depthImage.rows && y<float(depthImage.rows))
  {
    v = depthImage.rows - 1;
  }

  if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
  {
    return 0;
  }

  bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

  // Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
  // https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
  // Window weights:
  //  | 1 | 2 | 1 |
  //  | 2 | 4 | 2 |
  //  | 1 | 2 | 1 |
  int u_start = std::max(u-1, 0);
  int v_start = s