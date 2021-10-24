#ifndef VIDEO_H
#define VIDEO_H

#include "helper/bounding_box.h"

// An image frame and corresponding annotation.
struct Frame {
  int frame_num;
  BoundingBox bbox;
};

// Container for video data and the corresponding frame annotations.
class Video {
public:
  // For a given annotation index, get the corresponding frame number, image,
  // and bounding box.
  void LoadAnnotation(const int annotation_index, int* frame_num, cv::Mat* image,
                     BoundingBox* box) const;

  // Find and return the first frame with an annotation in this video.
  void LoadFirstAnnotation(int