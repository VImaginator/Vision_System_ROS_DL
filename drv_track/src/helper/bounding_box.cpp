#include "bounding_box.h"

#include <cstdio>
#include "helper/image_proc.h"

// Uncomment line below if you want to use rectangles
#define VOT_RECTANGLE
#include "native/vot.h"

#include "helper.h"

// How much context to pad the image and target with (relative to the
// bounding box size).
const double kContextFactor = 2;

// Factor by which to scale the bounding box coordinates, based on the
// neural network default output range.
const double kScaleFactor = 10;

// If true, the neural network will estimate the bounding box corners: (x1, y1, x2, y2)
// If false, the neural network will estimate the bounding box center location and size: (center_x, center_y, width, height)
const bool use_coordinates_output = true;

BoundingBox::BoundingBox() :
  scale_factor_(kScaleFactor)
{
}

BoundingBox::BoundingBox(const VOTRegion& region)
{
  // VOTRegion is given by left, top, width, and height.
  x1_ = region.get_x();
  y1_ = region.get_y();
  x2_ = region.get_x() + region.get_width();
  y2_ = region.get_y() + region.get_height();
}

void BoundingBox::GetRegion(VOTRegion* region) {
  // VOTRegion is given by left, top, width, and height.
  region->set_x(x1_);
  region->set_y(y1_);
  region->set_width(get_width());
  region->set_height(get_height());
}


BoundingBox::BoundingBox(const std::vector<float>& bounding_box)
  : scale_factor_(kScaleFactor)
{
  if (bounding_box.size() != 4) {
    printf("Error - bounding box vector has %zu elements\n",
           bounding_box.size());
  }

  if (use_coordinates_output) {
    // Set bounding box coordinates.
    x1_ = bounding_box[0];
    y1_ = bounding_box[1];
    x2_ = bounding_box[2];
    y2_ = bounding_box[3];
  } else {
    // Get bounding box in format: (center_x, center_y, width, height)
    const double center_x = bounding_box[0];
    const double center_y = bounding_box[1];
    const double width = bounding_box[2];
    const double height = bounding_box[3];

    // Convert (center_x, center_y, width, height) to (x1, y1, x2, y2).
    x1_ = center_x - width / 2;
    y1_ = center_y - height / 2;
    x2_ = center_x + width / 2;
    y2_ = center_y + height / 2;
  }
}

void BoundingBox::GetVector(std::vector<float>* bounding_box) const {
  if (use_coordinates_output) {
    // Convert bounding box into a vector format using (x1, y1, x2, y2).
    bounding_box->push_back(x1_);
    bounding_box->push_back(y1_);
    bounding_box->push_back(x2_);
    bounding_box->push_back(y2_);
  } else {
    // Convert bounding box into a vector format using (center_x, center_y, width, height).
    bounding_box->push_back(get_center_x());
    bounding_box->push_back(get_center_y());
    bounding_box->push_back(get_width());
    bounding_box->push_back(get_height());
  }
}

void BoundingBox::Print() const {
  printf("Bounding box: x,y: %lf, %lf, %lf, %lf, w,h: %lf, %lf\n", x1_, y1_, x2_, y2_, get_width(), get_height());
}

void BoundingBox::Scale(const cv::Mat& image, BoundingBox* bbox_scaled) const {
  *bbox_scaled = *this;

  const int width = image.cols;
  const int height = image.rows;

  // Scale the bounding box so that the coordinates range from 0 to 1.
  bbox_scaled->x1_ /= width;
  bbox_scaled->y1_ /= height;
  bbox_scaled->x2_ /= width;
  bbox_scaled->y2_ /= height;

  // Scale the bounding box so that the coordinates range from 0 to scale_factor_.
  bbox_scaled->x1_ *= scale_factor_;
  bbox_scaled->x2_ *= scale_factor_;
  bbox_scaled->y1_ *= scale_factor_;
  bbox_scaled->y2_ *= scale_factor_;
}

void BoundingBox::Unscale(const cv::Mat& image, BoundingBox* bbox_unscaled) const {
  *bbox_unscaled = *this;

  const int image_width = image.cols;
  const int image_height = image.rows;

  // Unscale the bounding box so that the coordinates range from 0 to 1.
  bbox_unscaled->x1_ /= scale_factor_;
  bbox_unscaled->x2_ /= scale_factor_;
  bbox_unscaled->y1_ /= scale_factor_;
  bbox_unscaled->y2_ /= scale_factor_;

  // Unscale the bounding box so that the coordinates match the original image coordinates
  // (undoing the effect from the Scale 