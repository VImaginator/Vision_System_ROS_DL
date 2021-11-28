// Train the neural network tracker.

#include <string>
#include <iostream>

#include <caffe/caffe.hpp>

#include "example_generator.h"
#include "helper/helper.h"
#include "loader/loader_imagenet_det.h"
#include "loader/loader_alov.h"
#include "network/regressor_train.h"
#include "train/tracker_trainer.h"
#include "tracker/tracker_manager.h"
#include "loader/video.h"
#include "loader/video_loader.h"

using std::string;

// Desired number of training batches.
const int kNumBatches = 500000;

namespace {

// Train on a random image.
void train_image(const LoaderImagenetDet& image_loader,
           const std::vector<std::vector<Annotation> >& images,
           TrackerTrainer* tracker_trainer) {
  // Get a random image.
  const int image_num = rand() % images.size();
  const std::vector<Annotation>& annotations = images[image_num];

  // Choose a random annotation.
  const int annotation_num = rand() % annotations.size();

  // Load the image with its ground-truth bounding box.
  cv::Mat image;
  BoundingBox bbox;
  image_loader.LoadAnnotation(image_num, annotation_num, &image, &bbox);

  // Train on this example
  tracker_trainer->Train(image, image, bbox, bbox);
}

// Train on all annotated frames in the set of videos.
void train_video(const std::vector<Video>& videos, TrackerTrainer* tracker_trainer) {
  // Get a random video.
  const int video_num = rand() % videos.size();
  const Video& video = videos[video_num];

  // Get the video's annotations.
  const std::vector<Frame>& annotations = video.annotations;

  // We need at least 2 annotations in this video for this to be useful.
  if (annotations.size() < 2) {
    printf("Error - video %s has only %zu annotations\n", video.path.c_str(),
           annotations.size());
    return;
  }

  // Choose a random annotation.
  const int annotation_index = rand() % (annotations.size() - 1);

  // Load the frame's annotation.
  int frame_num_prev;
  cv::Mat image_prev;
  BoundingBox bbox_prev;
  video.LoadAnnotation(annotation_index, &frame_num_prev, &image_prev, &bbox_prev);

  // Load the next frame's annotation.
  int frame_num_curr;
  cv::Mat image_curr;
  BoundingBox bbox_curr;
  video.LoadAnnotation(annotation_index + 1, &frame_num_curr, &image_curr, &bbox_curr);

  // Train on this example
  tracker_trainer->Train(image_prev, image_curr, bbox_prev, bbox_curr);

  // Save
  frame_num_prev = frame_num_curr;
  image_prev = image_curr;
  bbox_prev = bbox_curr;
}

} // namespace

int main (int argc, char *argv[]) {
  if (argc < 14) {
    std::cerr << "Usage: " << argv[0]
              << " videos_folder_imagenet annotations_folder_imagenet"
              << " alov_videos_folder alov_annotations_folder"
              << " network.caffemodel train.prototxt val.prototxt"
              << " solver_file"
              << " lambda_shift lambda_scale min_scale max_scale"
              << " gpu_id"
              << std::endl;
    return 1;
  }

  FLAGS_alsologtostderr = 1;

  ::google::InitGoogleLogging(argv[0]);

  int arg_index = 1;
  const string& videos_folder_i