#include <tinyxml.h>

#include "train/example_generator.h"
#include "loader/loader_imagenet_det.h"
#include "helper/helper.h"

using std::vector;
using std::string;

namespace bfs = boost::filesystem;

// If true, only load a small number of images.
const bool kDoTest = false;

// Max ratio of bbox size to image size that we load.
// If the ratio is too large (i.e. the object occupies almost the entire image),
// then we will not be able to simulate object motion.
const double kMaxRatio = 0.66;

LoaderImagenetDet::LoaderImagenetDet(const std::string& image_folder,
                                     const std::string& annotations_folder)
  : path_(image_folder)
{
  if (!bfs::is_directory(annotations_folder)) {
    printf("Error - %s is not a valid directory!\n", annotations_folder.c_str());
    return;
  }

  // Find all image subfolders.
  vector<string> subfolders;
  find_subfolders(annotations_folder, &subfolders);

  size_t num_annotations = 0;

  const int max_subfolders = kDoTest ? 1 : subfolders.size();

  printf("Found %zu subfolders...\n", subfolders.size());
  printf("Loading images, please wait...\n");

  // Iterate over all subfolders.
  for (size_t i = 0; i < max_subfolders; ++i) {
    // Every 100 iterations, print an update.
    if (i % 10 == 0 && i > 0) {
      printf("Loaded %zu subfolders\n", i);
    }
    const string& subfolder_name = subfolders[i];
    const string& subfolder_path = annotations_folder + "/" + subfolder_name;

    //printf("Loading subfolder: %s\n", subfolder_name.c_str());

    // Find the annotation files.
    const boost::regex annotation_filter(".*\\.xml");
    vector<string> annotation_files;
    find_matching_files(subfolder_path, annotation_filter, &annotation_files);

    //printf("Found %zu annotations\n", annotation_files.size());

    // Iterate over all annotation files.
    for (size_t j = 0; j < annotation_files.size(); ++j) {
      const string& annotation_file = annotation_files[j];

      const string& full_path = subfolder_path + "/" + annotation_file;

      // Read the annotations.
      //printf("Processing annotation file: %s\n", full_path.c_str());
      vector<Annotation> annotations;
      LoadAnnotationFile(full_path, &annotations);

      if (annotations.size() == 0) {
        continue;
      }

      // Count the number of annotations.
      //printf("Found %zu annotations\n", annotations.size());
      num_annotations += annotations.size();

      // Save the annotations.
      images_.push_back(annotations);
    } // Process all annotations in a subfolder.
  } // Process all subfolders.
  printf("Found %zu annotations from %zu images\n", num_annotations, images_.size());
}

void LoaderImagenetDet::LoadAnnotationFile(const string& annotation_file,
                                