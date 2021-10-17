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
                                           vector<Annotation>* image_annotations) {
  // Open the annotation file.
  TiXmlDocument document(annotation_file.c_str());
  document.LoadFile();

  // Read the top-level element.
  TiXmlHandle docHandle( &document );
  TiXmlElement* annotations = docHandle.FirstChild().ToElement();
  if (!annotations) {
    printf("No annotations!\n");
    return;
  }

  // Get the folder and filename for the image corresponding to this annotation.
  const string& folder = annotations->FirstChildElement("folder")->GetText();
  //printf("Folder: %s\n", folder);

  const string& filename = annotations->FirstChildElement("filename")->GetText();
  //printf("File: %s\n", filename);

  // Get the relative image size that was displayed to the annotater (may have been downsampled).
  TiXmlNode* size = annotations->FirstChild("size");
  if (!size) {
    printf("Error - no size!\n");
    return;
  }
  const int display_width = atoi(size->FirstChildElement("width")->GetText());
  const int display_height = atoi(size->FirstChildElement("height")->GetText());
  //printf("Size: %d %d\n", display_width, display_height);

  // Get all of the bounding boxes in this image.
  for(TiXmlNode* object = annotations->FirstChild("object"); object; object = object->NextSibling("object")) {
    // Get the boudning box coordinates.
    TiXmlElement* bbox = object->FirstChildElement("bndbox");
    const int xmin = atoi(bbox->FirstChildElement("xmin")->GetText());
    const int xmax = atoi(bbox->FirstChildElement("xmax")->GetText());
    const int ymin = atoi(bbox->FirstChildElement("ymin")->GetText());
    const int ymax = atoi(bbox->FirstChildElement("ymax")->GetText());

    const double width = xmax - xmin;
    const double height = ymax - ymin;

    // If this object occupies almost the entire image, then ignore it,
    // since we will not be able to simulate object motion.
    if (width > kMaxRatio * display_width || height > kMaxRatio * display_height) {
      continue;
    }

    // Convert the annotation to bounding box format.
    Annotation annotation;
    annotation.image_path = folder + "/" + filename;
    annotation.bbox.x1_ = xmin;
    annotation.bbox.x2_ = xmax;
    annotation.bbox.y1_ = ymin;
    annotation.bbox.y2_ = ymax;
    annotation.display_width_ = display_width;
    annotation.display_height_ = display_height;

    // Check if the annotation is outside of the border of the image or otherwise invalid.
    if (xmin < 0 || ymin < 0 || xmax <= xmin || ymax <= ymin) {
      printf("Skipping invalid annotation from file: %s\n", annotation_file.c_str());
      printf("Annotation: %d, %d, %d, %d\n", xmin, xmax, ymin, ymax);
      printf("Image path: %s\n", annotation.image_path.c_str());
      printf("Display: %d, %d\n", display_width, display_height);
      continue;
    }

    // Save the annotation.
    image_annotations->push_back(annotation);

    //printf("Path: %s\n", annotation.image_path.c_str());
    //printf("bbox: %d %d %d %d\n", xmin, xmax, ymin, ymax);
  }
}

void LoaderImagenetDet::ShowImages() const {
  // Iterate over all images.
  for (size_t image_index = 0; image_index < images_.size(); ++image_index) {
    // Load the image.
    cv::Mat image;
    LoadImage(image_index, &image);

    // Display the image and wait for a keystroke to continue.
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.
    cv::waitKey(0);                                          // Wait for a keystroke in the window
  }
}

void LoaderImagenetDet::ComputeStatistics() const {
  bool first_time = true;

  // Variables to store various image statistics.
  double min_width;
  double min_height;
  double max_width;

  double max_height;
  double mean_width;
  double mean_height;

  double min_width_frac;
  double min_height_frac;
  double max_width_frac;

  double max_height_frac;
  double mean_width_frac;
  double mean_height_frac;

  int n = 0;

  // Ite