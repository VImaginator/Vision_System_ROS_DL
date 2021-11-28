// Visualize the tracker performance.

#include <string>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "network/regressor.h"
#include "loader/loader_alov.h"
#include "loader/loader_vot.h"
#include "tracker/tracker.h"
#include "tracker/tracker_manager.h"

using std::string;

// Set to true to show more detailed tracking visualizations.
const bool show_intermediate_output = false;

int main (int argc, char *argv[]) {
  if (argc < 5) {
    std::cerr << "Usage: " << argv[0]
              << " deploy.prototxt network.caffemodel videos_folder annotations_folder"
     