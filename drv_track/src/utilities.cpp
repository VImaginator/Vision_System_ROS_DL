
#include "utilities.h"

using namespace cv;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

Utilities::Utilities()
{
}

std::string Utilities::intToString(int number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

void Utilities::markImage(Mat img_in, Rect roi, Mat &img_out, std::vector<unsigned int> &mask_id, float &color_mean)
{
  Mat mask(img_in.rows + 2, img_in.cols + 2, CV_8UC1, Scalar(0));
  Mat img_hsv;
  cvtColor(img_in, img_hsv, CV_BGR2HSV);
  Point roi_center = Point(roi.x + roi.width / 2, roi.y + roi.height / 2);

  floodFill(img_hsv, mask, roi_center, Scalar(255,255,255), 0, Scalar(30,40,50), Scalar(30,40,50), 4 | (255 << 8) | CV_FLOODFILL_FIXED_RANGE | CV_FLOODFILL_MASK_ONLY);

  Mat element(9, 9, CV_8U, Scalar(255));
  Mat closed;
  morphologyEx(mask, closed, MORPH_CLOSE, element);

  int hueTemp = 0;
  int count = 0;

  // get the pixel id in mask
  for (size_t r = 0; r < closed.rows; r++)
  {
    for (size_t c = 0; c < closed.cols; c++)
    {
      if (closed.at<uchar>(r, c) > 0)
      {
        mask_id.push_back((r - 1) * 640 + c - 1);
        Vec3b hsvPixel = img_hsv.at<Vec3b>(r, c);
        hueTemp += hsvPixel[0];
        count++;
      }
    }
  }
  // use this to judge if the tracked object has changed
  color_mean = hueTemp / count;

  vector<vector<Point> > contours;
  findContours(closed, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  drawContours(img_out, contours, 0, Scalar(196,0,225), 2);
  rectangle(img_out, roi, Scalar(0, 255, 0));
  drawObject(roi_center.x, roi_center.y, img_out);
}

void Utilities::drawObject(int x, int y, Mat &frame)
{
  //use some of the openCV drawing functions to draw crosshairs
  //on your tracked image!

  circle(frame,Point(x,y),12,Scalar(0,255,0),2);
  if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
  else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
  if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
  else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
  if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
  else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
  if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
  else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

  putText(frame, intToString(x) + "," + intToString(y), Point(x,y+30), 1, 1, Scalar(0,255,0), 1.5);
}

void Utilities::expandGt(Rect &gt, int margin)
{
  int w = gt.width;
  int h = gt.height;
  int x = gt.x;
  int y = gt.y;
  if (x - margin < 0)
  {
    w += x;
    gt.x = 0;
  }
  else
  {
    gt.x = x - margin;
    w += margin;
  }
  if (x + gt.width + margin >= 640)
  {
    w += 640 - x - gt.width;
  }
  else
  {
    w += margin;
  }
  if (y - margin < 0)
  {
    h += y;
    gt.y = 0;
  }
  else
  {
    gt.y = y - margin;
    h += margin;
  }
  if (y + gt.height + margin >= 480)
  {
    h += 480 - y - gt.height;
  }
  else
  {
    h += margin;
  }
  gt.width = w;
  gt.height = h;
}