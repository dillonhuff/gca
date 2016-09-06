#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "utils/check.h"

using namespace cv;
using namespace std;

int thresh = 1;
int max_thresh = 255;

RNG rng(12345);

typedef boost::geometry::model::d2::point_xy<double> boost_point_2;
typedef boost::geometry::model::polygon<boost_point_2> boost_poly_2;
typedef boost::geometry::model::multi_polygon<boost_poly_2> boost_multipoly_2;
typedef boost::geometry::model::multi_point<boost_point_2> boost_multipoint_2;

int main(int argc, char** argv) {
  if( argc != 2) {
    cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  Mat image;
  image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

  if(! image.data ) { // Check for invalid input
    cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }

  Mat gray_image;
  cvtColor( image, gray_image, CV_BGR2GRAY );

  Mat img_bw = gray_image > 240;

  double img_len = 4.5;
  double img_width = 1.5;

  double pixel_len = img_len / static_cast<double>(img_bw.cols);
  double pixel_width = img_width / static_cast<double>(img_bw.rows);

  cout << "pixel length = " << pixel_len << " inches" << endl;
  cout << "pixel width = " << pixel_width << " inches" << endl;

  unsigned black_pixels = 0;
  unsigned white_pixels = 0;
  for(int i = 0; i < img_bw.rows; i++) {

    const unsigned char* Mi = img_bw.ptr<unsigned char>(i);

    double start_y = pixel_width*i;
    double end_y = pixel_width*(i + 1);

    for(int j = 0; j < img_bw.cols; j++) {

      double start_x = pixel_len*i;
      double end_x = pixel_len*(i + 1);
      
      boost_poly_2 p;

      if (Mi[j] == 0) {
	black_pixels += 1;
      } else if (Mi[j] == 255) {
	white_pixels += 1;
      } else {
	DBG_ASSERT(false);
      }
    }

  }

  cout << "Num black pixels = " << black_pixels << endl;
  cout << "Num white pixels = " << white_pixels << endl;
  // imwrite( "./Gray_Image.jpg", gray_image );

  // namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );

  // imshow( "Gray image", gray_image );
  // imshow( "Display window", image );
  // imshow( "Black and white", img_bw );

  // waitKey(0);

  
  return 0;
}
