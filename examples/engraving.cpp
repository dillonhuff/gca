#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_to_pocket.h"
#include "synthesis/gcode_generation.h"
#include "utils/check.h"

using namespace gca;
using namespace cv;
using namespace std;

int thresh = 1;
int max_thresh = 255;

RNG rng(12345);

namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> boost_point_2;
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

  cout << "# rows = " << img_bw.rows << endl;
  cout << "# col  = " << img_bw.cols << endl;

  boost_multipoly_2 dark_areas;
  
  unsigned black_pixels = 0;
  unsigned white_pixels = 0;
  for(int i = 0; i < img_bw.rows; i++) {

    const unsigned char* Mi = img_bw.ptr<unsigned char>(i);

    double start_y = -1*(pixel_width*i);
    double end_y = -1*(pixel_width*(i + 1));

    for(int j = 0; j < img_bw.cols; j++) {

      double start_x = pixel_len*j;
      double end_x = pixel_len*(j + 1);

      if (Mi[j] == 0) {
      
	boost_poly_2 p;
	bg::append(p, bg::model::d2::point_xy<double>(start_x, start_y));
	bg::append(p, bg::model::d2::point_xy<double>(start_x, end_y));
	bg::append(p, bg::model::d2::point_xy<double>(end_x, end_y));
	bg::append(p, bg::model::d2::point_xy<double>(end_x, start_y));
	bg::correct(p);

	boost_multipoly_2 r_tmp = dark_areas;
	boost::geometry::clear(dark_areas);
	boost::geometry::union_(r_tmp, p, dark_areas);

	black_pixels += 1;
      } else if (Mi[j] == 255) {
	white_pixels += 1;
      } else {
	DBG_ASSERT(false);
      }
    }

    cout << "Done with row " << i << endl;

  }

  cout << "Num black pixels = " << black_pixels << endl;
  cout << "Num white pixels = " << white_pixels << endl;

  const gca::rotation id_rotation =
    gca::rotate_from_to(gca::point(0, 0, 1), gca::point(0, 0, 1));
  vector<gca::labeled_polygon_3> dark_polys;

  for (auto& r : dark_areas) {
    gca::labeled_polygon_3 lp = gca::to_labeled_polygon_3(id_rotation, 0.0, r);
    dark_polys.push_back(lp);
  }

  gca::vtk_debug_polygons(dark_polys);

  double depth = 0.1;
  vector<gca::feature> features;
  for (auto dark_area : dark_polys) {
    features.push_back(gca::feature(depth, dark_area));
  }


  vector<pocket> pockets;
  for (auto f : features) {
    concat(pockets, pockets_for_feature(f));
  }

  vector<tool> tools{tool(0.01, 3.0, 4, HSS, FLAT_NOSE)};
  vector<toolpath> toolpaths = mill_pockets(pockets, tools, ALUMINUM);

  auto program = build_gcode_program("Engraving", toolpaths, emco_f1_code_G10_TLC);

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);

  cout << program.name << endl;
  cout << program.blocks << endl;

  return 0;
}
