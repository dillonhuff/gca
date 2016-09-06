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

bool has_black_neighbor_tblr(const int i, const int j, Mat& img_bw) {
  vector<int> row_neighbor_inds;
  vector<int> col_neighbor_inds;

  if (i == 0) {
    row_neighbor_inds.push_back(i + 1);
  } else if (i == (img_bw.rows - 1)) {
    row_neighbor_inds.push_back(i - 1);
  } else {
    row_neighbor_inds.push_back(i + 1);
    row_neighbor_inds.push_back(i - 1);
  }

  if (j == 0) {
    col_neighbor_inds.push_back(j + 1);
  } else if (j == (img_bw.cols - 1)) {
    col_neighbor_inds.push_back(j - 1);
  } else {
    col_neighbor_inds.push_back(j + 1);
    col_neighbor_inds.push_back(j - 1);
  }
  
  for (auto r : row_neighbor_inds) {
    const unsigned char* Mi = img_bw.ptr<unsigned char>(r);

    for (auto c : col_neighbor_inds) {
      if (Mi[c] == 0) { return true; }
    }
  }

  return false;
}

typedef std::pair<int, int> pixel;

namespace std
{
    template<> struct hash<pixel>
    {
        typedef pixel argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const& s) const
        {
            result_type const h1 ( std::hash<int>()(s.first) );
            result_type const h2 ( std::hash<int>()(s.second) );
            return h1 ^ (h2 << 1);
        }
    };
}

bool operator==(const pixel l, const pixel r) {
  return (l.first == r.first) && (l.second == r.second);
}

std::vector<pixel> tblr_neighbor_pixels(const pixel l) {
  int top_row = l.first - 1;
  int bottom_row = l.first + 1;

  int left_col = l.second - 1;
  int right_col = l.second + 1;

  vector<pixel> neighbors;
  neighbors.push_back(pixel(top_row, l.second));
  neighbors.push_back(pixel(bottom_row, l.second));

  neighbors.push_back(pixel(l.first, left_col));
  neighbors.push_back(pixel(l.first, right_col));

  return neighbors;
}

bool tblr_neighbors(const pixel l, const pixel r) {
  for (auto n : tblr_neighbor_pixels(l)) {
    if (r == n) { return true; }
  }
  return false;
}

std::vector<pixel> black_image_pixels(Mat& img_bw) {
  vector<pixel> black_pixels;
  unsigned white_pixels = 0;
  for(int i = 0; i < img_bw.rows; i++) {

    const unsigned char* Mi = img_bw.ptr<unsigned char>(i);

    for(int j = 0; j < img_bw.cols; j++) {

      if (Mi[j] == 0) {
	black_pixels.push_back(pixel(i, j));
      } else if (Mi[j] == 255) {
	white_pixels += 1;
      } else {
	DBG_ASSERT(false);
      }
    }

  }

  cout << "Num black pixels = " << black_pixels.size() << endl;
  cout << "Num white pixels = " << white_pixels << endl;

  return black_pixels;
}

template<typename I, typename P>
std::vector<I>
dfs_by_value(std::unordered_set<I>& elems,
	     P neighbors) {
  vector<I> comp;
  if (elems.size() == 0) {
    return comp;
  }

  std::vector<I> buf;
  buf.push_back(*begin(elems));
  elems.erase(begin(elems));

  while (buf.size() > 0) {
    auto next = buf.back();
    comp.push_back(next);
    buf.pop_back();

    std::vector<I> elems_to_remove = neighbors(next, elems);
    concat(buf, elems_to_remove);

    for (auto e : elems_to_remove) {
      elems.erase(e);
    }

  }

  return comp;
}


template<typename I, typename P>
std::vector<std::vector<I>>
connected_components_by_value(const std::vector<I>& i_elems, P p) {
  std::vector<std::vector<I>> components;
  std::unordered_set<I> elems(begin(i_elems), end(i_elems));
  while (elems.size() > 0) {
    components.push_back(dfs_by_value(elems, p));
  }

  return components;
}

boost_multipoly_2 pixel_polygon(const std::vector<pixel>& pixels,
				const double pixel_len,
				const double pixel_width) {
  boost_multipoly_2 dark_areas;

  for (auto pixel : pixels) {
    int i = pixel.first;
    int j = pixel.second;

    // TODO: Come up with a better solution, or do pixel dilation only
    // on the pixels that are involved in a butterfly
    double start_y = -1*(pixel_width*i);
    double end_y = -1*(pixel_width*(i + 1)) - 0.001;

    double start_x = pixel_len*j;
    double end_x = pixel_len*(j + 1) + 0.001;

    boost_poly_2 p;
    bg::append(p, bg::model::d2::point_xy<double>(start_x, start_y));
    bg::append(p, bg::model::d2::point_xy<double>(start_x, end_y));
    bg::append(p, bg::model::d2::point_xy<double>(end_x, end_y));
    bg::append(p, bg::model::d2::point_xy<double>(end_x, start_y));
    bg::correct(p);

    boost_multipoly_2 r_tmp = dark_areas;
    boost::geometry::clear(dark_areas);
    boost::geometry::union_(r_tmp, p, dark_areas);
  }

  return dark_areas;
}

int main(int argc, char** argv) {
  if( argc != 2) {
    cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  arena_allocator a;
  set_system_allocator(&a);

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

  vector<pixel> black_pixels = black_image_pixels(img_bw);

  auto neighbs = [](const pixel p, const std::unordered_set<pixel>& others) {
    vector<pixel> ns;
    for (auto n : tblr_neighbor_pixels(p)) {
      if (others.find(n) != end(others)) {
	ns.push_back(n);
      }
    }

    return ns;
  };

  vector<vector<pixel>> pixel_groups =
    connected_components_by_value(black_pixels, neighbs);

  cout << "# black pixel groups = " << pixel_groups.size() << endl;

  vector<boost_poly_2> dark_polygons;
  for (auto pixel_group : pixel_groups) {
    
    if (pixel_group.size() < 20000) {
      cout << "Creating dark polygon from " << pixel_group.size() << " pixels" << endl;
      boost_multipoly_2 area_poly = pixel_polygon(pixel_group,
						  pixel_len,
						  pixel_width);
      cout << "Done creating dark polygon" << endl;
      for (auto area_p : area_poly) {
	dark_polygons.push_back(area_p);
      }

    } else {
      cout << "PIXEL GROUP IS TOO LARGE, SKIPPING FOR NOW" << endl;
    }

  }

  cout << "# of dark polygons = " << dark_polygons.size() << endl;
  auto max_poly = max_e(dark_polygons,
			[](const boost_poly_2& p) { return bg::area(p); });

  cout << "Max area polygon = " << bg::area(max_poly) << endl;

  auto min_poly = min_e(dark_polygons,
			[](const boost_poly_2& p) { return bg::area(p); });

  cout << "Min area polygon = " << bg::area(min_poly) << endl;
  
  const gca::rotation id_rotation =
    gca::rotate_from_to(gca::point(0, 0, 1), gca::point(0, 0, 1));
  vector<gca::labeled_polygon_3> dark_polys;

  for (auto& r : dark_polygons) {
    gca::labeled_polygon_3 lp = gca::to_labeled_polygon_3(id_rotation, 0.0, r);

    check_simplicity(lp);
    
    dark_polys.push_back(lp);
  }

  gca::vtk_debug_polygons(dark_polys);

  double depth = 0.1;
  vector<gca::feature> features;
  for (auto dark_area : dark_polys) {
    features.push_back(gca::feature(depth, dark_area));
  }

  cout << "Done building features" << endl;
  cout << "Number of features = " << features.size() << endl;

  tool t1(0.01, 3.0, 4, HSS, FLAT_NOSE);
  vector<tool> tools{t1};

  delete_if(features, [&t1](const feature& f) {
      return t1.cross_section_area() > bg::area(to_boost_poly_2(f.base()));
    });

  cout << "# of features after area culling = " << features.size() << endl;

  vtk_debug_features(ptrs(features));

  DBG_ASSERT(features.size() > 0);

  vector<pocket> pockets;
  for (auto f : features) {
    concat(pockets, pockets_for_feature(f));
  }

  cout << "Done building pockets" << endl;
  cout << "Number of pockets = " << pockets.size() << endl;

  DBG_ASSERT(pockets.size() > 0);

  cout << "Milling pockets" << endl;
  vector<toolpath> toolpaths = mill_pockets(pockets, tools, ALUMINUM);
  cout << "Done milling pockets" << endl;

  cout << "Building GCODE" << endl;
  auto program = build_gcode_program("Engraving", toolpaths, emco_f1_code_G10_TLC);
  cout << "Done building GCODE" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);

  cout << program.name << endl;
  cout << program.blocks << endl;

  return 0;
}
