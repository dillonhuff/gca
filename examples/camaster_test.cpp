#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_to_pocket.h"
#include "synthesis/gcode_generation.h"
#include "utils/check.h"

using namespace gca;
using namespace std;

typedef bg::model::d2::point_xy<double> boost_point_2;
typedef boost::geometry::model::polygon<boost_point_2> boost_poly_2;
typedef boost::geometry::model::multi_polygon<boost_poly_2> boost_multipoly_2;
typedef boost::geometry::model::multi_point<boost_point_2> boost_multipoint_2;

std::vector<pocket>
engraving_pockets_for_feature(const std::vector<tool>& tools, const feature& f) {
  DBG_ASSERT(angle_eps(f.normal(), point(0, 0, 1), 0.0, 1.0));

  //  vtk_debug_feature(f);

  vector<pocket> pockets;
  auto depths = f.range_along(point(0, 0, 1));
  oriented_polygon p(point(0, 0, 1), f.base().vertices());
  pockets.push_back(trace_pocket(depths.second, depths.first, p));

  for (unsigned i = 0; i < f.base().holes().size(); i++) {//auto& h : f.base().holes()) {
    vector<point> h = f.base().hole(i);

    cout << "Creating pockets for holes" << endl;

    //    vtk_debug_ring(h);

    oriented_polygon hp(point(0, 0, 1), h);
    pockets.push_back(trace_pocket(depths.second, depths.first, hp));
  }
  return pockets;
}

std::vector<pocket>
build_engraving_pockets(const std::vector<tool>& tools,
			std::vector<feature>& features) {

  // delete_if(features, [&t1](const feature& f) {
  //     return t1.cross_section_area() > bg::area(to_boost_poly_2(f.base()));
  //   });

  cout << "# of features after area culling = " << features.size() << endl;

  vtk_debug_features(ptrs(features));

  DBG_ASSERT(features.size() > 0);

  vector<pocket> pockets;
  for (auto f : features) {
    concat(pockets, engraving_pockets_for_feature(tools, f));
  }

  cout << "Done building pockets" << endl;
  cout << "Number of pockets = " << pockets.size() << endl;

  DBG_ASSERT(pockets.size() > 0);

  return pockets;

}

int main(int argc, char** argv) {
  arena_allocator a;
  set_system_allocator(&a);

  double start_x = 5.0;
  double end_x = 5.5;
  double start_y = 5.0;
  double end_y = 5.5;
  
  vector<boost_poly_2> dark_polygons;
  boost_poly_2 p;
  bg::append(p, bg::model::d2::point_xy<double>(start_x, start_y));
  bg::append(p, bg::model::d2::point_xy<double>(start_x, end_y));
  bg::append(p, bg::model::d2::point_xy<double>(end_x, end_y));
  bg::append(p, bg::model::d2::point_xy<double>(end_x, start_y));
  bg::correct(p);

  dark_polygons.push_back(p);

  
  

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
    gca::labeled_polygon_3 lp = gca::to_labeled_polygon_3(id_rotation, 0.5, r);

    check_simplicity(lp);
    
    dark_polys.push_back(lp);
  }

  gca::vtk_debug_polygons(dark_polys);

  double depth = 0.05;
  vector<gca::feature> features;
  for (auto dark_area : dark_polys) {
    features.push_back(gca::feature(depth, dark_area));
  }

  cout << "Done building features" << endl;
  cout << "Number of features = " << features.size() << endl;

  tool t1(0.01, 3.0, 4, HSS, FLAT_NOSE, 2);
  vector<tool> tools{t1};

  vector<pocket> pockets = build_engraving_pockets(tools, features);

  cout << "Milling pockets" << endl;
  vector<toolpath> toolpaths = mill_pockets(pockets, tools, ALUMINUM);
  cout << "Done milling pockets" << endl;

  int num_empty_toolpaths = 0;
  for (auto t : toolpaths) {
    if (t.lines.size() == 0) {
      num_empty_toolpaths++;
    }
  }

  cout << "# of empty toolpaths = " << num_empty_toolpaths << endl;

  cout << "Building GCODE" << endl;
  auto program = build_gcode_program("Engraving",
				     toolpaths,
				     camaster_prefix_blocks,
				     camaster_suffix_blocks,
				     camaster_engraving);
  cout << "Done building GCODE" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);

  cout << program.name << endl;
  cout << program.blocks << endl;

  return 0;
}
