#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/axis_3.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {
  assert(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto triangles = parse_stl(argv[1]).triangles;
  //  select_visible_triangles(triangles);
  double tool_radius = 0.05;
  //  double cut_depth = 0.1;
  double workpiece_height = 0.8;

  auto pockets = make_pockets(triangles, workpiece_height);
  cout << "# of pockets = " << pockets.size() << endl;
  vector<polyline> pts;
  for (auto pocket : pockets) {
    auto bounds = pocket.get_boundaries();
    for (auto bound : bounds) {
      pts.push_back(to_polyline(interior_offset(project(bound, 0.0), tool_radius)));
    }
    // vector<oriented_polygon> bound_polys(bounds.size());
    // transform(begin(bounds), end(bounds), begin(bound_polys),
    // 	      [tool_radius](const oriented_polygon& p)
    // 	      { return interior_offset(project(p, 0.0), tool_radius); });
    // box b = bounding_box(begin(bound_polys), end(bound_polys));
    // auto new_pts = sample_points_2d(b,
    // 				    tool_radius,
    // 				    tool_radius,
    // 				    tool_radius);
    // if (new_pts.size() > 1) {
    //   pts.push_back(new_pts);
    // }
    //pts.insert(end(pts), begin(new_pts), end(new_pts));
  }
  // if (!(pockets.size() == 1)) {
  //   cout << "# of pockets = " << pockets.size() << endl;
  //   assert(false);
  // }
  // auto lines = rough_pockets(pockets,
  // 			     tool_radius,
  // 			     cut_depth);
  // auto lines = mill_surface_lines(triangles,
  // 				  tool_radius,
  // 				  cut_depth,
  // 				  workpiece_height);

  // point shift(0, 0, 0); //(-1, -1, 0); //-3, -2.5, -0.5);
  // vector<polyline> shifted_lines;
  // for (auto l : lines) {
  //   vector<point> pts;
  //   for (auto pt : l) {
  //     pts.push_back(pt + shift);
  //   }
  //   shifted_lines.push_back(pts);
  // }

  auto bs = emco_f1_code(pts); //shifted_lines);

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  cout << bs << endl;
}

