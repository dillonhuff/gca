#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/axis_3.h"
#include "synthesis/mesh_to_gcode.h"
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

  auto box_triangles = parse_stl(argv[1]).triangles;
  auto mesh = make_mesh(box_triangles, 0.001);

  vice test_vice(1.5, 1.5, 0.75, Y_AXIS);
  tool t1(0.3, FLAT_NOSE);
  vector<tool> tools{t1};
  workpiece_dimensions workpiece_dims(1.5, 1.2, 1.5);
  auto result_programs = mesh_to_gcode(mesh, test_vice, tools, workpiece_dims);

  cout << "All programs" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  for (auto program : result_programs) {
    cout << program.name << endl;
    cout << program.blocks << endl;
  }
  // double tool_radius = 0.5; //0.05;
  // double cut_depth = 0.1;
  // double workpiece_height = 1.0;
  // double face_height = 0.8;

  // box b(0, face_height,
  // 	0, face_height,
  // 	0, face_height);

  // vector<point> pts = sample_points_2d(b,
  // 				       tool_radius / 2.0,
  // 				       tool_radius / 2.0,
  // 				       0.0);

  // vector<vector<point>> pt_lines;
  // split_by(pts, pt_lines,
  // 	   [](const point l, const point r)
  // 	   { return within_eps(l.x, r.x); });
  // vector<polyline> lines;
  // for (auto pt_group : pt_lines) {
  //   lines.push_back(pt_group);
  // }

  // auto final_lines = tile_vertical(lines,
  // 				   workpiece_height,
  // 				   face_height,
  // 				   cut_depth);
  // point shift(0, 0, -face_height);
  // auto bs = emco_f1_code(shift_lines(final_lines, shift)); //shifted_lines);

  // cout.setf(ios::fixed, ios::floatfield);
  // cout.setf(ios::showpoint);
  // cout << bs << endl;
}

