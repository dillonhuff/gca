#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>

#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "backend/shapes_to_gcode.h"
#include "backend/toolpath_generation.h"
#include "utils/algorithm.h"
#include "system/parse_stl.h"
#include "system/json.h"

using namespace gca;

int main(int argc, char* argv[]) {
  assert(argc == 3);

  arena_allocator a;
  set_system_allocator(&a);

  auto mesh = parse_stl(argv[1], 0.001);

  vice v = current_setup();
  std::vector<plate_height> plates{0.1, 0.3};
  fixtures fixes(v, plates);

  tool t1(0.30, 3.0, 4, HSS, FLAT_NOSE);
  tool t2(0.14, 3.15, 2, HSS, FLAT_NOSE);
  vector<tool> tools{t1, t2};
  workpiece workpiece_dims(3.5, 3.0, 3.0, ACETAL);

  auto plan = make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

  ptree json_tree = encode_json(plan);
  write_json(argv[2], json_tree);
}

