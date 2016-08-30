#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/visual_debug.h"
#include "utils/algorithm.h"
#include "system/file.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  auto check_mesh = [](const std::string& n) {
    cout << "Reading = " << n << endl;
    auto mesh = parse_stl(n, 0.001);
  };
  read_dir(argv[1], check_mesh);
}  
  // arena_allocator a;
  // set_system_allocator(&a);

  

  // auto box_triangles = parse_stl(argv[1]).triangles;
  // auto mesh = make_mesh(box_triangles, 0.001);

