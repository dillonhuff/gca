#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/shapes_to_gcode.h"
#include "backend/toolpath_generation.h"
#include "utils/algorithm.h"
#include "system/parse_stl.h"
#include "system/json.h"

using boost::property_tree::ptree;

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {
  assert(argc == 4);

  arena_allocator a;
  set_system_allocator(&a);

  triangular_mesh mesh = parse_stl(argv[1], 0.001);

  plan_inputs inputs = parse_inputs_json(argv[2]);
  auto plan = make_fabrication_plan(mesh, inputs.fixes, inputs.tools, {inputs.workpiece_dims});

  ptree json_tree = encode_json(plan);
  write_json(argv[3], json_tree);

  cout << "Finished Plan" << endl;
}

