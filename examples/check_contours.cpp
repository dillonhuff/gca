#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "process_planning/axis_location.h"
#include "synthesis/visual_debug.h"
#include "utils/algorithm.h"
#include "system/file.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {

  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);
  
  auto check_mesh = [](const std::string& n) {
    cout << "Reading = " << n << endl;

    auto mesh = parse_stl(n, 0.001);
    point axis = part_axis(mesh);

    cout << "Major part axis   = " << axis << endl;
    cout << "Length along axis = " << diameter(axis, mesh) << endl;

    feature_decomposition* f = build_feature_decomposition(mesh, axis);

    cout << "# of features = " << collect_features(f).size() << endl;

    //    vtk_debug_mesh(mesh);
    //    vtk_debug_feature_tree(f);
  };

  read_dir(argv[1], check_mesh);

}  
  

  // auto box_triangles = parse_stl(argv[1]).triangles;
  // auto mesh = make_mesh(box_triangles, 0.001);

