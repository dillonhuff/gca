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

    box bounding = mesh.bounding_box();

    cout << "Bounding box = " << endl;
    cout << bounding << endl;
    
    vtk_debug_mesh(mesh);
    // point axis = part_axis(mesh);
    // point neg_axis = part_axis(mesh);

    // cout << "Major part axis   = " << axis << endl;
    // cout << "Length along axis = " << diameter(axis, mesh) << endl;

    // feature_decomposition* f = build_feature_decomposition(mesh, axis);
    // feature_decomposition* g = build_feature_decomposition(mesh, -1*axis);

    // cout << "# of features along " << axis << "  = " << collect_features(f).size() << endl;
    // cout << "# of features along " << neg_axis << " = " << collect_features(g).size() << endl;

    // vtk_debug_mesh(mesh);
    //    vtk_debug_feature_tree(f);
  };

  read_dir(argv[1], check_mesh);

}  
