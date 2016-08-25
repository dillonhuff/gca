#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_to_pocket.h"
#include "synthesis/axis_3.h"
#include "synthesis/millability.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  point n(0, -1, 0);

  auto mesh = parse_stl(argv[1], 0.001);
  feature_decomposition* f = build_feature_decomposition(mesh, n);

  vector<pocket> pockets = feature_pockets(*f, n);

  cout << "# of pockets = " << pockets.size() << endl;

}
