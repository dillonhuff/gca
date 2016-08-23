#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
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

  auto mesh = parse_stl(argv[1], 0.001);
  feature_decomposition* f = build_feature_decomposition(mesh, point(0, 0, 1));
  vector<pocket> pockets = feature_pockets(*f);

  cout << "# of pockets = " << pockets.size() << endl;

}
