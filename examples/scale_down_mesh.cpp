#include "process_planning/axis_location.h"
#include "utils/algorithm.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  auto mesh = parse_stl(argv[1], 0.001);

  box bounding = mesh.bounding_box();

  point axis = part_axis(mesh);
  cout << "Axis = " << axis << endl;

  cout << "Diameter along axis = " << diameter(axis, mesh) << endl;

  cout << "Bounding box = " << endl;
  cout << bounding << endl;
}

