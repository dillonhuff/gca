#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "geometry/point.h"
#include "geometry/polygon.h"
#include "geometry/triangle.h"
#include "system/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

int name_length = 80;

int main(int argc, char* argv[]) {
  assert(argc == 2);
  string stl_path = argv[1];
  auto info = parse_stl(stl_path);
  auto triangles = info.triangles;
  cout << "# triangles: " << triangles.size() << endl;
  delete_if(triangles, [](const triangle& t) { return !is_upward_facing(t, 1e-2); });
  cout << "# upward facing triangles: " << triangles.size() << endl;
  greedy_adjacent_chains(triangles.begin(), triangles.end(),
			 [](const triangle& x,
			    const triangle& y)
			 { return same_orientation(x, y, 0.01); });
  cout << "Grouped by adjacency" << endl;
  vector<vector<triangle>> constant_orientation_groups;
  split_by(triangles, constant_orientation_groups,
	   [](const triangle& x,
	      const triangle& y)
	   { return same_orientation(x, y, 0.01); });
  cout << "# constant orientation groups: " << constant_orientation_groups.size() << endl;
  vector<oriented_polygon> polys;
  for (auto g : constant_orientation_groups) {
    assert(g.size() > 0);
    auto merged_polys = merge_triangles(g);
    polys.insert(polys.end(), merged_polys.begin(), merged_polys.end());
  }
}
