#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "geometry/point.h"
#include "geometry/polygon.h"
#include "geometry/triangle.h"
#include "system/algorithm.h"

using namespace gca;
using namespace std;

int name_length = 80;

struct stl_data {
  string name;
  vector<triangle> triangles;

  stl_data(string namep) : name(namep) {}
};

float parse_float(ifstream& s) {
  char f_buf[sizeof(float)];
  s.read(f_buf, 4);
  float* fptr = (float*) f_buf;
  return *fptr;
}

point parse_point(ifstream& s) {
  float x = parse_float(s);
  float y = parse_float(s);
  float z = parse_float(s);
  return point(x, y, z);
}

stl_data parse_stl(const string& stl_path) {
  ifstream stl_file(stl_path.c_str(), ios::in | ios::binary);
  if (!stl_file) {
    cout << "ERROR: COULD NOT READ FILE" << endl;
    assert(false);
  }

  char header_info[80] = "";
  char n_triangles[4];
  stl_file.read(header_info, 80);
  printf("HEADER: %s\n", header_info);
  stl_file.read(n_triangles, 4);
  string h(header_info);
  stl_data info(h);
  unsigned int* r = (unsigned int*) n_triangles;
  unsigned int num_triangles = *r;
  printf("# triangles = %u\n", num_triangles);
  for (unsigned int i = 0; i < num_triangles; i++) {
    auto normal = parse_point(stl_file);
    auto v1 = parse_point(stl_file);
    auto v2 = parse_point(stl_file);
    auto v3 = parse_point(stl_file);
    info.triangles.push_back(triangle(normal, v1, v2, v3));
    char dummy[2];
    stl_file.read(dummy, 2);
  }
  return info;
}

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
  // for (auto t : triangles) {
  //   cout << t << endl;
  // }
}
