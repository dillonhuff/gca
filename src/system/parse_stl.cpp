#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "system/parse_stl.h"

namespace gca {
  
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
    stl_file.read(n_triangles, 4);
    string h(header_info);
    stl_data info(h);
    unsigned int* r = (unsigned int*) n_triangles;
    unsigned int num_triangles = *r;

    cout << "# of triangles in " << stl_path << " = " << num_triangles << endl;
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

  triangular_mesh parse_stl(const string& stl_path, const double tolerance) {
    return make_mesh(parse_stl(stl_path).triangles, tolerance);
  }

}
