#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "geometry/point.h"

using namespace gca;
using namespace std;

int name_length = 80;

struct triangle {
  point normal;
  point v1;
  point v2;
  point v3;

  triangle(point normalp, point v1p, point v2p, point v3p) :
    normal(normalp), v1(v1p), v2(v2p), v3(v3p) {}
};

ostream& operator<<(ostream& out, const triangle& t) {
  cout << "---- TRIANGLE ----" << endl;
  cout << t.normal << endl;;
  cout << t.v1 << endl;;
  cout << t.v2 << endl;;
  cout << t.v3 << endl;;
  return out;
}

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

int main() {
  string stl_path = "/Users/dillon/Downloads/ktoolcor.stl";
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
  printf("# triangles: %s\n", n_triangles);
  unsigned int* r = (unsigned int*) n_triangles;
  unsigned int num_triangles = *r;
  printf("# triangles = %u\n", num_triangles);
  vector<triangle> triangles;
  for (unsigned int i = 0; i < num_triangles; i++) {
    auto normal = parse_point(stl_file);
    auto v1 = parse_point(stl_file);
    auto v2 = parse_point(stl_file);
    auto v3 = parse_point(stl_file);
    triangles.push_back(triangle(normal, v1, v2, v3));
    char dummy[2];
    stl_file.read(dummy, 2);
  }

  for (auto t : triangles) {
    cout << t << endl;
  }
  
  // ifstream t(stl_path);
  // string str((istreambuf_iterator<char>(t)),
  // 	     istreambuf_iterator<char>());
  // int file_index = 0;
  // string name = str.substr(0, name_length - 1);
  // cout << name << endl;
  // file_index = name_length;
  // string rest = str.substr(name_length);
  // const char* rs = rest.c_str();
  // const unsigned int i = (const long int) rs;
  // cout << i << endl;
}
