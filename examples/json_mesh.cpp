#include <fstream>

#include "geometry/triangular_mesh.h"
#include "system/algorithm.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

std::string json_string(const triangular_mesh& m) {
  string s = "{";
  s += "\"pts\" : [";
  for (unsigned i = 0; i < m.vertex_list().size(); i++) {
    point p = m.vertex_list()[i];
    s += "[" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + "]";
    if (i < m.vertex_list().size() - 1) {
      s += ", ";
    }
  }
  s += "], ";
  s += "\"faces\" : [";
  for (unsigned i = 0; i < m.face_indexes().size(); i++) {
    triangle_t t = m.triangle_vertices(i);
       s += "[" + std::to_string(t.v[0]) + ", " + std::to_string(t.v[1]) + ", " + std::to_string(t.v[2]) + "]";
    if (i < m.face_indexes().size() - 1) {
      s += ", ";
    }
  }
  s += "]";
  s += "}";
  return s;
}

int main(int argc, char* argv[]) {
  assert(argc == 3);

  arena_allocator a;
  set_system_allocator(&a);

  auto mesh = parse_stl(argv[1], 0.001);
  string json = json_string(mesh);
  ofstream json_out(argv[2], ofstream::out);
  json_out << json;
  json_out.close();
}

