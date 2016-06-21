#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>

#include "geometry/triangular_mesh.h"
#include "system/algorithm.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

using boost::property_tree::ptree;

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

ptree encode_json(const triangle_t t) {
  ptree children;

  ptree v1;
  v1.put("", t.v[0]);
  children.push_back(std::make_pair("", v1));

  ptree v2;
  v2.put("", t.v[1]);
  children.push_back(std::make_pair("", v2));

  ptree v3;
  v3.put("", t.v[2]);
  children.push_back(std::make_pair("", v3));

  return children;
}

std::string encode_json(const point p) {
  return "pt";
}

template<typename T>
ptree encode_json(const std::vector<T>& elems) {
  ptree children;
  for (auto e : elems) {
    ptree p;
    p.put("", encode_json(e));
    children.push_back(std::make_pair("", p));
  }
  return children;
}

ptree encode_face_list_json(const triangular_mesh& m) {
  ptree children;
  for (auto i : m.face_indexes()) {
    triangle_t t = m.triangle_vertices(i);
    ptree p = encode_json(t);
    children.push_back(std::make_pair("", p));
  }
  return children;
}

ptree encode_json(const triangular_mesh& m) {
  ptree p;
  p.add_child("pts", encode_json(m.vertex_list()));
  p.add_child("faces", encode_face_list_json(m));
  return p;
}

int main(int argc, char* argv[]) {
  assert(argc == 3);

  arena_allocator a;
  set_system_allocator(&a);

  auto mesh = parse_stl(argv[1], 0.001);
  ptree json_tree = encode_json(mesh);
  write_json(argv[2], json_tree);
  // string json = json_string(mesh);
  // ofstream json_out(argv[2], ofstream::out);
  // json_out << json;
  // json_out.close();
}

