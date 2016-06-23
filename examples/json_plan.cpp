#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>

#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/axis_3.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"
#include "system/parse_stl.h"

using boost::property_tree::ptree;

using namespace gca;
using namespace std;

ptree encode_json(const triangle_t t);
ptree encode_json(const point p);
ptree encode_json(const gcode_program& prog);
ptree encode_face_list_json(const triangular_mesh& m);
ptree encode_json(const triangular_mesh& m);
ptree encode_json(const fabrication_setup& prog);
ptree encode_json(const fabrication_plan& plan);

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

ptree encode_json(const point p) {
  ptree children;

  ptree v1;
  v1.put("", p.x);
  children.push_back(std::make_pair("", v1));

  ptree v2;
  v2.put("", p.y);
  children.push_back(std::make_pair("", v2));

  ptree v3;
  v3.put("", p.z);
  children.push_back(std::make_pair("", v3));

  return children;
}

ptree encode_json(const gcode_program& prog) {
  ptree p;
  stringstream ss;
  ss << prog.blocks;
  p.put("", ss.str());
  return p;
}

template<typename T>
ptree encode_json(const std::vector<T>& elems) {
  ptree children;
  for (auto e : elems) {
    children.push_back(std::make_pair("", encode_json(e)));
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

ptree encode_json(const vice& v) {
  box main = main_box(v);
  box upper_clamp = upper_clamp_box(v);
  box lower_clamp = lower_clamp_box(v);

  vector<triangular_mesh> vice_boxes;
  vice_boxes.push_back(make_mesh(box_triangles(main), 0.001));
  vice_boxes.push_back(make_mesh(box_triangles(upper_clamp), 0.001));
  vice_boxes.push_back(make_mesh(box_triangles(lower_clamp), 0.001));

  return encode_json(vice_boxes); //encode_json(m);
}

ptree encode_json(const fabrication_setup& prog) {
  ptree p;
  p.add_child("partMesh", encode_json(prog.part));
  p.add_child("gcode", encode_json(prog.prog));
  p.add_child("vice", encode_json(prog.v));
  return p;
}

ptree encode_json(const fabrication_plan& plan) {
  ptree p;
  p.add_child("clippingPrograms", encode_json(plan.stock_clipping_programs()));
  p.add_child("setups", encode_json(plan.steps()));
  return p;
}

struct plan_inputs {
  std::vector<tool> tools;
  workpiece workpiece_dims;
  fixtures fixes;

  plan_inputs(const std::vector<tool>& p_tools,
	      const workpiece& p_workpiece_dims,
	      const fixtures& p_fixes)
    : tools(p_tools), workpiece_dims(p_workpiece_dims), fixes(p_fixes) {}

};

template<typename T>
T decode_json(const ptree& p) {
  assert(false);
}

// TODO: Actual decoding for elements
template<>
tool decode_json(const ptree& p) {
  return tool(0.3, 3.0, FLAT_NOSE);
}

template<>
vice decode_json(const ptree& p) {
  return current_setup();
}

template<>
plate_height decode_json(const ptree& p) {
  assert(false);
}

template<typename T>
std::vector<T> decode_vector(const ptree& p) {
  std::vector<T> elems;
  BOOST_FOREACH(const ptree::value_type& v, p.get_child("")) {
    elems.push_back(decode_json<T>(v.second));
  }
  return elems;
}

std::vector<tool> decode_tools_json(const ptree& p) {
  return decode_vector<tool>(p);
  // tool t1(0.30, 3.0, FLAT_NOSE);
  // tool t2(0.14, 3.15, FLAT_NOSE);
  // vector<tool> tools{t1, t2};
}

fixtures decode_fixtures_json(const ptree& p) {
  vice v = decode_json<vice>(p.get_child("vice"));
  std::vector<plate_height> plates =
    decode_vector<plate_height>(p.get_child("base_plates"));
  // vice v = current_setup();
  // std::vector<plate_height> plates{0.1, 0.3};
  // fixtures fixes(v, plates);
  //  return fixes;
  return fixtures(v, plates);
}

workpiece decode_workpiece_json(const ptree& p) {
  // workpiece workpiece_dims(3.5, 3.0, 3.0);
  assert(false);
}

plan_inputs parse_inputs_json(const std::string& s) {
  ifstream ins(s);
  ptree inputs;
  read_json(ins, inputs);

  std::vector<tool> tools = decode_tools_json(inputs.get_child("tools"));
  fixtures fixes = decode_fixtures_json(inputs.get_child("fixtures"));
  workpiece workpiece_dims = decode_workpiece_json(inputs.get_child("workpiece"));

  return plan_inputs(tools, workpiece_dims, fixes);
}

int main(int argc, char* argv[]) {
  assert(argc == 4);

  arena_allocator a;
  set_system_allocator(&a);

  triangular_mesh mesh = parse_stl(argv[1], 0.001);

  plan_inputs inputs = parse_inputs_json(argv[2]);

  auto plan = make_fabrication_plan(mesh, inputs.fixes, inputs.tools, inputs.workpiece_dims);

  ptree json_tree = encode_json(plan);
  write_json(argv[3], json_tree);
}

