#ifndef GCA_JSON_H__
#define GCA_JSON_H__

#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

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
using namespace std;

namespace gca {

  struct plan_inputs {
    std::vector<tool> tools;
    workpiece workpiece_dims;
    fixtures fixes;

    plan_inputs(const std::vector<tool>& p_tools,
		const workpiece& p_workpiece_dims,
		const fixtures& p_fixes)
      : tools(p_tools), workpiece_dims(p_workpiece_dims), fixes(p_fixes) {}

  };

  ptree encode_json(const triangle_t t);
  ptree encode_json(const point p);
  ptree encode_json(const gcode_program& prog);
  ptree encode_face_list_json(const triangular_mesh& m);
  ptree encode_json(const triangular_mesh& m);
  ptree encode_json(const fabrication_setup& prog);
  ptree encode_json(const fabrication_plan& plan);

  template<typename T>
  static
  ptree encode_json(const std::vector<T>& elems) {
    ptree children;
    for (auto e : elems) {
      children.push_back(std::make_pair("", encode_json(e)));
    }
    return children;
  }
  
  template<typename T>
  T decode_json(const ptree& p) {
    assert(false);
  }

  template<>
  inline double decode_json(const ptree& p) {
    std::string s = p.get<std::string>("");
    return std::stod(s);
  }

  template<>
  inline unsigned decode_json(const ptree& p) {
    std::string s = p.get<std::string>("");
    return std::stoul(s);
  }

  // TODO: Actual decoding for elements
  template<>
  inline tool decode_json(const ptree& p) {
    double diam = decode_json<double>(p.get_child("tool_diameter"));
    double length = decode_json<double>(p.get_child("tool_length"));
    unsigned num_flutes = decode_json<unsigned>(p.get_child("tool_num_flutes"));
    material mat = decode_json<material>(p.get_child("tool_material"));
    return tool(diam, length, num_flutes, mat, FLAT_NOSE);
  }

  template<>
  inline vice decode_json(const ptree& p) {
    return current_setup();
  }

  template<typename T>
  std::vector<T> decode_vector(const ptree& p) {
    std::vector<T> elems;
    BOOST_FOREACH(const ptree::value_type& v, p.get_child("")) {
      elems.push_back(decode_json<T>(v.second));
    }
    return elems;
  }

  template<>
  inline point decode_json(const ptree& p) {
    std::vector<double> coords = decode_vector<double>(p.get_child(""));
    return point(coords[0], coords[1], coords[2]);
  }

  plan_inputs parse_inputs_json(const std::string& s);

}

#endif
