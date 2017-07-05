#ifndef GCA_JSON_H__
#define GCA_JSON_H__

#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "simulators/simulate_operations.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "backend/shapes_to_gcode.h"
#include "backend/toolpath_generation.h"
#include "utils/algorithm.h"
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
  T decode_json(const ptree& p);

  template<>
  inline double decode_json(const ptree& p) {
    std::string s = p.get<std::string>("");
    return std::stod(s);
  }

  template<>
  inline int decode_json(const ptree& p) {
    std::string s = p.get<std::string>("");
    return std::stoi(s);
  }
  
  template<>
  inline unsigned decode_json(const ptree& p) {
    std::string s = p.get<std::string>("");
    return std::stoul(s);
  }

  template<>
  inline material decode_json(const ptree& p) {
    std::string name = p.get<std::string>("");
    if (name == "HSS") {
      return HSS;
    } else if (name == "CARBIDE") {
      return CARBIDE;
    } else if (name == "ACETAL") {
      return ACETAL;
    } else if (name == "BRASS") {
      return BRASS;
    } else {
      DBG_ASSERT(false);
    }
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

  // TODO: Actually decode the vice
  template<>
  inline vice decode_json(const ptree& p) {
    point pos = decode_json<point>(p.get_child("vice_pos"));
    double x_len = decode_json<double>(p.get_child("vice_x_length"));
    double y_len = decode_json<double>(p.get_child("vice_y_length"));
    double base_height = decode_json<double>(p.get_child("vice_base_height"));
    double top_height = decode_json<double>(p.get_child("vice_top_height"));
    double clamp_width = decode_json<double>(p.get_child("vice_clamp_width"));
    double max_jaw_width = decode_json<double>(p.get_child("vice_max_jaw_width"));
    point base_norm = decode_json<point>(p.get_child("base_normal"));
    point top_clamp_norm = decode_json<point>(p.get_child("top_clamp_normal"));
    
    double protective_base_plate_height =
      decode_json<double>(p.get_child("vice_protective_base_plate_height"));
    return vice(pos, x_len, y_len, base_height, top_height, clamp_width, max_jaw_width, protective_base_plate_height, base_norm, top_clamp_norm);
  }
  plan_inputs parse_inputs_json(const std::string& s);

  ptree encode_json(const labeled_operation_params& op);

  ptree encode_json(const operation_params& op);

  ptree encode_json(const operation_range& op_range);

  tool_end decode_tool_end_json(const ptree& p);

  operation_params decode_json_params(const ptree& p);

  std::vector<operation_params> decode_params(const ptree& p);

  ptree encode_json(const simulation_log& sim_log);
  
  ptree encode_params(const std::vector<operation_params>& elems);

  void write_as_json(const std::vector<operation_params>& all_params);

  std::vector<operation_params>
  read_operation_params_json(const std::string& dir_name);
  
}

#endif
