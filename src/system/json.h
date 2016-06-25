#ifndef GCA_JSON_H
#define GCA_JSON_H

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

  template<typename T>
  T decode_json(const ptree& p) {
    assert(false);
  }

  template<>
  double decode_json(const ptree& p) {
    std::string s = p.get<std::string>("");
    return std::stod(s);
  }

  template<>
  unsigned decode_json(const ptree& p) {
    std::string s = p.get<std::string>("");
    return std::stoul(s);
  }

  // TODO: Actual decoding for elements
  template<>
  tool decode_json(const ptree& p) {
    double diam = decode_json<double>(p.get_child("tool_diameter"));
    double length = decode_json<double>(p.get_child("tool_length"));
    unsigned num_flutes = decode_json<unsigned>(p.get_child("tool_num_flutes"));
    return tool(diam, length, num_flutes, FLAT_NOSE);
  }

  template<>
  vice decode_json(const ptree& p) {
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
  point decode_json(const ptree& p) {
    std::vector<double> coords = decode_vector<double>(p.get_child(""));
    return point(coords[0], coords[1], coords[2]);
  }

  
}

#endif
