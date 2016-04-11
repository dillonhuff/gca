#ifndef GCA_PARSE_STL_H
#define GCA_PARSE_STL_H

#include "geometry/triangle.h"

namespace gca {

  struct stl_data {
    string name;
    vector<triangle> triangles;

    stl_data(string namep) : name(namep) {}
  };

  stl_data parse_stl(const string& stl_path);

}

#endif
