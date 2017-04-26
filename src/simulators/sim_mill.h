#ifndef GCA_SIM_MILL_H
#define GCA_SIM_MILL_H

#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_res.h"
#include "gcode/cut.h"

namespace gca {

  class region bounding_region(double tool_diameter, box b, double material_height);
  double update_cut(const cut& c, class region& r, const mill_tool& t);
  double simulate_mill(const vector<cut*>& p, class region& r, const mill_tool& t);
  class region set_up_region(const vector<vector<cut*>>& paths, double tool_diameter);
  class region set_up_region_conservative(const vector<vector<cut*>>& paths, double tool_diameter);

  vector<point_update>
  update_cut_with_logging(const cut& c, class region& r, const mill_tool& t);
  

}

#endif
