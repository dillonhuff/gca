#ifndef GCA_SIM_MILL_H
#define GCA_SIM_MILL_H

#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_res.h"
#include "gcode/cut.h"

namespace gca {

  double update_cut(const cut& c, region& r, const mill_tool& t);
  double simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t);
  region set_up_region(const vector<vector<cut*>>& paths, double tool_diameter);
  region set_up_region_conservative(const vector<vector<cut*>>& paths, double tool_diameter);

}

#endif
