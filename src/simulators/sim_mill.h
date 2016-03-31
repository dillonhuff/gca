#ifndef GCA_SIM_MILL_H
#define GCA_SIM_MILL_H

#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_res.h"
#include "synthesis/cut.h"

namespace gca {

  double simulate_mill(const vector<cut*>& p, region& r, const mill_tool& t);

}

#endif
