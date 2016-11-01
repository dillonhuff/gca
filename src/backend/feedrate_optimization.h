#pragma once

#include "simulators/sim_mill.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  class region bounding_region(const flat_region& mregion);

  void optimize_feedrates_by_MRR_simulation(const flat_region& r,
					    std::vector<toolpath>& toolpaths,
					    const double machine_hp,
					    const double material_unit_hp);

}
