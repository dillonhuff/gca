#pragma once

#include "synthesis/clamp_orientation.h"
#include "synthesis/fabrication_plan.h"
#include "process_planning/surface_planning.h"

namespace gca {

  struct dir_fixture {
    clamp_orientation orient;
    homogeneous_transform placement;
    vice v;
  };

  struct axis_fixture {
    boost::optional<dir_fixture> positive;
    boost::optional<dir_fixture> negative;
  };

  axis_fixture build_axis_fixture(const fixtures& f,
				  const major_axis_decomp& d);

}
