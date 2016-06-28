#ifndef GCA_WORKPIECE_CLIPPING_H
#define GCA_WORKPIECE_CLIPPING_H

#include "synthesis/fixture_analysis.h"
#include "synthesis/workpiece.h"

namespace gca {

  std::vector<gcode_program>
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& fixes);


}

#endif
