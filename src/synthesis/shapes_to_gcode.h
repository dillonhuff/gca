#ifndef GCA_DXF_TO_GCODE_H
#define GCA_DXF_TO_GCODE_H

#include "core/gprog.h"
#include "synthesis/align_blade.h"
#include "synthesis/shape_layout.h"
#include "synthesis/output.h"

namespace gca {

  typedef vector<cut*> cut_group;
  
  class toolpath {
  public:
    int tool_no;
    vector<cut_group> cut_groups;
  };

  toolpath drill_toolpath(const vector<hole_punch*>& holes,
			  cut_params params);

  void group_adjacent_cuts(const vector<cut*>& cuts,
			   vector<cut_group>& cut_groups,
			   double max_orientation_change);

  gprog* shape_layout_to_gcode(const shape_layout& shapes_to_cut,
			       cut_params params);
}

#endif
