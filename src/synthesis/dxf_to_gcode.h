#ifndef GCA_DXF_TO_GCODE_H
#define GCA_DXF_TO_GCODE_H

#include "core/gprog.h"
#include "synthesis/align_blade.h"
#include "synthesis/output.h"

namespace gca {

  typedef vector<cut*> cut_group;
  
  class cut_params {
  public:
    double safe_height;
    double material_depth;
    double cut_depth;
    double push_depth;
    point start_loc;
    point start_orient;
  };

  class shape_layout {
  public:
    vector<cut*> lines;
    vector<hole_punch*> holes;
    vector<b_spline*> splines;

  shape_layout(const vector<cut*>& linesp,
	       const vector<hole_punch*>& holesp,
	       const vector<b_spline*>& splinesp) :
    lines(linesp), holes(holesp), splines(splinesp) {}
  };

  class toolpath {
  public:
    int tool_no;
    vector<cut_group> cut_groups;
  };

  toolpath drill_toolpath(const shape_layout& shapes_to_cut,
			  cut_params params);

  void group_adjacent_cuts(const vector<cut*>& cuts,
			   vector<cut_group>& cut_groups,
			   double max_orientation_change);

  shape_layout read_dxf(const char* file);

  gprog* dxf_to_gcode(char* file, cut_params params);

  gprog* shape_layout_to_gcode(const shape_layout& shapes_to_cut,
			       cut_params params);
}

#endif
