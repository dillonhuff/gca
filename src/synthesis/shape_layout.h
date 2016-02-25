#ifndef GCA_SHAPE_READER_H
#define GCA_SHAPE_READER_H


#include "geometry/b_spline.h"
#include "synthesis/cut.h"
#include "synthesis/hole_punch.h"
#include "synthesis/machine.h"

namespace gca {

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

  enum ToolOptions { DRILL_ONLY = 0,
		     DRAG_KNIFE_ONLY, 
		     DRILL_AND_DRAG_KNIFE };
  
  class cut_params {
  public:
    double safe_height;
    double material_depth;
    double cut_depth;
    double push_depth;
    point start_loc;
    point start_orient;
    double default_feedrate;
    double machine_z_zero;
    bool machine_z_is_inverted;
    bool one_pass_only;
    bool set_default_feedrate;
    double pass_depth;
    machine_name target_machine;
    ToolOptions tools;

    cut_params() {
      one_pass_only = false;
      set_default_feedrate = false;
      machine_z_is_inverted = false;
      machine_z_zero = 0.0;
    }
  };

}

#endif
