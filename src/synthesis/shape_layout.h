#ifndef GCA_SHAPE_READER_H
#define GCA_SHAPE_READER_H

#include "geometry/b_spline.h"
#include "gcode/cut.h"
#include "gcode/hole_punch.h"
#include "gcode/machine.h"

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
  protected:
    double plunge_feedrate;

    bool set_plunge_feedrate;
    
  public:
    double safe_height;
    double material_depth;
    double cut_depth;
    double push_depth;
    point start_loc;
    point start_orient;
    double default_feedrate;
    double machine_z_zero;
    double max_orientation_diff;
    bool machine_z_is_inverted;
    bool set_default_feedrate;
    machine_name target_machine;
    ToolOptions tools;

    cut_params() {
      set_default_feedrate = false;
      set_plunge_feedrate = false;
      machine_z_is_inverted = false;
      machine_z_zero = 0.0;
      max_orientation_diff = 15;
    }

    void set_plunge_feed(const double f) {
      plunge_feedrate = f;
      set_plunge_feedrate = true;
    }

    bool plunge_feed_is_set() const { return set_plunge_feedrate; }

    double plunge_feed() const {
      return plunge_feedrate;
    }

  };

}

#endif
