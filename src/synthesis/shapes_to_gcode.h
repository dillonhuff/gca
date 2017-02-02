#ifndef GCA_DXF_TO_GCODE_H
#define GCA_DXF_TO_GCODE_H

#include "backend/align_blade.h"
#include "backend/output.h"
#include "synthesis/shape_layout.h"
#include "backend/toolpath.h"

namespace gca {

  vector<cut*> insert_transitions(const vector<cut*>& cuts,
				  const cut_params& params);

  void set_feedrates(vector<cut*>& cuts,
		     const cut_params& params);

  bool cuts_are_adjacent(const vector<cut*>& cuts);
  
  vector<block> shape_layout_to_gcode(const shape_layout& shapes_to_cut,
				      const cut_params& params);

  vector<block> cuts_to_gcode(const vector<cut*>& cuts,
			      const cut_params& params);  
  
  string shape_layout_to_gcode_string(const shape_layout& shapes_to_cut,
				      const cut_params& params);

  string cuts_to_gcode_string(const vector<cut*>& cuts,
			      const cut_params& params);

  vector<block> cuts_to_gcode_no_transitions(const vector<cut*>& all_cuts,
					     const cut_params& params);
  
}

#endif
