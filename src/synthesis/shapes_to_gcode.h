#ifndef GCA_DXF_TO_GCODE_H
#define GCA_DXF_TO_GCODE_H

#include "synthesis/align_blade.h"
#include "synthesis/output.h"
#include "synthesis/shape_layout.h"
#include "synthesis/toolpath.h"

namespace gca {

  vector<block> shape_layout_to_gcode(const shape_layout& shapes_to_cut,
				      const cut_params& params);

  vector<block> cuts_to_gcode(const vector<cut*>& cuts,
			      const cut_params& params);  
  
  string shape_layout_to_gcode_string(const shape_layout& shapes_to_cut,
				      const cut_params& params);

  string cuts_to_gcode_string(const vector<cut*>& cuts,
			      const cut_params& params);

}

#endif
