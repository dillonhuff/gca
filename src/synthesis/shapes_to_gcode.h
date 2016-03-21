#ifndef GCA_DXF_TO_GCODE_H
#define GCA_DXF_TO_GCODE_H

#include "core/gprog.h"
#include "synthesis/align_blade.h"
#include "synthesis/output.h"
#include "synthesis/shape_layout.h"
#include "synthesis/toolpath.h"

namespace gca {

  string shape_layout_to_gcode_string(const shape_layout& shapes_to_cut,
				      const cut_params& params);

  string cuts_to_gcode_string(const vector<cut*>& cuts,
			      const cut_params& params);

}

#endif
