#ifndef GCA_DXF_TO_GCODE_H
#define GCA_DXF_TO_GCODE_H

#include "core/gprog.h"
#include "synthesis/align_blade.h"
#include "synthesis/output.h"
#include "synthesis/shape_layout.h"
#include "synthesis/toolpath.h"

namespace gca {

  toolpath drill_toolpath(const vector<hole_punch*>& holes,
			  cut_params params);

  gprog* shape_layout_to_gcode(const shape_layout& shapes_to_cut,
			       cut_params params);
}

#endif
