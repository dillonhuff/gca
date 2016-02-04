#ifndef GCA_DXF_TO_GCODE_H
#define GCA_DXF_TO_GCODE_H

#include "core/gprog.h"
#include "synthesis/align_blade.h"
#include "synthesis/output.h"

namespace gca {

  class cut_params {
  public:
    double safe_height;
    double material_depth;
    double cut_depth;
    double push_depth;
    point start_loc;
    point start_orient;
  };

  gprog* dxf_to_gcode(char* file, cut_params params);
}

#endif
